/*
 *  ESCCMD:   ESC DSHOT command packets formating API
 *
 *  Note:     Best viewed using Arduino IDE with tab space = 2
 *
 *  Authors:  Arda Yiğit and Jacques Gangloff
 *  Date:     May 2019
 */

// Includes
#include <Arduino.h>
#include "DSHOT.h"
#include "ESCCMD.h"

// Error handling
#define ESCCMD_ERROR( code )    { ESCCMD_last_error[i] = code; return code; }

//
//  Global variables
//
uint8_t             ESCCMD_n;                               // Number of initialized outputs

volatile uint16_t   ESCCMD_state[ESCCMD_MAX_ESC];           // Current state of the cmd subsystem
uint16_t            ESCCMD_CRC_errors[ESCCMD_MAX_ESC];      // Overall number of CRC error since start
int16_t             ESCCMD_last_error[ESCCMD_MAX_ESC];      // Last error code
uint16_t            ESCCMD_cmd[ESCCMD_MAX_ESC];             // Last command
uint16_t            ESCCMD_throttle_wd[ESCCMD_MAX_ESC];     // Throttle watchdog counter
uint8_t             ESCCMD_tlm_deg[ESCCMD_MAX_ESC];         // ESC temperature (°C)
uint16_t            ESCCMD_tlm_volt[ESCCMD_MAX_ESC];        // Voltage of the ESC power supply (0.01V)
uint16_t            ESCCMD_tlm_amp[ESCCMD_MAX_ESC];         // ESC current (0.01A)
uint16_t            ESCCMD_tlm_mah[ESCCMD_MAX_ESC];         // ESC consumption (mAh)
uint16_t            ESCCMD_tlm_rpm[ESCCMD_MAX_ESC];         // ESC electrical rpm (100rpm)
uint8_t             ESCCMD_tlm[ESCCMD_MAX_ESC];             // Set to 1 when asking for telemetry
uint8_t             ESCCMD_tlm_pend[ESCCMD_MAX_ESC];        // Flag indicating a pending telemetry data request
uint8_t             ESCCMD_tlm_valid[ESCCMD_MAX_ESC];       // Flag indicating the validity of telemetry data

volatile uint16_t   ESCCMD_tic_pend = 0;                    // Number of timer tic waiting for ackowledgement
volatile uint8_t    ESCCMD_init_flag = 0;                   // Subsystem initialization flag
volatile uint8_t    ESCCMD_timer_flag = 0;                  // Periodic loop enable/disable flag

IntervalTimer       ESCCMD_timer;                           // Timer object
HardwareSerial      ESCCMD_serial[ESCCMD_NB_UART] = {       // Array of Serial objects
                                                Serial1,
                                                Serial2,
                                                Serial3,
                                                Serial4,
                                                Serial5,
                                                Serial6 };
//
//  Initialization
//
void ESCCMD_init( uint8_t n )  {
  static int i;

  if ( ESCCMD_init_flag )
    return;

  if ( n <= ESCCMD_MAX_ESC )
    ESCCMD_n = n;
  else
    ESCCMD_n = ESCCMD_MAX_ESC;

  // Initialize data arrays to zero
  for ( i = 0; i < ESCCMD_n; i++ ) {
    ESCCMD_state[i]       = 0;
    ESCCMD_CRC_errors[i]  = 0;
    ESCCMD_last_error[i]  = 0;
    ESCCMD_cmd[i]         = 0;
    ESCCMD_throttle_wd[i] = 0;
    ESCCMD_tlm_deg[i]     = 0;
    ESCCMD_tlm_volt[i]    = 0;
    ESCCMD_tlm_amp[i]     = 0;
    ESCCMD_tlm_mah[i]     = 0;
    ESCCMD_tlm_rpm[i]     = 0;
    ESCCMD_tlm[i]         = 0;
    ESCCMD_tlm_pend[i]    = 0;
    ESCCMD_tlm_valid[i]   = 0;
  }

  // Initialize DSHOT generation subsystem
  DSHOT_init( ESCCMD_n );

  // Initialize telemetry UART channels
  for ( i = 0; i < ESCCMD_n; i++ )
    ESCCMD_serial[i].begin( ESCCMD_TLM_UART_SPEED );

  // Set the initialization flag
  ESCCMD_init_flag = 1;
}

//
//  Arm all ESCs
//
//  Return values: see defines
//
int ESCCMD_arm_all( void )  {
  static int i, k;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;

  // Check if all the ESCs are in the initial state
  for ( i = 0; i < ESCCMD_n; i++ )
    if ( ESCCMD_state[i] & ESCCMD_STATE_ARMED )
      ESCCMD_ERROR( ESCCMD_ERROR_SEQ )

  // Define stop command
  for ( i = 0; i < ESCCMD_n; i++ )  {
    ESCCMD_cmd[i] = DSHOT_CMD_MOTOR_STOP;
    ESCCMD_tlm[i] = 0;
  }

  // Send command ESCCMD_CMD_ARMING_REP times
  for ( i = 0; i < ESCCMD_CMD_ARMING_REP; i++ )  {

    // Send DSHOT signal to all ESCs
    if ( DSHOT_send( ESCCMD_cmd, ESCCMD_tlm ) ) {
      for ( k = 0; k < ESCCMD_n; k++ )
        ESCCMD_last_error[k] = ESCCMD_ERROR_DSHOT;
      return ESCCMD_ERROR_DSHOT;
    }

    // Wait some time
    delayMicroseconds( 2 * ESCCMD_CMD_DELAY );
  }

  // Set the arming flag
  for ( i = 0; i < ESCCMD_n; i++ )
    ESCCMD_state[i] |= ESCCMD_STATE_ARMED;

  return 0;
}

//
//  Make motor number k beep
//
//  Return values: see defines
//
int ESCCMD_beep( uint8_t i, uint16_t tone )  {
  static int k;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;

  // Check if motor is within range
  if ( i >= ESCCMD_n )
    return ESCCMD_ERROR_PARAM;

  // Check if tone value is within valid range
  if ( ( tone < DSHOT_CMD_BEACON1 ) || ( tone > DSHOT_CMD_BEACON5 ) )
    ESCCMD_ERROR( ESCCMD_ERROR_PARAM )

  // Define beep command only for motor n
  for ( k = 0; k < ESCCMD_n; k++ )  {
    ESCCMD_cmd[k] = DSHOT_CMD_MOTOR_STOP;
    ESCCMD_tlm[k] = 1;
  }
  ESCCMD_cmd[i] = tone;

  // Send command a number of times corresponding to the desired duration
  for ( k = 0; k < ( ( ESCCMD_BEEP_DURATION * 1000 ) / ESCCMD_CMD_DELAY ); k++ )  {

    // Send DSHOT signal to all ESCs
    if ( DSHOT_send( ESCCMD_cmd, ESCCMD_tlm ) )
      ESCCMD_ERROR( ESCCMD_ERROR_DSHOT )

    // Wait some time
    delayMicroseconds( ESCCMD_CMD_DELAY );
  }

  return 0;
}

//
//  Activate 3D mode
//
//  Return values: see defines
//
int ESCCMD_3D_on( void )  {
  static int i, k;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;

  for ( i = 0; i < ESCCMD_n; i++ )  {
    // Check if ESCs are stopped
    if ( ESCCMD_state[i] & ESCCMD_STATE_START )
      ESCCMD_ERROR( ESCCMD_ERROR_SEQ )
  }

  // Define 3D on command
  for ( i = 0; i < ESCCMD_n; i++ )  {
    ESCCMD_cmd[i] = DSHOT_CMD_3D_MODE_ON;
    ESCCMD_tlm[i] = 1;
  }

  // Send command ESCCMD_CMD_REPETITION times
  for ( i = 0; i < ESCCMD_CMD_REPETITION; i++ )  {

    // Send DSHOT signal to all ESCs
    if ( DSHOT_send( ESCCMD_cmd, ESCCMD_tlm ) ) {
      for ( k = 0; k < ESCCMD_n; k++ )
        ESCCMD_last_error[k] = ESCCMD_ERROR_DSHOT;
      return ESCCMD_ERROR_DSHOT;
    }

    // Wait some time
    delayMicroseconds( ESCCMD_CMD_DELAY );
  }

  // Define save settings command
  for ( i = 0; i < ESCCMD_n; i++ )  {
    ESCCMD_cmd[i] = DSHOT_CMD_SAVE_SETTINGS;
    ESCCMD_tlm[i] = 1;
  }

  // Send command ESCCMD_CMD_REPETITION times
  for ( i = 0; i < ESCCMD_CMD_REPETITION; i++ )  {

    // Send DSHOT signal to all ESCs
    if ( DSHOT_send( ESCCMD_cmd, ESCCMD_tlm ) ) {
      for ( k = 0; k < ESCCMD_n; k++ )
        ESCCMD_last_error[k] = ESCCMD_ERROR_DSHOT;
      return ESCCMD_ERROR_DSHOT;
    }

    // Wait some time
    delayMicroseconds( ESCCMD_CMD_DELAY );
  }

  // Set the 3D mode flag
  for ( i = 0; i < ESCCMD_n; i++ )
    ESCCMD_state[i] |= ESCCMD_STATE_3D;

  // Minimum delay before next command
  delayMicroseconds( ESCCMD_CMD_SAVE_DELAY );

  // ESC is disarmed after previous delay
  for ( i = 0; i < ESCCMD_n; i++ )
    ESCCMD_state[i] &= ~(ESCCMD_STATE_ARMED);

  return 0;
}

//
//  Deactivate 3D mode
//
//  Return values: see defines
//
int ESCCMD_3D_off( void )  {
  static int i, k;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;

  for ( i = 0; i < ESCCMD_n; i++ )  {
    // Check if ESCs are stopped
    if ( ESCCMD_state[i] & ESCCMD_STATE_START )
      ESCCMD_ERROR( ESCCMD_ERROR_SEQ )
  }

  // Define 3D off command
  for ( i = 0; i < ESCCMD_n; i++ )  {
    ESCCMD_cmd[i] = DSHOT_CMD_3D_MODE_OFF;
    ESCCMD_tlm[i] = 1;
  }

  // Send command ESCCMD_CMD_REPETITION times
  for ( i = 0; i < ESCCMD_CMD_REPETITION; i++ )  {

    // Send DSHOT signal to all ESCs
    if ( DSHOT_send( ESCCMD_cmd, ESCCMD_tlm ) ) {
      for ( k = 0; k < ESCCMD_n; k++ )
        ESCCMD_last_error[k] = ESCCMD_ERROR_DSHOT;
      return ESCCMD_ERROR_DSHOT;
    }

    // Wait some time
    delayMicroseconds( ESCCMD_CMD_DELAY );
  }

  // Define save settings command
  for ( i = 0; i < ESCCMD_n; i++ )  {
    ESCCMD_cmd[i] = DSHOT_CMD_SAVE_SETTINGS;
    ESCCMD_tlm[i] = 1;
  }

  // Send command ESCCMD_CMD_REPETITION times
  for ( i = 0; i < ESCCMD_CMD_REPETITION; i++ )  {

    // Send DSHOT signal to all ESCs
    if ( DSHOT_send( ESCCMD_cmd, ESCCMD_tlm ) ) {
      for ( k = 0; k < ESCCMD_n; k++ )
        ESCCMD_last_error[k] = ESCCMD_ERROR_DSHOT;
      return ESCCMD_ERROR_DSHOT;
    }

    // Wait some time
    delayMicroseconds( ESCCMD_CMD_DELAY );
  }

  // Clear the 3D mode flag
  for ( i = 0; i < ESCCMD_n; i++ )
    ESCCMD_state[i] &= ~(ESCCMD_STATE_3D);

  // Minimum delay before next command
  delayMicroseconds( ESCCMD_CMD_SAVE_DELAY );

  // ESC is disarmed after previous delay
  for ( i = 0; i < ESCCMD_n; i++ )
    ESCCMD_state[i] &= ~(ESCCMD_STATE_ARMED);

  return 0;
}

//
//  Start periodic loop. ESC should be armed.
//
//  Return values: see defines
//
int ESCCMD_start_timer( void )  {
  static int i;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;

  // Check if timer already started
  if ( ESCCMD_timer_flag )
    return ESCCMD_ERROR_SEQ;

  // Checks
  for ( i = 0; i < ESCCMD_n; i++ )  {
    // Check if all the ESCs are armed
    if ( !( ESCCMD_state[i] & ESCCMD_STATE_ARMED ) )
      ESCCMD_ERROR( ESCCMD_ERROR_SEQ )

    // Check if ESCs are stopped
    if ( ESCCMD_state[i] & ESCCMD_STATE_START )
      ESCCMD_ERROR( ESCCMD_ERROR_SEQ )
  }

  // Initialize ESC structure
  for ( i = 0; i < ESCCMD_n; i++ )  {
    ESCCMD_cmd[i] = DSHOT_CMD_MOTOR_STOP;
    ESCCMD_tlm[i] = 1;
    ESCCMD_tlm_pend[i] = 0;
    ESCCMD_throttle_wd[i] = ESCCMD_THWD_LEVEL;
  }

  ESCCMD_tic_pend = 0;

  // Initialize timer
  ESCCMD_timer.begin( ESCCMD_ISR_timer, ESCCMD_TIMER_PERIOD );

  return 0;
}

//
//  Stop periodic loop. ESC should be armed.
//
//  Return values: see defines
//
int ESCCMD_stop_timer( void )  {
  static int i;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;

  // Check if timer started
  if ( !ESCCMD_timer_flag )
    return ESCCMD_ERROR_SEQ;

  // Stop timer
  ESCCMD_timer.end();
  ESCCMD_timer_flag = 0;

  // Update ESC state
  for ( i = 0; i < ESCCMD_n; i++ )  {
    ESCCMD_cmd[i] = DSHOT_CMD_MOTOR_STOP;
    ESCCMD_tlm[i] = 0;
    ESCCMD_state[i] &= ~( ESCCMD_STATE_ARMED | ESCCMD_STATE_START );
  }

  return 0;
}

//
//  Define throttle of ESC number i:
//    Default mode: 0 -> 1999
//    3D mode     : -999 -> 999
//
int ESCCMD_throttle( uint8_t i, int16_t throttle ) {
  static uint8_t local_state;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;
  
  // Check if motor is within range
  if ( i >= ESCCMD_n )
    return ESCCMD_ERROR_PARAM;

  // Define a local copy of the state
  noInterrupts();
  local_state = ESCCMD_state[i];
  interrupts();

  // Check if ESC is armed
  if ( !( local_state & ESCCMD_STATE_ARMED ) )
    ESCCMD_ERROR( ESCCMD_ERROR_SEQ )

  // Define throttle depending on the mode
  if ( local_state & ESCCMD_STATE_3D )  {
    // Check limits
    if ( ( throttle < ESCCMD_MIN_3D_THROTTLE ) || ( throttle > ESCCMD_MAX_3D_THROTTLE ))
      ESCCMD_ERROR( ESCCMD_ERROR_PARAM )

    // 3D mode
    // 48 - 1047    : positive direction (48 slowest)
    // 1048 - 2047  : negative direction (1048 slowest)
    if ( throttle >= 0 )
      ESCCMD_cmd[i] = DSHOT_CMD_MAX + 1 + throttle;
    else
      ESCCMD_cmd[i] = DSHOT_CMD_MAX + 1 + ESCCMD_MAX_3D_THROTTLE - throttle;
  }
  else {

    // Check limits
    if ( ( throttle < 0 ) || ( throttle > ESCCMD_MAX_THROTTLE ))
      ESCCMD_ERROR( ESCCMD_ERROR_PARAM )

    // Default mode
    ESCCMD_cmd[i] = DSHOT_CMD_MAX + 1 + throttle;
  }

  // Switch start mode on only if needed
  if ( !( local_state & ESCCMD_STATE_START ) ) {
    noInterrupts();
    ESCCMD_state[i] |= ESCCMD_STATE_START;
    interrupts();
    ESCCMD_tlm[i] = 1;
  }

  // Reset the throttle watchdog
  ESCCMD_throttle_wd[i] = 0;

  return 0;
}

//
//  Stop motor number i
//
int ESCCMD_stop( uint8_t i ) {
  static uint8_t local_state;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;
  
  // Check if motor is within range
  if ( i >= ESCCMD_n )
    return ESCCMD_ERROR_PARAM;

  // Define a local copy of the state
  noInterrupts();
  local_state = ESCCMD_state[i];
  interrupts();

  // Check if ESC is armed
  if ( !( local_state & ESCCMD_STATE_ARMED ) )
    ESCCMD_ERROR( ESCCMD_ERROR_SEQ )

  // Set command to stop
  ESCCMD_cmd[i] = DSHOT_CMD_MOTOR_STOP;

  // Switch start mode off only if needed
  if ( local_state & ESCCMD_STATE_START  ) {
    noInterrupts();
    ESCCMD_state[i] &= ~ESCCMD_STATE_START;
    interrupts();
    ESCCMD_tlm[i] = 0;
  }

  return 0;
}

//
//  Return last error code of motor number i
//
int ESCCMD_read_err( uint8_t i, int *err )  {

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;
    
  // Check if motor is within range
  if ( i >= ESCCMD_n )
    return ESCCMD_ERROR_PARAM;
  

  *err = (int)ESCCMD_last_error[i];

  return 0;
}

//
//  Return last command code of motor number i
//
int ESCCMD_read_cmd( uint8_t i, int *cmd )  {

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;
    
  // Check if motor is within range
  if ( i >= ESCCMD_n )
    return ESCCMD_ERROR_PARAM;
  

  *cmd = (int)ESCCMD_cmd[i];

  return 0;
}

//
//  Read temperature of motor number i
//  Unit is degree Celsius
//
int ESCCMD_read_deg( uint8_t i, float *deg )  {
  static uint8_t local_state;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;
  
  // Check if motor is within range
  if ( i >= ESCCMD_n )
    return ESCCMD_ERROR_PARAM;

  // Define a local copy of the state
  noInterrupts();
  local_state = ESCCMD_state[i];
  interrupts();

  // Check if ESC is armed
  if ( !( local_state & ESCCMD_STATE_ARMED ) )
    ESCCMD_ERROR( ESCCMD_ERROR_SEQ )

  // Check if telemetry is valid
  if ( ESCCMD_tlm_valid[i] )  {
    *deg = (float)ESCCMD_tlm_deg[i];
  }
  else {
    ESCCMD_ERROR( ESCCMD_ERROR_TLM_INVAL )
  }

  return 0;
}

//
//  Read dc power supply voltage of motor number i
//  Unit is Volt
//
int ESCCMD_read_volt( uint8_t i, float *volt )  {
  static uint8_t local_state;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;
  
  // Check if motor is within range
  if ( i >= ESCCMD_n )
    return ESCCMD_ERROR_PARAM;

  // Define a local copy of the state
  noInterrupts();
  local_state = ESCCMD_state[i];
  interrupts();

  // Check if ESC is armed
  if ( !( local_state & ESCCMD_STATE_ARMED ) )
    ESCCMD_ERROR( ESCCMD_ERROR_SEQ )

  // Check if telemetry is valid
  if ( ESCCMD_tlm_valid[i] )  {
    *volt = (float)ESCCMD_tlm_volt[i] * 0.01;
  }
  else {
    ESCCMD_ERROR( ESCCMD_ERROR_TLM_INVAL )
  }

  return 0;
}

//
//  Read current of motor number i
//  Unit is Ampere
//
int ESCCMD_read_amp( uint8_t i, float *amp )  {
  static uint8_t local_state;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;
  
  // Check if motor is within range
  if ( i >= ESCCMD_n )
    return ESCCMD_ERROR_PARAM;

  // Define a local copy of the state
  noInterrupts();
  local_state = ESCCMD_state[i];
  interrupts();

  // Check if ESC is armed
  if ( !( local_state & ESCCMD_STATE_ARMED ) )
    ESCCMD_ERROR( ESCCMD_ERROR_SEQ )

  // Check if telemetry is valid
  if ( ESCCMD_tlm_valid[i] )  {
    *amp = (float)ESCCMD_tlm_amp[i] * 0.01;
  }
  else {
    ESCCMD_ERROR( ESCCMD_ERROR_TLM_INVAL )
  }

  return 0;
}

//
//  Read consumption of motor number i
//  Unit is milli Ampere.hour 
//
int ESCCMD_read_mah( uint8_t i, float *mah )  {
  static uint8_t local_state;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;
  
  // Check if motor is within range
  if ( i >= ESCCMD_n )
    return ESCCMD_ERROR_PARAM;

  // Define a local copy of the state
  noInterrupts();
  local_state = ESCCMD_state[i];
  interrupts();

  // Check if ESC is armed
  if ( !( local_state & ESCCMD_STATE_ARMED ) )
    ESCCMD_ERROR( ESCCMD_ERROR_SEQ )

  // Check if telemetry is valid
  if ( ESCCMD_tlm_valid[i] )  {
    *mah = (float)ESCCMD_tlm_mah[i];
  }
  else {
    ESCCMD_ERROR( ESCCMD_ERROR_TLM_INVAL )
  }

  return 0;
}

//
//  Read shaft rotational velocity of motor number i
//  Unit is round per minute
//  The sign of the measurement depends on the last throttle sign
//
int ESCCMD_read_rpm( uint8_t i, float *rpm )  {
  static uint8_t local_state;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;
  
  // Check if motor is within range
  if ( i >= ESCCMD_n )
    return ESCCMD_ERROR_PARAM;

  // Define a local copy of the state
  noInterrupts();
  local_state = ESCCMD_state[i];
  interrupts();

  // Check if ESC is armed
  if ( !( local_state & ESCCMD_STATE_ARMED ) )
    ESCCMD_ERROR( ESCCMD_ERROR_SEQ )

  // Check if telemetry is valid
  if ( ESCCMD_tlm_valid[i] )  {
    // Check current mode
    if ( local_state & ESCCMD_STATE_3D )  {
      // 3D mode
      // 48 - 1047    : positive direction (48 slowest)
      // 1048 - 2047  : negative direction (1048 slowest)
      if ( ESCCMD_cmd[i] > DSHOT_CMD_MAX + 1 + ESCCMD_MAX_3D_THROTTLE )
        *rpm = (float)( -ESCCMD_tlm_rpm[i] );  // 3D mode negative direction
      else
        *rpm = (float)( ESCCMD_tlm_rpm[i] );   // 3D mode positive direction
    }
    else {
      // Default mode
      *rpm = (float)( ESCCMD_tlm_rpm[i] );
    }

    // Conversion in rpm
    *rpm *= 100.0 / ( ESCCMD_TLM_NB_POLES / 2 );
  }
  else {
    ESCCMD_ERROR( ESCCMD_ERROR_TLM_INVAL )
  }

  return 0;
}

//
//  This routine should be called within the main loop
//
int ESCCMD_tic( void )  {
  static int      i, j, ret;
  static uint8_t  bufferTlm[ESCCMD_TLM_LENGTH];
  static uint16_t local_tic_pend;

  // Defaults to no error
  ret = 0;

  // Read telemetry if packets are pending
  for ( i = 0; i < ESCCMD_n; i++ )  {

    // Check number of telemetry packet pendng
    if ( ESCCMD_tlm_pend[i] > ESCCMD_TLM_MAX_PEND ) {
      ESCCMD_last_error[i] = ESCCMD_ERROR_TLM_PEND;
      ret = ESCCMD_ERROR_TLM_PEND;
    }

    // Process telemetry packets if available
    if ( ESCCMD_tlm_pend[i] ) {
      // Update pending packet counter
        ESCCMD_tlm_pend[i]--;

      // Check if a complete packet has arrived
      if ( ESCCMD_serial[i].available( ) >= ESCCMD_TLM_LENGTH )  {

        // Read packet
        for ( j = 0; j < ESCCMD_TLM_LENGTH; j++ )
          bufferTlm[j] = ESCCMD_serial[i].read( );

        // If a packet has arrived, process it
        ESCCMD_tlm_deg[i]     =   bufferTlm[0];
        ESCCMD_tlm_volt[i]    = ( bufferTlm[1] << 8 ) | bufferTlm[2];
        ESCCMD_tlm_amp[i]     = ( bufferTlm[3] << 8 ) | bufferTlm[4];
        ESCCMD_tlm_mah[i]     = ( bufferTlm[5] << 8 ) | bufferTlm[6];
        ESCCMD_tlm_rpm[i]     = ( bufferTlm[7] << 8 ) | bufferTlm[8];
        ESCCMD_tlm_valid[i]   = ( bufferTlm[9] == ESCCMD_crc8( bufferTlm, ESCCMD_TLM_LENGTH - 1 ) );

        // If crc is invalid, increment crc error counter
        // and flush UART buffer
        if ( !ESCCMD_tlm_valid[i] ) {

          ESCCMD_CRC_errors[i]++;

          ret = ESCCMD_ERROR_TLM_CRC;
          ESCCMD_last_error[i] = ESCCMD_ERROR_TLM_CRC;

          // Wait for last out of sync bytes to come in
          for ( j = 0; j < ESCCMD_tlm_pend[i] + 1; j++ )
            delayMicroseconds( ESCCMD_TIMER_PERIOD );

          // Reset pending packet counter: all packets should be arrived
          ESCCMD_tlm_pend[i] = 0;

          // Flush UART incoming buffer
          ESCCMD_serial[i].clear( );
        }
        else {
          // Make some verifications on the telemetry

          // Check for overtheating of the ESC
          if ( ESCCMD_tlm_deg[i] >= ESCCMD_TLM_MAX_TEMP ) {
            ESCCMD_last_error[i] = ESCCMD_ERROR_TLM_TEMP;
            ret = ESCCMD_ERROR_TLM_TEMP;
          }

          // Check for excessive CRC errors
          if ( ESCCMD_CRC_errors[i] >= ESCCMD_TLM_MAX_CRC_ERR ) {
            ESCCMD_last_error[i] = ESCCMD_ERROR_TLM_CRCMAX;
            ret = ESCCMD_ERROR_TLM_CRCMAX;
          }
        }
      }
    }
  }

  // Do something only if tics are pending
  noInterrupts();
  local_tic_pend = ESCCMD_tic_pend;
  interrupts();

  if ( local_tic_pend ) {

    // Acknowledgement of one timer clock event
    noInterrupts();
    ESCCMD_tic_pend--;
    interrupts();

    // If there is no error, inform caller that a tic occured
    if ( !ret )
      ret = ESCCMD_TIC_OCCURED;

    // Check if everything is initialized
    if ( !ESCCMD_init_flag )
      return ESCCMD_ERROR_INIT;

    // Check if all ESC are armed
    for ( i = 0; i < ESCCMD_n; i++ )
      if ( !( ESCCMD_state[i] & ESCCMD_STATE_ARMED ) )
        ESCCMD_ERROR( ESCCMD_ERROR_SEQ )

    // Throttle watchdog
    for ( i = 0; i < ESCCMD_n; i++ )  {
      if ( ESCCMD_throttle_wd[i] >= ESCCMD_THWD_LEVEL )  {
        // Watchdog triggered on ESC number i
        ESCCMD_cmd[i] = DSHOT_CMD_MOTOR_STOP;
        ESCCMD_tlm[i] = 0;
        noInterrupts();
        ESCCMD_state[i] &= ~( ESCCMD_STATE_START );
        interrupts();
      }
      else {
        ESCCMD_throttle_wd[i]++;
      }
    }

    // Send current command
    if ( DSHOT_send( ESCCMD_cmd, ESCCMD_tlm ) ) {
      for ( i = 0; i < ESCCMD_n; i++ )
        ESCCMD_last_error[i] = ESCCMD_ERROR_DSHOT;
      return ESCCMD_ERROR_DSHOT;
    }
    delayMicroseconds( ESCCMD_CMD_DELAY );

    // Update telemetry packet pending counter
    for ( i = 0; i < ESCCMD_n; i++ )
      if ( ESCCMD_tlm[i] )
        ESCCMD_tlm_pend[i]++;
  }

  return ret;
}

//
// crc8 calculation
//
uint8_t ESCCMD_update_crc8( uint8_t crc, uint8_t crc_seed ) {
  static uint8_t crc_u;

  crc_u = crc;
  crc_u ^= crc_seed;

  for ( int i = 0; i < 8; i++ ) {
    crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  }

  return crc_u;
}

uint8_t ESCCMD_crc8( uint8_t* buf, uint8_t buflen ) {
  static uint8_t crc;

  crc = 0;

  for ( int i = 0; i < buflen; i++ ) {
    crc = ESCCMD_update_crc8( buf[i], crc );
  }

  return crc;
}

//
//  Timer ISR
//
void ESCCMD_ISR_timer( void ) {
  static int i;

  // Check for maximum missed tics (ESC watchdog timer = 250ms on a KISS ESC)
  if ( ESCCMD_tic_pend >= ESCCMD_TIMER_MAX_MISS )  {

    // ESC watchdog switch to disarmed and stopped mode
    for ( i = 0; i < ESCCMD_n; i++ )
      ESCCMD_state[i] &= ~( ESCCMD_STATE_ARMED | ESCCMD_STATE_START );
  }
  else
    ESCCMD_tic_pend++;
}
