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

// ESC emulation
//#define ESCCMD_ESC_EMULATION                              // Uncomment to activate ESC emulation
#define ESCCMD_ESC_EMU_PKT_LOSS                             // Uncomment to emulate packet loss
#define ESCCMD_ESC_FRACTION_PKTLOSS       300               // One out of x packets lost

// Error handling
#define ESCCMD_ERROR( code )              { ESCCMD_last_error[i] = code; return code; }

//
//  Global variables
//
volatile uint8_t    ESCCMD_n;                               // Number of initialized outputs

volatile uint8_t    ESCCMD_state[ESCCMD_MAX_ESC];           // Current state of the cmd subsystem
uint16_t            ESCCMD_CRC_errors[ESCCMD_MAX_ESC];      // Overall number of CRC error since start
int8_t              ESCCMD_last_error[ESCCMD_MAX_ESC];      // Last error code
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
uint8_t             ESCCMD_tlm_lost_cnt[ESCCMD_MAX_ESC];    // Lost packet counter of telemetry data
uint64_t            ESCCMD_tic_counter = 0;                 // Counts the number of clock iterations

volatile uint16_t   ESCCMD_tic_pend = 0;                    // Number of timer tic waiting for ackowledgement
volatile uint8_t    ESCCMD_init_flag = 0;                   // Subsystem initialization flag
volatile uint8_t    ESCCMD_timer_flag = 0;                  // Periodic loop enable/disable flag

IntervalTimer       ESCCMD_timer;                           // Timer object
HardwareSerial*     ESCCMD_serial[ESCCMD_NB_UART] = {       // Array of Serial objects
                                                &Serial1,
                                                &Serial2,
                                                &Serial3,
                                                &Serial4,
                                                &Serial5,
                                                &Serial6 };
uint8_t             ESCCMD_bufferTlm[ESCCMD_NB_UART][ESCCMD_TLM_LENGTH];

#ifdef ESCCMD_ESC_EMULATION
#define             ESCCMD_EMU_TLM_MAX      5               // Max number of telemetry packets
#define             ESCCMD_EMU_TLM_DEG      25              // Nominal temperature (deg)
#define             ESCCMD_EMU_TLM_VOLT     1200            // Nominal voltage (0.01V)
#define             ESCCMD_EMU_TLM_AMP      2500            // Nominal current (0.01A)
#define             ESCCMD_EMU_TLM_MAH      1000            // Nominal consumption (mAh)
#define             ESCCMD_EMU_TLM_RPM      10              // Nominal electrical rpm (100rpm)
#define             ESCCMD_EMU_TLM_NOISE    3.0             // Percentge of emulated noise (%)
#define             ESCCMD_EMU_MOTOR_KV     2600            // Kv of the emulated motor

uint8_t             ESCCMD_tlm_emu_nb[ESCCMD_MAX_ESC] = {}; // Number of available packets

uint8_t             ESCCMD_tlm_emu_deg[ESCCMD_EMU_TLM_MAX][ESCCMD_MAX_ESC];
uint16_t            ESCCMD_tlm_emu_volt[ESCCMD_EMU_TLM_MAX][ESCCMD_MAX_ESC];
uint16_t            ESCCMD_tlm_emu_amp[ESCCMD_EMU_TLM_MAX][ESCCMD_MAX_ESC];
uint16_t            ESCCMD_tlm_emu_mah[ESCCMD_EMU_TLM_MAX][ESCCMD_MAX_ESC];
uint16_t            ESCCMD_tlm_emu_rpm[ESCCMD_EMU_TLM_MAX][ESCCMD_MAX_ESC];
#endif


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
    ESCCMD_tlm_lost_cnt[i]= 0;
  }

  // Initialize telemetry UART channels
  for ( i = 0; i < ESCCMD_n; i++ )  {
    ESCCMD_serial[i]->begin( ESCCMD_TLM_UART_SPEED );
  }

  // Initialize DSHOT generation subsystem
  DSHOT_init( ESCCMD_n );

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
//  Activate 3D mode
//
//  Return values: see defines
//
int ESCCMD_3D_on( void )  {
  static int i, k;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;
  
  // Check if timer is disabled
  if ( ESCCMD_timer_flag )
    return ESCCMD_ERROR_PARAM;

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

  // Flush incoming serial buffers due to a transcient voltage
  // appearing on Tx when saving, generating a 0xff serial byte
  for ( i = 0; i < ESCCMD_n; i++ )
    ESCCMD_serial[i]->clear( );

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
  
  // Check if timer is disabled
  if ( ESCCMD_timer_flag )
    return ESCCMD_ERROR_PARAM;

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

  // Flush incoming serial buffers due to a transcient voltage
  // appearing on Tx when saving, generating a 0xff serial byte
  for ( i = 0; i < ESCCMD_n; i++ )
    ESCCMD_serial[i]->clear( );

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

  // Initialize ESC structure and clear UART input buffer
  for ( i = 0; i < ESCCMD_n; i++ )  {
    ESCCMD_cmd[i] = DSHOT_CMD_MOTOR_STOP;
    ESCCMD_tlm[i] = 0;
    ESCCMD_tlm_pend[i] = 0;
    ESCCMD_tlm_lost_cnt[i] = 0;
    ESCCMD_CRC_errors[i] = 0;
    ESCCMD_last_error[i]  = 0;
    ESCCMD_throttle_wd[i] = ESCCMD_THWD_LEVEL;
    ESCCMD_serial[i]->clear( );
  }

  ESCCMD_tic_pend = 0;

  // Initialize timer
  ESCCMD_timer.begin( ESCCMD_ISR_timer, ESCCMD_TIMER_PERIOD );
  
  // Raise the timer flag
  ESCCMD_timer_flag = 1;

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
  
  // Check if timer started
  if ( !ESCCMD_timer_flag )
    ESCCMD_ERROR( ESCCMD_ERROR_SEQ );

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
  if ( ESCCMD_throttle_wd[i] >= ESCCMD_THWD_LEVEL ) {
    // If watchdog was previously triggered:
    //  Clear UART input buffer
    //  Also clear pending errors, pending packets...
    ESCCMD_serial[i]->clear( );
    ESCCMD_tlm_pend[i] = 0;
    ESCCMD_tlm_lost_cnt[i] = 0;
    ESCCMD_CRC_errors[i] = 0;
    ESCCMD_last_error[i]  = 0;
  }
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
  
  // Check if timer started
  if ( !ESCCMD_timer_flag )
    ESCCMD_ERROR( ESCCMD_ERROR_SEQ );

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
int ESCCMD_read_err( uint8_t i, int8_t *err )  {

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;

  // Check if motor is within range
  if ( i >= ESCCMD_n )
    return ESCCMD_ERROR_PARAM;

  // Return error code
  *err = ESCCMD_last_error[i];
  
  // Clear error
  ESCCMD_last_error[i] = 0;

  return 0;
}

//
//  Return last command code of motor number i
//
int ESCCMD_read_cmd( uint8_t i, uint16_t *cmd )  {

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;

  // Check if motor is within range
  if ( i >= ESCCMD_n )
    return ESCCMD_ERROR_PARAM;


  *cmd = ESCCMD_cmd[i];

  return 0;
}

//
//  Read telemetry status of ESC number i
//
int ESCCMD_read_tlm_status( uint8_t i )  {
  static uint8_t local_state;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;

  // Check if motor is within range
  if ( i >= ESCCMD_n )
    return ESCCMD_ERROR_PARAM;
  
  // Check if timer started
  if ( !ESCCMD_timer_flag )
    ESCCMD_ERROR( ESCCMD_ERROR_SEQ );

  // Define a local copy of the state
  noInterrupts();
  local_state = ESCCMD_state[i];
  interrupts();

  // Check if ESC is armed
  if ( !( local_state & ESCCMD_STATE_ARMED ) )
    ESCCMD_ERROR( ESCCMD_ERROR_SEQ )

  // Check if telemetry is valid and active
  if ( ESCCMD_tlm_valid[i] && ESCCMD_tlm[i] )  {
    return 0;
  }
  else
    return ESCCMD_ERROR_TLM_INVAL;
}

//
//  Read temperature of motor number i
//  Unit is degree Celsius
//
int ESCCMD_read_deg( uint8_t i, uint8_t *deg )  {
  static uint8_t local_state;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;

  // Check if motor is within range
  if ( i >= ESCCMD_n )
    return ESCCMD_ERROR_PARAM;
  
  // Check if timer started
  if ( !ESCCMD_timer_flag )
    ESCCMD_ERROR( ESCCMD_ERROR_SEQ );

  // Define a local copy of the state
  noInterrupts();
  local_state = ESCCMD_state[i];
  interrupts();

  // Check if ESC is armed
  if ( !( local_state & ESCCMD_STATE_ARMED ) )
    ESCCMD_ERROR( ESCCMD_ERROR_SEQ )

  // Check if telemetry is valid
  if ( ESCCMD_tlm_valid[i] )  {
    *deg = ESCCMD_tlm_deg[i];
  }
  else
    return ESCCMD_ERROR_TLM_INVAL;

  return 0;
}

//
//  Read dc power supply voltage of motor number i
//  Unit is Volt
//
int ESCCMD_read_volt( uint8_t i, uint16_t *volt )  {
  static uint8_t local_state;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;

  // Check if motor is within range
  if ( i >= ESCCMD_n )
    return ESCCMD_ERROR_PARAM;
  
  // Check if timer started
  if ( !ESCCMD_timer_flag )
    ESCCMD_ERROR( ESCCMD_ERROR_SEQ );

  // Define a local copy of the state
  noInterrupts();
  local_state = ESCCMD_state[i];
  interrupts();

  // Check if ESC is armed
  if ( !( local_state & ESCCMD_STATE_ARMED ) )
    ESCCMD_ERROR( ESCCMD_ERROR_SEQ )

  // Check if telemetry is valid
  if ( ESCCMD_tlm_valid[i] )  {
    *volt = ESCCMD_tlm_volt[i];
  }
  else
    return ESCCMD_ERROR_TLM_INVAL;

  return 0;
}

//
//  Read current of motor number i
//  Unit is Ampere
//
int ESCCMD_read_amp( uint8_t i, uint16_t *amp )  {
  static uint8_t local_state;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;

  // Check if motor is within range
  if ( i >= ESCCMD_n )
    return ESCCMD_ERROR_PARAM;
  
  // Check if timer started
  if ( !ESCCMD_timer_flag )
    ESCCMD_ERROR( ESCCMD_ERROR_SEQ );

  // Define a local copy of the state
  noInterrupts();
  local_state = ESCCMD_state[i];
  interrupts();

  // Check if ESC is armed
  if ( !( local_state & ESCCMD_STATE_ARMED ) )
    ESCCMD_ERROR( ESCCMD_ERROR_SEQ )

  // Check if telemetry is valid
  if ( ESCCMD_tlm_valid[i] )  {
    *amp = ESCCMD_tlm_amp[i];
  }
  else
    return ESCCMD_ERROR_TLM_INVAL;

  return 0;
}

//
//  Read consumption of motor number i
//  Unit is milli Ampere.hour
//
int ESCCMD_read_mah( uint8_t i, uint16_t *mah )  {
  static uint8_t local_state;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;

  // Check if motor is within range
  if ( i >= ESCCMD_n )
    return ESCCMD_ERROR_PARAM;
  
  // Check if timer started
  if ( !ESCCMD_timer_flag )
    ESCCMD_ERROR( ESCCMD_ERROR_SEQ );

  // Define a local copy of the state
  noInterrupts();
  local_state = ESCCMD_state[i];
  interrupts();

  // Check if ESC is armed
  if ( !( local_state & ESCCMD_STATE_ARMED ) )
    ESCCMD_ERROR( ESCCMD_ERROR_SEQ )

  // Check if telemetry is valid
  if ( ESCCMD_tlm_valid[i] )  {
    *mah = ESCCMD_tlm_mah[i];
  }
  else
    return ESCCMD_ERROR_TLM_INVAL;

  return 0;
}

//
//  Read shaft rotational velocity of motor number i
//  Unit is round per minute
//  The sign of the measurement depends on the last throttle sign
//
int ESCCMD_read_rpm( uint8_t i, int16_t *rpm )  {
  static uint8_t local_state;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;

  // Check if motor is within range
  if ( i >= ESCCMD_n )
    return ESCCMD_ERROR_PARAM;
  
  // Check if timer started
  if ( !ESCCMD_timer_flag )
    ESCCMD_ERROR( ESCCMD_ERROR_SEQ );

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
        *rpm = -ESCCMD_tlm_rpm[i];  // 3D mode negative direction
      else
        *rpm = ESCCMD_tlm_rpm[i];   // 3D mode positive direction
    }
    else {
      // Default mode
      *rpm = ESCCMD_tlm_rpm[i];
    }
    
    // Convert electrical rpm * 100 into motor rpm * 10
    *rpm = ( *rpm * 10 * 2 ) / ESCCMD_TLM_NB_POLES;
  }
  else
    return ESCCMD_ERROR_TLM_INVAL;

  return 0;
}

//
//  Read next serial packet
//
uint8_t *ESCCMD_read_packet( uint8_t i )  {
  #ifndef ESCCMD_ESC_EMULATION
  static int      buffer_idx[ESCCMD_NB_UART] = { 0 };
  static int      serial_ret;
  #endif
  
  #ifdef ESCCMD_ESC_EMULATION
  uint8_t *pt_c;
  
  // Check if a packet is available
  if ( ESCCMD_tlm_emu_nb[i] )  {
  
    ESCCMD_tlm_emu_nb[i]--;
    
    pt_c = &ESCCMD_tlm_emu_deg[ESCCMD_tlm_emu_nb[i]][i];
    ESCCMD_bufferTlm[i][0] = pt_c[0];
    pt_c = (uint8_t*)&ESCCMD_tlm_emu_volt[ESCCMD_tlm_emu_nb[i]][i];
    ESCCMD_bufferTlm[i][1] = pt_c[1];
    ESCCMD_bufferTlm[i][2] = pt_c[0];
    pt_c = (uint8_t*)&ESCCMD_tlm_emu_amp[ESCCMD_tlm_emu_nb[i]][i];
    ESCCMD_bufferTlm[i][3] = pt_c[1];
    ESCCMD_bufferTlm[i][4] = pt_c[0];
    pt_c = (uint8_t*)&ESCCMD_tlm_emu_mah[ESCCMD_tlm_emu_nb[i]][i];
    ESCCMD_bufferTlm[i][5] = pt_c[1];
    ESCCMD_bufferTlm[i][6] = pt_c[0];
    pt_c = (uint8_t*)&ESCCMD_tlm_emu_rpm[ESCCMD_tlm_emu_nb[i]][i];
    ESCCMD_bufferTlm[i][7] = pt_c[1];
    ESCCMD_bufferTlm[i][8] = pt_c[0];
    ESCCMD_bufferTlm[i][9] = ESCCMD_crc8( ESCCMD_bufferTlm[i], ESCCMD_TLM_LENGTH - 1 );
    
    return ESCCMD_bufferTlm[i];
  }
  #else
  // Read all bytes in rx buffer up to packet length
  while ( ( ESCCMD_serial[i]->available( ) ) && 
          ( buffer_idx[i] < ESCCMD_TLM_LENGTH ) ) {
          
    serial_ret = ESCCMD_serial[i]->read( );
      
    if ( serial_ret >= 0 )  {
      ESCCMD_bufferTlm[i][buffer_idx[i]] = (uint8_t)serial_ret;
      buffer_idx[i]++;
    }
  }
  
  // Check if a complete packet has arrived
  if ( buffer_idx[i] == ESCCMD_TLM_LENGTH  )  {
    
    // Reset byte index in packet buffer
    buffer_idx[i] = 0;
    
    // Return pointer to buffer
    return ESCCMD_bufferTlm[i];
  }
  #endif

  return NULL;
}

//
// Extract data from the current packet
//
int ESCCMD_extract_packet_data( uint8_t i )  {

  // Extract packet data
        
  ESCCMD_tlm_deg[i]     =   ESCCMD_bufferTlm[i][0];
  ESCCMD_tlm_volt[i]    = ( ESCCMD_bufferTlm[i][1] << 8 ) | ESCCMD_bufferTlm[i][2];
  ESCCMD_tlm_amp[i]     = ( ESCCMD_bufferTlm[i][3] << 8 ) | ESCCMD_bufferTlm[i][4];
  ESCCMD_tlm_mah[i]     = ( ESCCMD_bufferTlm[i][5] << 8 ) | ESCCMD_bufferTlm[i][6];
  ESCCMD_tlm_rpm[i]     = ( ESCCMD_bufferTlm[i][7] << 8 ) | ESCCMD_bufferTlm[i][8];
  ESCCMD_tlm_valid[i]   = ( ESCCMD_bufferTlm[i][9] == ESCCMD_crc8( ESCCMD_bufferTlm[i], ESCCMD_TLM_LENGTH - 1 ) );

  // If crc is invalid, increment crc error counter
  // and flush UART buffer
  if ( !ESCCMD_tlm_valid[i] ) {

    ESCCMD_CRC_errors[i]++;

    ESCCMD_last_error[i] = ESCCMD_ERROR_TLM_CRC;

    // Check for excessive CRC errors
    if ( ESCCMD_CRC_errors[i] >= ESCCMD_TLM_MAX_CRC_ERR ) {
      ESCCMD_last_error[i] = ESCCMD_ERROR_TLM_CRCMAX;
    }
    
    // Flush UART incoming buffer
    // If ESC is transmitting, need to wait for some byte(s) to come in
    do {
      ESCCMD_serial[i]->clear( );
      delayMicroseconds( ESCCMD_TLM_BYTE_TIME * 2 );
    } while ( ESCCMD_serial[i]->available( ) );
    
    // Reset pending packet counter
    ESCCMD_tlm_pend[i] = 0;

    // Reset lost packet counter
    ESCCMD_tlm_lost_cnt[i] = 0;
    
    return ESCCMD_last_error[i];
  }
  else {
    // Make some verifications on the telemetry

    // Check for overtheating of the ESC
    if ( ESCCMD_tlm_deg[i] >= ESCCMD_TLM_MAX_TEMP ) {
      ESCCMD_last_error[i] = ESCCMD_ERROR_TLM_TEMP;
      return ESCCMD_last_error[i];
    }
  }

  return 0;
}

#ifdef ESCCMD_ESC_EMULATION
//
//  Emulate telemetry
//
void ESCCMD_emulate_tlm( uint8_t i )   {
  static uint8_t  local_state;
  
  // Bufferize emulated telemetry data
  if ( ESCCMD_tlm_emu_nb[i] < ESCCMD_EMU_TLM_MAX ) {
    if ( ESCCMD_tlm[i] )  {
      ESCCMD_tlm_emu_deg[ESCCMD_tlm_emu_nb[i]][i]  = (uint8_t)( ESCCMD_EMU_TLM_DEG *   ( 1.0 + ESCCMD_EMU_TLM_NOISE / 100.0 * (float)( rand( ) - RAND_MAX / 2 ) / ( RAND_MAX / 2 ) ) );
      ESCCMD_tlm_emu_volt[ESCCMD_tlm_emu_nb[i]][i] = (uint16_t)( ESCCMD_EMU_TLM_VOLT * ( 1.0 + ESCCMD_EMU_TLM_NOISE / 100.0 * (float)( rand( ) - RAND_MAX / 2 ) / ( RAND_MAX / 2 ) ) );
      ESCCMD_tlm_emu_amp[ESCCMD_tlm_emu_nb[i]][i]  = (uint16_t)( ESCCMD_EMU_TLM_AMP *  ( 1.0 + ESCCMD_EMU_TLM_NOISE / 100.0 * (float)( rand( ) - RAND_MAX / 2 ) / ( RAND_MAX / 2 ) ) );
      ESCCMD_tlm_emu_mah[ESCCMD_tlm_emu_nb[i]][i]  = (uint16_t)( ESCCMD_EMU_TLM_MAH *  ( 1.0 + ESCCMD_EMU_TLM_NOISE / 100.0 * (float)( rand( ) - RAND_MAX / 2 ) / ( RAND_MAX / 2 ) ) );

      // Define a local copy of the state
      noInterrupts();
      local_state = ESCCMD_state[i];
      interrupts();

      // Compute rpm according to throttle cmd
      if ( local_state & ESCCMD_STATE_3D )  {
      
        // 3D mode
        if ( ESCCMD_cmd[i] > DSHOT_CMD_MAX + 1 + ESCCMD_MAX_3D_THROTTLE ) {
          // Negative direction
          ESCCMD_tlm_emu_rpm[ESCCMD_tlm_emu_nb[i]][i] = (uint16_t)(  (float)( ESCCMD_cmd[i] - ( DSHOT_CMD_MAX + 2 + ESCCMD_MAX_3D_THROTTLE ) ) / ESCCMD_MAX_3D_THROTTLE
                                                                * (float)( ESCCMD_EMU_TLM_VOLT / 100 ) * ESCCMD_EMU_MOTOR_KV
                                                                / 100.0 * ESCCMD_TLM_NB_POLES / 2 ); 
        }
        else  {
          // Positive direction
          ESCCMD_tlm_emu_rpm[ESCCMD_tlm_emu_nb[i]][i] = (uint16_t)(  (float)( ESCCMD_cmd[i] - ( DSHOT_CMD_MAX + 1 ) ) / ESCCMD_MAX_3D_THROTTLE
                                                                * (float)( ESCCMD_EMU_TLM_VOLT / 100 ) * ESCCMD_EMU_MOTOR_KV
                                                                / 100.0  * ESCCMD_TLM_NB_POLES / 2 ); 
        }
      }
      else  {
        // Normal mode
        ESCCMD_tlm_emu_rpm[ESCCMD_tlm_emu_nb[i]][i] = (uint16_t)(  (float)( ESCCMD_cmd[i] - ( DSHOT_CMD_MAX + 1 ) ) / ESCCMD_MAX_THROTTLE
                                                                * (float)( ESCCMD_EMU_TLM_VOLT / 100 ) * ESCCMD_EMU_MOTOR_KV
                                                                / 100.0  * ESCCMD_TLM_NB_POLES / 2 );
      }
    }
    
    // Increment tlm counter according to packet loss statistics
    #ifdef ESCCMD_ESC_EMU_PKT_LOSS
    if ( (int)( ( (float)rand( ) / (float)RAND_MAX * (float)ESCCMD_ESC_FRACTION_PKTLOSS ) ) ) {
      ESCCMD_tlm_emu_nb[i]++;
    }
    #else
    ESCCMD_tlm_emu_nb[i]++;
    #endif
  }
}
#endif

//
//  This routine should be called within the main loop
//  Returns ESCCMD_TIC_OCCURED when a tic occurs, 
//  Return 0 otherwise.
//
int ESCCMD_tic( void )  {
  static int      i;
  static uint16_t local_tic_pend;
 
  // Check if timer started
  if ( !ESCCMD_timer_flag ) {
    return 0;
  }
    
  //// Read telemetry
  
  for ( i = 0; i < ESCCMD_n; i++ )  {
    
    // Process all available telemetry packets
    while ( ESCCMD_read_packet( i ) )  {
        
      // Update pending packet counter
      if ( ESCCMD_tlm_pend[i] ) {
        ESCCMD_tlm_pend[i]--;
      }
      else  {
        // Spurious packet ?
        ESCCMD_last_error[i] = ESCCMD_ERROR_TLM_PEND;
      }
      
      // Extract packet data
      ESCCMD_extract_packet_data( i );
    }
        
    // Check for exceeding packet pending
    if ( ESCCMD_tlm_pend[i] > ESCCMD_TLM_MAX_PEND ) {

      // Packet is considered as lost: update packet lost counter
      if ( ESCCMD_tlm_lost_cnt[i] >= ESCCMD_TLM_MAX_PKT_LOSS )  {
        ESCCMD_last_error[i] = ESCCMD_ERROR_TLM_LOST;
      }
      else  {
        ESCCMD_tlm_lost_cnt[i]++;
      }

      // Decrement packet pending counter
      ESCCMD_tlm_pend[i]--;
    }
  }
  
  //// Process clock tics
  
  noInterrupts();
  local_tic_pend = ESCCMD_tic_pend;
  interrupts();

  if ( local_tic_pend ) {

    // Acknowledgement of one timer clock event
    noInterrupts();
    ESCCMD_tic_pend--;
    interrupts();

    // Update counters
    ESCCMD_tic_counter++;
    
    if ( !( ESCCMD_tic_counter % ESCCMD_TLM_PER ) ) {
      
      // Clear stat counters every ESCCMD_TLM_PER iterations
      for ( i = 0; i < ESCCMD_n; i++ )  {
        ESCCMD_tlm_lost_cnt[i] = 0;
        ESCCMD_CRC_errors[i] = 0; 
      }
    }

    // Check if everything is initialized
    if ( !ESCCMD_init_flag )  {
      for ( i = 0; i < ESCCMD_n; i++ )  {
        ESCCMD_last_error[i] = ESCCMD_ERROR_INIT;
      }
      return ESCCMD_TIC_OCCURED;
    }

    // Check if all ESC are armed
    for ( i = 0; i < ESCCMD_n; i++ )  {
      if ( !( ESCCMD_state[i] & ESCCMD_STATE_ARMED ) )  {
        ESCCMD_last_error[i] = ESCCMD_ERROR_SEQ;
        return ESCCMD_TIC_OCCURED;
      }
    }

    // Throttle watchdog
    for ( i = 0; i < ESCCMD_n; i++ )  {
      if ( ESCCMD_throttle_wd[i] >= ESCCMD_THWD_LEVEL )  {
        // Watchdog triggered on ESC number i
        ESCCMD_cmd[i] = DSHOT_CMD_MOTOR_STOP;
        ESCCMD_tlm[i] = 0;
        ESCCMD_last_error[i] = 0;
        ESCCMD_tlm_lost_cnt[i] = 0;
        ESCCMD_CRC_errors[i] = 0;
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
      for ( i = 0; i < ESCCMD_n; i++ )  {
        ESCCMD_last_error[i] = ESCCMD_ERROR_DSHOT;
      }
    }
    else  {
      delayMicroseconds( ESCCMD_CMD_DELAY );
  
      // Update telemetry packet pending counter
      for ( i = 0; i < ESCCMD_n; i++ )  {
        if ( ESCCMD_tlm[i] )  {
          ESCCMD_tlm_pend[i]++;
          #ifdef ESCCMD_ESC_EMULATION
          ESCCMD_emulate_tlm( i );
          #endif
        }
      }
    }
    
    // Inform caller that a clock tic occured
    return ESCCMD_TIC_OCCURED;
  }

  return 0;
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
    for ( i = 0; i < ESCCMD_n; i++ )  {
      ESCCMD_state[i] &= ~( ESCCMD_STATE_ARMED | ESCCMD_STATE_START );
    }
  }
  else  {
    ESCCMD_tic_pend++;
  }
}
