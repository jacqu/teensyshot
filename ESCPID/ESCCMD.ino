/*
 *  ESCCMD:   ESC DSHOT command packets formating API
 *
 *  Note:     Best viewed using Arduino IDE with tab space = 2
 *
 *  Authors:  Arda Yiğit and Jacques Gangloff
 *  Date:     May 2019
 */

#include "ESCPID.h"

// Defines
#define ESCCMD_MAX_ESC          ESCPID_NB_ESC     // Max number of ESCs

#define ESCCMD_STATE_ARMED      1                 // Mask for the arming flag
#define ESCCMD_STATE_3D         2                 // Mask for the default/3D mode
#define ESCCMD_STATE_START      4                 // Mask for the motor start/stop bit
#define ESCCMD_STATE_ERROR      128               // Mask for the error flag

#define ESCCMD_CMD_REPETITION   10                // Number of time commands have to be repeated to be acknowledged by ESC
#define ESCCMD_CMD_DELAY        10                // Delay between two consecutive DSHOT transmissions (us)

#define ESCCMD_TIMER_PERIOD     2000              // Periodic loop period (us)

#define ESCCMD_ERROR_DSHOT      -1                // DSHOT error
#define ESCCMD_ERROR_SEQ        -2                // Invalid function call sequence error
#define ESCCMD_ERROR_INIT       -3                // Call of non initialized function

// enums: borrowed from betaflight pwm_output.h
typedef enum {
  DSHOT_CMD_MOTOR_STOP = 0,
  DSHOT_CMD_BEACON1,
  DSHOT_CMD_BEACON2,
  DSHOT_CMD_BEACON3,
  DSHOT_CMD_BEACON4,
  DSHOT_CMD_BEACON5,
  DSHOT_CMD_ESC_INFO,                       // V2 includes settings
  DSHOT_CMD_SPIN_DIRECTION_1,
  DSHOT_CMD_SPIN_DIRECTION_2,
  DSHOT_CMD_3D_MODE_OFF,                    // 9
  DSHOT_CMD_3D_MODE_ON,                     // 10
  DSHOT_CMD_SETTINGS_REQUEST,               // Currently not implemented
  DSHOT_CMD_SAVE_SETTINGS,                  // 12
  DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
  DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
  DSHOT_CMD_LED0_ON,                        // BLHeli32 only
  DSHOT_CMD_LED1_ON,                        // BLHeli32 only
  DSHOT_CMD_LED2_ON,                        // BLHeli32 only
  DSHOT_CMD_LED3_ON,                        // BLHeli32 only
  DSHOT_CMD_LED0_OFF,                       // BLHeli32 only
  DSHOT_CMD_LED1_OFF,                       // BLHeli32 only
  DSHOT_CMD_LED2_OFF,                       // BLHeli32 only
  DSHOT_CMD_LED3_OFF,                       // BLHeli32 only
  DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30,  // KISS audio Stream mode on/Off
  DSHOT_CMD_SILENT_MODE_ON_OFF = 31,        // KISS silent Mode on/Off
  DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE = 32,
  DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY = 33,
  DSHOT_CMD_MAX = 47
} ESCCMD_codes;

// Main structure definition
typedef struct  {
  uint16_t      state;                // Current state of the cmd subsystem
  uint16_t      CRC_errors;           // Overall number of CRC error since start
  int16_t       last_error;           // Last error code
  uint16_t      cmd;                  // Last command
  uint8_t       tlm_deg;              // ESC temperature (°C)
  uint16_t      tlm_volt;             // Voltage of the ESC power supply (0.01V)
  uint16_t      tlm_amp;              // ESC current (0.01A)
  uint16_t      tlm_mah;              // ESC consumption (mAh)
  uint16_t      tlm_rpm;              // ESC electrical rpm (100rpm)
  uint8_t       tlm;                  // Set to 1 when asking for telemetry
  uint8_t       tlm_pend;             // Flag indicating a pending telemetry data request
  uint8_t       tlm_valid;            // Flag indicating the validity of telemetry data
  } ESCCMD_STRUCT;

//
//  Global variables
//
IntervalTimer   ESCCMD_timer;                 // Timer object
ESCCMD_STRUCT   ESCCMD[ESCCMD_MAX_ESC];       // Main data structure
uint8_t         ESCCMD_init_flag = 0;         // Subsystem initialization flag
uint8_t         ESCCMD_timer_flag = 0;        // Periodic loop enable/disable flag
uint16_t        ESCCMD_cmd[ESCCMD_MAX_ESC];   // ESC commands
uint8_t         ESCCMD_tlm[ESCCMD_MAX_ESC];   // ESC telemetry requests

//
//  Initialization
//
void ESCCMD_init( void )  {
  int i;

  if ( ESCCMD_init_flag )
    return;

  // Initialize data structures to zero
  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )
    ESCCMD[i] = {};

  // Initialize DSHOT generation subsystem
  DSHOT_init( );

  // Set the initialization flag
  ESCCMD_init_flag = 1;
}

//
//  Arm all ESCs
//
//  Return values: see defines
//
int ESCCMD_arm( void )  {
  int i;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;

  // Check if all the ESCs are in the initial state
  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )
    if ( ESCCMD[i].state & ESCCMD_STATE_ARMED )
      return ESCCMD_ERROR_SEQ;

  // Define stop command
  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )  {
    ESCCMD[i].cmd = DSHOT_CMD_MOTOR_STOP;
    ESCCMD_cmd[i] = DSHOT_CMD_MOTOR_STOP;
    ESCCMD[i].tlm = 0;
    ESCCMD_tlm[i] = 0;
  }

  // Send command ESCCMD_CMD_REPETITION times
  for ( i = 0; i < ESCCMD_CMD_REPETITION; i++ )  {

    // Send DSHOT signal to all ESCs
    if ( DSHOT_send( ESCCMD_cmd, ESCCMD_tlm ) )
      return ESCCMD_ERROR_DSHOT;

    // Wait some time
    delayMicroseconds( ESCCMD_CMD_DELAY );
  }

  // Set the arming flag
  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )
    ESCCMD[i].state |= ESCCMD_STATE_ARMED;

  return 0;
}

//
//  Activate 3D mode
//
//  Return values: see defines
//
int ESCCMD_3D_on( void )  {
  int i;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;

  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )  {
    // Check if all the ESCs are armed
    if ( !( ESCCMD[i].state & ESCCMD_STATE_ARMED ) )
      return ESCCMD_ERROR_SEQ;

    // Check if ESCs are already in 3D mode
    if ( ESCCMD[i].state & ESCCMD_STATE_3D )
      return ESCCMD_ERROR_SEQ;

    // Check if ESCs are stopped
    if ( ESCCMD[i].state & ESCCMD_STATE_START )
      return ESCCMD_ERROR_SEQ;
  }

  // Define 3D on command
  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )  {
    ESCCMD[i].cmd = DSHOT_CMD_3D_MODE_ON;
    ESCCMD_cmd[i] = DSHOT_CMD_3D_MODE_ON;
    ESCCMD[i].tlm = 0;
    ESCCMD_tlm[i] = 0;
  }

  // Send command ESCCMD_CMD_REPETITION times
  for ( i = 0; i < ESCCMD_CMD_REPETITION; i++ )  {

    // Send DSHOT signal to all ESCs
    if ( DSHOT_send( ESCCMD_cmd, ESCCMD_tlm ) )
      return ESCCMD_ERROR_DSHOT;

    // Wait some time
    delayMicroseconds( ESCCMD_CMD_DELAY );
  }

  // Define save settings command
  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )  {
    ESCCMD[i].cmd = DSHOT_CMD_SAVE_SETTINGS;
    ESCCMD_cmd[i] = DSHOT_CMD_SAVE_SETTINGS;
    ESCCMD[i].tlm = 0;
    ESCCMD_tlm[i] = 0;
  }

  // Send command ESCCMD_CMD_REPETITION times
  for ( i = 0; i < ESCCMD_CMD_REPETITION; i++ )  {

    // Send DSHOT signal to all ESCs
    if ( DSHOT_send( ESCCMD_cmd, ESCCMD_tlm ) )
      return ESCCMD_ERROR_DSHOT;

    // Wait some time
    delayMicroseconds( ESCCMD_CMD_DELAY );
  }

  // Set the 3D mode flag
  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )
    ESCCMD[i].state |= ESCCMD_STATE_3D;

  return 0;
}

//
//  Deactivate 3D mode
//
//  Return values: see defines
//
int ESCCMD_3D_off( void )  {
  int i;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;

  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )  {
    // Check if all the ESCs are armed
    if ( !( ESCCMD[i].state & ESCCMD_STATE_ARMED ) )
      return ESCCMD_ERROR_SEQ;

    // Check if ESCs are already in default mode
    if ( !( ESCCMD[i].state & ESCCMD_STATE_3D ) )
      return ESCCMD_ERROR_SEQ;

    // Check if ESCs are stopped
    if ( ESCCMD[i].state & ESCCMD_STATE_START )
      return ESCCMD_ERROR_SEQ;
  }

  // Define 3D off command
  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )  {
    ESCCMD[i].cmd = DSHOT_CMD_3D_MODE_OFF;
    ESCCMD_cmd[i] = DSHOT_CMD_3D_MODE_OFF;
    ESCCMD[i].tlm = 0;
    ESCCMD_tlm[i] = 0;
  }

  // Send command ESCCMD_CMD_REPETITION times
  for ( i = 0; i < ESCCMD_CMD_REPETITION; i++ )  {

    // Send DSHOT signal to all ESCs
    if ( DSHOT_send( ESCCMD_cmd, ESCCMD_tlm ) )
      return ESCCMD_ERROR_DSHOT;

    // Wait some time
    delayMicroseconds( ESCCMD_CMD_DELAY );
  }

  // Define save settings command
  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )  {
    ESCCMD[i].cmd = DSHOT_CMD_SAVE_SETTINGS;
    ESCCMD_cmd[i] = DSHOT_CMD_SAVE_SETTINGS;
    ESCCMD[i].tlm = 0;
    ESCCMD_tlm[i] = 0;
  }

  // Send command ESCCMD_CMD_REPETITION times
  for ( i = 0; i < ESCCMD_CMD_REPETITION; i++ )  {

    // Send DSHOT signal to all ESCs
    if ( DSHOT_send( ESCCMD_cmd, ESCCMD_tlm ) )
      return ESCCMD_ERROR_DSHOT;

    // Wait some time
    delayMicroseconds( ESCCMD_CMD_DELAY );
  }

  // Clear the 3D mode flag
  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )
    ESCCMD[i].state &= ~(ESCCMD_STATE_3D);

  return 0;
}

//
//  Start periodic loop. ESC should be armed.
//
//  Return values: see defines
//
int ESCCMD_start( void )  {
  int i;

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;

  // Check if timer already started
  if ( ESCCMD_timer_flag )
    return ESCCMD_ERROR_SEQ;

  // Checks
  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )  {
    // Check if all the ESCs are armed
    if ( !( ESCCMD[i].state & ESCCMD_STATE_ARMED ) )
      return ESCCMD_ERROR_SEQ;

    // Check if ESCs are stopped
    if ( ESCCMD[i].state & ESCCMD_STATE_START )
      return ESCCMD_ERROR_SEQ;
  }

  // Initialize ESC structure
  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )  {
    ESCCMD[i].cmd = 0;
    ESCCMD[i].tlm = 1;
    ESCCMD[i].tlm_pend = 0;
    ESCCMD[i].state |= ESCCMD_STATE_START;
  }

  // Initialize timer
  ESCCMD_timer.begin( ESCCMD_ISR_timer, ESCCMD_TIMER_PERIOD );

  return 0;
}

//
//  Timer ISR
//
void ESCCMD_ISR_timer( void ) {
  int i;

  // Check if telemetry data is pending


  // Define command buffer
  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )
    ESCCMD_cmd[i] = ESCCMD[i].cmd;

  // Send DSHOT command
  /* FOR DEBUG:
   * last_error contains the error code sent by DSHOT_send
   */
  int temp = DSHOT_send( ESCCMD_cmd, ESCCMD_tlm );
  if ( temp ) {
    for ( i = 0; i < ESCCMD_MAX_ESC; i++ )  {
      ESCCMD[i].last_error = temp;
      ESCCMD[i].state |= ESCCMD_STATE_ERROR;
    }
  }

  // If telemetry is asked, increment the pending counter
  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )
    if ( ESCCMD[i].tlm )
      ESCCMD[i].tlm_pend += 1;
}

/* FOR DEBUG*/
int getError( void ) {
  int i;
  for (i = 0; i < ESCCMD_MAX_ESC; i++) {
    //noInterrupts();
    int temp = ESCCMD[i].last_error;
    //interrupts();
    if (temp) {
      return temp;
    }
  }
  return 0;
}
