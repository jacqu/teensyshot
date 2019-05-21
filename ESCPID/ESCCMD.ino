/*
 *  ESCCMD:    ESC DSHOT command packets formating API
 *  
 *  Note:     Best viewed using Arduino IDE with tab space = 2
 *
 *  Authors:  Arda Yiğit and Jacques Gangloff
 *  Date:     May 2019
 */

#include "ESCPID.h"

// Defines
#define ESCCMD_MAX_ESC          ESCPID_NB_ESC       // Max number of ESCs

#define ESCCMD_STATE_ARMED      1                   // Mask for the arming flag
#define ESCCMD_STATE_3D         2                   // Mask for the normal/3D mode
#define ESCCMD_STATE_START      4                   // Mask for the start/stop bit
#define ESCCMD_STATE_ERROR      128                 // Mask for the error flag

#define ESCCMD_CMD_REPETITION   10                  // Number of time commands have to be repeated to be acknowledged by ESC
#define ESCCMD_CMD_DELAY        10                  // Delay between two consecutive DSHOT transmissions (us)

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
  uint16_t      last_error;           // Last error code
  uint16_t      cmd;                  // Last command
  uint8_t       tlm_deg;              // ESC temperature (°C)
  uint16_t       tlm_volt;            // Voltage of the ESC power supply (0.01V)
  uint16_t       tlm_amp;             // ESC current (0.01A)
  uint16_t       tlm_mah;             // ESC consumption (mAh)
  uint16_t       tlm_rpm;             // ESC electrical rpm (100rpm)
  uint8_t        tlm;                 // Set to 1 when asking for telemetry
  uint8_t        tlm_valid;           // Flag indicating the validity of telemetry data
  } ESCCMD_STRUCT;
 
//
//  Global variables
//
ESCCMD_STRUCT  ESCCMD[ESCCMD_MAX_ESC];
uint8_t        ESCCMD_init_flag = 0;
uint16_t       ESCCMD_cmd[ESCCMD_MAX_ESC];
uint8_t       ESCCMD_tlm[ESCCMD_MAX_ESC];

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
//  Return values:
//    -1: ESCCMD subsystem not initialized
//    -2: ESC already armed
//    -3: DSHOT error
//
int ESCCMD_arm( void )  {
  int i;
  
  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return -1;
    
  // Check if all the ESCs are in the initial state
  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )
    if ( ESCCMD[i].state & ESCCMD_STATE_ARMED )
      return -2;
      
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
      return -3;
    
    // Wait some time
    delayMicroseconds( ESCCMD_CMD_DELAY )
  }
  
  // Set the arming flag
  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )
    ESCCMD[i].state |= ESCCMD_STATE_ARMED;
      
  return 0;
}

//
//  Activate 3D mode
//
//  Return values:
//    -1: ESCCMD subsystem not initialized
//    -2: ESC not armed
//    -3: ESC already in 3D mode
//    -4: motor not stopped
//    -5: DSHOT error
//
int ESCCMD_3D_on( void )  {
  int i;
  
  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return -1;
  
  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )  {
    // Check if all the ESCs are armed
    if ( !( ESCCMD[i].state & ESCCMD_STATE_ARMED ) )
      return -2;
  
    // Check if ESCs are already in 3D mode
    if ( ESCCMD[i].state & ESCCMD_STATE_3D )
      return -3;
  
    // Check if ESCs are stopped
    if ( ESCCMD[i].state & ESCCMD_STATE_START )
      return -4;
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
      return -5;
    
    // Wait some time
    delayMicroseconds( ESCCMD_CMD_DELAY )
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
      return -5;
    
    // Wait some time
    delayMicroseconds( ESCCMD_CMD_DELAY )
  }
  
  // Set the 3D mode flag
  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )
    ESCCMD[i].state |= ESCCMD_STATE_3D;
      
  return 0;
}

//
//  Deactivate 3D mode
//
//  Return values:
//    -1: ESCCMD subsystem not initialized
//    -2: ESC not armed
//    -3: ESC already in 3D mode
//    -4: motor not stopped
//    -5: DSHOT error
//
int ESCCMD_3D_off( void )  {
  int i;
  
  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return -1;
  
  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )  {
    // Check if all the ESCs are armed
    if ( !( ESCCMD[i].state & ESCCMD_STATE_ARMED ) )
      return -2;
  
    // Check if ESCs are already in default mode
    if ( !( ESCCMD[i].state & ESCCMD_STATE_3D ) )
      return -3;
  
    // Check if ESCs are stopped
    if ( ESCCMD[i].state & ESCCMD_STATE_START )
      return -4;
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
      return -5;
    
    // Wait some time
    delayMicroseconds( ESCCMD_CMD_DELAY )
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
      return -5;
    
    // Wait some time
    delayMicroseconds( ESCCMD_CMD_DELAY )
  }
  
  // Clear the 3D mode flag
  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )
    ESCCMD[i].state &= ~(ESCCMD_STATE_3D);
      
  return 0;
}