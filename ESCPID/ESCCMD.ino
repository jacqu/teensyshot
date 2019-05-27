/*
 *  ESCCMD:   ESC DSHOT command packets formating API
 *
 *  Note:     Best viewed using Arduino IDE with tab space = 2
 *
 *  Authors:  Arda Yiğit and Jacques Gangloff
 *  Date:     May 2019
 */

#include "ESCPID.h"
#include "ESCCMD.h"

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
IntervalTimer           ESCCMD_timer;                 // Timer object
volatile uint16_t       ESCCMD_tic_pend = 0;          // Number of timer tic waiting for ackowledgement
volatile ESCCMD_STRUCT  ESCCMD[ESCCMD_MAX_ESC];       // Main data structure
uint8_t                 ESCCMD_init_flag = 0;         // Subsystem initialization flag
uint8_t                 ESCCMD_timer_flag = 0;        // Periodic loop enable/disable flag
uint16_t                ESCCMD_cmd[ESCCMD_MAX_ESC];   // ESC commands
uint8_t                 ESCCMD_tlm[ESCCMD_MAX_ESC];   // ESC telemetry requests

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
  
  // Initialize telemetry UART channels
  Serial1.begin( ESCCMD_TLM_UART_SPEED );
  Serial2.begin( ESCCMD_TLM_UART_SPEED );
  Serial3.begin( ESCCMD_TLM_UART_SPEED );
  Serial4.begin( ESCCMD_TLM_UART_SPEED );
  Serial5.begin( ESCCMD_TLM_UART_SPEED );
  Serial6.begin( ESCCMD_TLM_UART_SPEED );

  // Set the initialization flag
  ESCCMD_init_flag = 1;
}

//
//  Arm all ESCs
//
//  Return values: see defines
//
int ESCCMD_arm_all( void )  {
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
//  Arm specific ESC
//
//  Return values: see defines
//
int ESCCMD_arm_ESC( uint8_t i )  {

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;
    
  // Check if i is valid
  if ( i >= ESCCMD_MAX_ESC )
    return ESCCMD_ERROR_PARAM;

  // Check if the ESC is in the initial state
  if ( ESCCMD[i].state & ESCCMD_STATE_ARMED )
    return ESCCMD_ERROR_SEQ;

  // Define stop command on the ith ESC, keep last cmd on other
  ESCCMD[i].cmd = DSHOT_CMD_MOTOR_STOP;
  ESCCMD_cmd[i] = DSHOT_CMD_MOTOR_STOP;
  ESCCMD[i].tlm = 0;
  ESCCMD_tlm[i] = 0;

  // Send command ESCCMD_CMD_REPETITION times
  for ( i = 0; i < ESCCMD_CMD_REPETITION; i++ )  {

    // Send DSHOT signal to all ESCs
    if ( DSHOT_send( ESCCMD_cmd, ESCCMD_tlm ) )
      return ESCCMD_ERROR_DSHOT;

    // Wait some time
    delayMicroseconds( ESCCMD_CMD_DELAY );
  }

  // Set the arming flag
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
    
  // Minimum delay before next command
  delayMicroseconds( ESCCMD_CMD_SAVE_DELAY );

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
int ESCCMD_start_timer( void )  {
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
  }
  
  noInterrupts();
  ESCCMD_tic_pend = 0;
  interrupts();
  
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
  int i;

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
  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )  {
    ESCCMD[i].cmd = 0;
    ESCCMD[i].tlm = 0;
    ESCCMD[i].state &= ^( ESCCMD_STATE_ARMED | ESCCMD_STATE_START );
  }

  return 0;
}

//
//  Define throttle of ESC number i:
//    Default mode: 0 -> 1999
//    3D mode     : -999 -> 999
//
int ESCCMD_throttle( uint8_t i, int16_t throttle ) {

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;
  
  // Define a local copy of the state
  noInterrupts();
  uint8_t local_state = ESCCMD[i].state;
  interrupts();
  
  // Check if ESC is armed
  if ( !( local_state & ESCCMD_STATE_ARMED ) )
    return ESCCMD_ERROR_SEQ;
  
  // Define throttle depending on the mode
  
  if ( local_state & ESCCMD_STATE_3D )  {
    // Check limits
    if ( ( throttle < ESCCMD_MIN_3D_THROTTLE ) || ( throttle > ESCCMD_MAX_3D_THROTTLE )
      return ESCCMD_ERROR_PARAM;
      
    // 3D mode
    if ( throttle >= 0 )
      ESCCMD[i].cmd = DSHOT_CMD_MAX + 1 + throttle;
    else
      ESCCMD[i].cmd = DSHOT_CMD_MAX + 1 + ESCCMD_MAX_3D_THROTTLE - throttle;
  }
  else {
  
    // Check limits
    if ( ( throttle < 0 ) || ( throttle > ESCCMD_MAX_THROTTLE )
      return ESCCMD_ERROR_PARAM;
      
    // Default mode
    ESCCMD[i].cmd = DSHOT_CMD_MAX + 1 + throttle;
    
  }
  
  // Switch start mode on
  noInterrupts();
  ESCCMD[i].state |= ESCCMD_STATE_START;
  interrupts();
  
  return 0;
}

//
//  Read rotational velocity of motor number i
//  The sign of the measurement depends on the last throttle sign
//
int ESCCMD_read_RPM( uint8_t i, double *rpm )  {

  // Check if everything is initialized
  if ( !ESCCMD_init_flag )
    return ESCCMD_ERROR_INIT;
  
  // Define a local copy of the state
  noInterrupts();
  uint8_t local_state = ESCCMD[i].state;
  interrupts();
  
  // Check if ESC is armed
  if ( !( local_state & ESCCMD_STATE_ARMED ) )
    return ESCCMD_ERROR_SEQ;
  
  // Check if telemetry is valid
  if ( ESCCMD[i].tlm_valid )  {
    // Check current mode
    if ( local_state & ESCCMD_STATE_3D )  {
      // 3D mode
      if ( ESCCMD[i].cmd < 0 )
        *rpm = (double)( -ESCCMD[i].tlm_rpm );
      else
        *rpm = (double)( ESCCMD[i].tlm_rpm );
    }
    else {
      // Default mode
      *rpm = (double)( ESCCMD[i].tlm_rpm );
    }
  }
  else {
    return ESCCMD_ERROR_TLM_INVAL;
  }
  
  return 0;
}


//
//  This routine should be called within the main loop
//
int ESCCMD_tic( void )  {
  int             i, j, ret = 0;
  uint8_t         packet_flag;
  static uint8_t  bufferTlm[ESCCMD_TLM_LENGTH];

  // Read telemetry if packets are pending
  for ( i = 0; i < ESCCMD_MAX_ESC; i++ )  {
    if ( ESCCMD[i].tlm_pend ) {
      
      // Packet flag indicates if a complete packet has arrived
      packet_flag = 0;
      
      switch( i ) {
        case 1:
          // Check if a complete packet has arrived
          if ( Serial1.available( ) == ESCCMD_TLM_LENGTH )  {
          
            // Read packet
            for ( j = 0; j < ESCCMD_TLM_LENGTH; j++ )
              bufferTlm[i] = Serial1.read( );
            
            // Update packet flag and pending packet counter
            packet_flag = 1;
            }
          break;
          
        case 2:
          // Check if a complete packet has arrived
          if ( Serial2.available( ) == ESCCMD_TLM_LENGTH )  {
          
            // Read packet
            for ( j = 0; j < ESCCMD_TLM_LENGTH; j++ )
              bufferTlm[i] = Serial2.read( );
            
            // Update packet flag and pending packet counter
            packet_flag = 1;
          }
          break;
        
        case 3:
          // Check if a complete packet has arrived
          if ( Serial3.available( ) == ESCCMD_TLM_LENGTH )  {
          
            // Read packet
            for ( j = 0; j < ESCCMD_TLM_LENGTH; j++ )
              bufferTlm[i] = Serial3.read( );
              
            // Update packet flag and pending packet counter
            packet_flag = 1;
          }
          break;
        
        case 4:
          // Check if a complete packet has arrived
          if ( Serial4.available( ) == ESCCMD_TLM_LENGTH )  {
          
            // Read packet
            for ( j = 0; j < ESCCMD_TLM_LENGTH; j++ )
              bufferTlm[i] = Serial4.read( );
            
            // Update packet flag and pending packet counter
            packet_flag = 1;
          }
          break;
          
        case 5:
          // Check if a complete packet has arrived
          if ( Serial5.available( ) == ESCCMD_TLM_LENGTH )  {
          
            // Read packet
            for ( j = 0; j < ESCCMD_TLM_LENGTH; j++ )
              bufferTlm[i] = Serial5.read( );
            
            // Update packet flag and pending packet counter
            packet_flag = 1;
          }
          break;
        
        case 6:
          // Check if a complete packet has arrived
          if ( Serial6.available( ) == ESCCMD_TLM_LENGTH )  {
          
            // Read packet
            for ( j = 0; j < ESCCMD_TLM_LENGTH; j++ )
              bufferTlm[i] = Serial6.read( );
            
            // Update packet flag and pending packet counter
            packet_flag = 1;
          }
          break;
        
        default:
      }
      
      // If a packet has arrived, process it
      
      if ( packet_flag )  {
        ESCCMD[i].tlm_deg     =   bufferTlm[0];
        ESCCMD[i].tlm_volt    = ( bufferTlm[1] << 8 ) | bufferTlm[2];
        ESCCMD[i].tlm_amp     = ( bufferTlm[3] << 8 ) | bufferTlm[4];
        ESCCMD[i].tlm_mah     = ( bufferTlm[5] << 8 ) | bufferTlm[6];
        ESCCMD[i].tlm_rpm     = ( bufferTlm[7] << 8 ) | bufferTlm[8];
        ESCCMD[i].tlm_valid   = ( bufferTlm[9] == ESCCMD_crc8( bufferTlm, ESCCMD_TLM_LENGTH - 1 ) );
        
        // Update pending packet counter
        ESCCMD[i].tlm_pend--;
        
        // If crc is invalid, increment crc error counter
        // and flush UART buffer
        if ( !ESCCMD[i].tlm_valid ) {
        
          ESCCMD[i].CRC_errors++;
          ret = ESCCMD_ERROR_CRC;
          
          // Wait for last out of sync bytes to come in
          for (j = 0; j < ESCCMD[i].tlm_pend + 1; j++ )
            delayMicroseconds( ESCCMD_TIMER_PERIOD );
          
          // Flush UART incoming buffer
          
          switch( i ) {
            case 1:
              while ( Serial1.available( ) )  Serial1.read( );
              break;
            
            case 2:
              while ( Serial2.available( ) )  Serial2.read( );
              break;
            
            case 3:
              while ( Serial3.available( ) )  Serial3.read( );
              break;
            
            case 4:
              while ( Serial4.available( ) )  Serial4.read( );
              break;
            
            case 5:
              while ( Serial5.available( ) )  Serial5.read( );
              break;
            
            case 6:
              while ( Serial6.available( ) )  Serial6.read( );
              break;
              
            default:          
          }
        
        // Reset pending packet counter
        ESCCMD[i].tlm_pend = 0;
        }
      }
    }
  }
  
  // Do something only if tics are pending
  noInterrupts();
  uint16_t local_tic_pend = ESCCMD_tic_pend;
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
    for ( i = 0; i < ESCCMD_MAX_ESC; i++ )
      if ( !( ESCCMD[i].state & ESCCMD_STATE_ARMED ) )
        return ESCCMD_ERROR_SEQ;
    
    // Send current command
    for ( i = 0; i < ESCCMD_MAX_ESC; i++ )  {
      ESCCMD_cmd[i] = ESCCMD[i].cmd;
      ESCCMD_tlm[i] = ESCCMD[i].tlm;
    }
    if ( DSHOT_send( ESCCMD_cmd, ESCCMD_tlm ) )
      return ESCCMD_ERROR_DSHOT;
    delayMicroseconds( ESCCMD_CMD_DELAY );
    
    // Update telemetry packet pending counter
    for ( i = 0; i < ESCCMD_MAX_ESC; i++ )
      if ( ESCCMD[i].tlm )
        ESCCMD[i].tlm_pend++;
  }

  return ret;
}

//
// crc8 calculation
//
uint8_t ESCCMD_update_crc8( uint8_t crc, uint8_t crc_seed ) {
  uint8_t crc_u = crc;
  crc_u ^= crc_seed;

  for ( int i = 0; i < 8; i++ ) {
    crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  }

  return crc_u;
}

uint8_t ESCCMD_crc8( uint8_t* buf, uint8_t buflen ) {
  uint8_t crc = 0;
  for ( int i = 0; i < buflen; i++ ) {
    crc = ESCCMD_update_crc8( buf[i], crc );
  }

  return crc;
}

//
//  Timer ISR
//
void ESCCMD_ISR_timer( void ) {
  int i;
  
  // Increment tic pending counter
  for ( i = 0; i < ESCCMD_MAX_ ESC; i++ )  {
  
    // Check for maximum missed tics (ESC watchdog timer = 250ms on a KISS ESC)
    if ( ESCCMD[i].tic_pend >= ESCCMD_TIMER_MAX_MISS )  {
    
      // ESC watchdog switch to disarmed mode
      ESCCMD[i].state &= ~( ESCCMD_STATE_ARMED | ESCCMD_STATE_START );
    }
    else {
      ESCCMD[i].tic_pend++;
    }
  }
}