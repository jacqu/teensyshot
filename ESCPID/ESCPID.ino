/*
 *  ESCPID:   PID control of up to 6 ESCs using teensy 3.5 MCU
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
#include "AWPID.h"
#include "ESCPID.h"

// Flags
//#define ESCPID_DEBUG_MSG                        // Send debug messages to serial

// Globals
double    ESCPID_Reference[ESCPID_NB_ESC] = {};
double    ESCPID_Measurement[ESCPID_NB_ESC] = {};
double    ESCPID_Control[ESCPID_NB_ESC] = {};
char      ESCPID_error_msg[ESCPID_ERROR_MSG_LENGTH];
uint16_t  ESCPID_comm_wd = 0;

double    ESCPID_Kp[ESCPID_NB_ESC];
double    ESCPID_Ki[ESCPID_NB_ESC];
double    ESCPID_Kd[ESCPID_NB_ESC];
double    ESCPID_f[ESCPID_NB_ESC];
double    ESCPID_Min[ESCPID_NB_ESC];
double    ESCPID_Max[ESCPID_NB_ESC];

ESCPIDcomm_struct_t ESCPID_comm = {
                                  ESCPID_COMM_MAGIC,
                                  {},
                                  {},
                                  {},
                                  {},
                                  {},
                                  {}
                                  };
Hostcomm_struct_t   Host_comm =   {
                                  ESCPID_COMM_MAGIC,
                                  {},
                                  {},
                                  {},
                                  {},
                                  {}
                                  };

//
// Manage communication with the host
//
int ESCPID_comm_update( void ) {
  static int          i;
  static uint8_t      *pt;
  static int          ret;
  
  ret = 0;
  
  // Update output data structure values
  for ( i = 0; i < ESCPID_NB_ESC; i++ ) {
    ESCCMD_read_err( i, &ESCPID_comm.err[i] );    // Last error number
    ESCCMD_read_cmd( i, &ESCPID_comm.cmd[i] );    // Current ESC command value
    ESCCMD_read_deg( i, &ESCPID_comm.deg[i] );    // ESC temperature (°C)
    ESCCMD_read_volt( i, &ESCPID_comm.volt[i] );  // Voltage of the ESC power supply (V)
    ESCCMD_read_amp( i, &ESCPID_comm.amp[i] );    // ESC current (A)
    ESCCMD_read_mah( i, &ESCPID_comm.mah[i] );    // ESC consumption (mAh)
    ESCCMD_read_rpm( i, &ESCPID_comm.rpm[i] );    // Motor velocity (rpm)
  }
  
  // Pointer points to the outcoming data structure
  pt = (uint8_t*)(&ESCPID_comm);
  
  // Send data structure to host
  Serial.write( pt, sizeof( ESCPID_comm ) );
  
  // Pointer points to the incoming data structure
  pt = (uint8_t*)(&Host_comm);
  
  // Check if an incoming packet is available
  if ( Serial.available( ) >= (int)sizeof( Host_comm ) ) {
  
    // Copy incoming packet bytes into data structure
    for ( i = 0; i < (int)sizeof( Host_comm ); i++ )
      pt[i] = Serial.read( );
  
    // Check the validity of the magic number
    if ( Host_comm.magic != ESCPID_COMM_MAGIC ) {
    
      // Flush input buffer
      while ( Serial.read( ) != -1 );
    
      ret = ESCPID_ERROR_MAGIC;
    }
    else {
      // Reset the communication watchdog
      ESCPID_comm_wd = 0;
      
      // Update the reference
      for ( i = 0; i < ESCPID_NB_ESC; i++ )
        ESCPID_Reference[i] = Host_comm.RPM_r[i];
      
      // Update PID tuning parameters
      for ( i = 0; i < ESCPID_NB_ESC; i++ ) {
        ESCPID_Kp[i] = Host_comm.PID_P[i];
        ESCPID_Ki[i] = Host_comm.PID_I[i];
        ESCPID_Kd[i] = Host_comm.PID_D[i];
        ESCPID_f[i] = Host_comm.PID_f[i];
        AWPID_tune(     i,
                        ESCPID_Kp[i],
                        ESCPID_Ki[i],
                        ESCPID_Kd[i],
                        ESCPID_f[i],
                        ESCPID_Min[i],
                        ESCPID_Max[i]
                       );
      }
    }
  }

  return ret;
}

//
//  Error processing
//
char *ESCPID_error( const char *prefix, int ret ) {

  switch( ret ) {
    case ESCCMD_ERROR_DSHOT:
      snprintf( ESCPID_error_msg,
                ESCPID_ERROR_MSG_LENGTH, "%s:%s",
                prefix,
                "DSHOT error" );
      break;
    case ESCCMD_ERROR_SEQ:
      snprintf( ESCPID_error_msg,
                ESCPID_ERROR_MSG_LENGTH, "%s:%s",
                prefix,
                "invalid function call sequence error" );
      break;
    case ESCCMD_ERROR_INIT:
      snprintf( ESCPID_error_msg,
                ESCPID_ERROR_MSG_LENGTH, "%s:%s",
                prefix,
                "call of non initialized function" );
      break;
    case ESCCMD_ERROR_PARAM:
      snprintf( ESCPID_error_msg,
                ESCPID_ERROR_MSG_LENGTH, "%s:%s",
                prefix,
                "invalid parameter error" );
      break;
    case ESCCMD_ERROR_TLM_CRC:
      snprintf( ESCPID_error_msg,
                ESCPID_ERROR_MSG_LENGTH, "%s:%s",
                prefix,
                "CRC error" );
      break;
    case ESCCMD_ERROR_TLM_INVAL:
      snprintf( ESCPID_error_msg,
                ESCPID_ERROR_MSG_LENGTH, "%s:%s",
                prefix,
                "invalid telemetry error" );
      break;
    case ESCCMD_ERROR_TLM_TEMP:
      snprintf( ESCPID_error_msg,
                ESCPID_ERROR_MSG_LENGTH, "%s:%s",
                prefix,
                "ESC overheating error" );
      break;
    case ESCCMD_ERROR_TLM_CRCMAX:
      snprintf( ESCPID_error_msg,
                ESCPID_ERROR_MSG_LENGTH, "%s:%s",
                prefix,
                "max allowed CRC error" );
      break;

    default:
      snprintf( ESCPID_error_msg,
                ESCPID_ERROR_MSG_LENGTH, "%s:%s",
                prefix,
                "unknown error" );
  }

  return ESCPID_error_msg;
}

//
//  Arduino setup function
//
void setup() {
  int i;
  #ifdef ESCPID_DEBUG_MSG
  int ret;
  #endif

  // Initialize USB serial link
  Serial.begin( ESCPID_USB_UART_SPEED );

  // Wait for serial link to initialize
  while ( !Serial );

  // Initialize PID gains
  for ( i = 0; i < ESCPID_NB_ESC; i++ ) {
    ESCPID_Kp[i] =  ESCPID_PID_P;
    ESCPID_Ki[i] =  ESCPID_PID_I;
    ESCPID_Kd[i] =  ESCPID_PID_D;
    ESCPID_f[i] =   ESCPID_PID_F;
    ESCPID_Min[i] = ESCPID_PID_MIN;
    ESCPID_Max[i] = ESCPID_PID_MAX;
  }

  // Initialize PID subsystem
  AWPID_init( ESCPID_NB_ESC, 
              ESCPID_Kp, 
              ESCPID_Ki, 
              ESCPID_Kd, 
              ESCPID_f, 
              ESCPID_Min, 
              ESCPID_Max );

  // Initialize the CMD subsystem
  ESCCMD_init( ESCPID_NB_ESC );

  // Arming ESCs
  #ifndef ESCPID_DEBUG_MSG
  ESCCMD_arm_all( );
  #else
  ret = ESCCMD_arm_all( );
  // Process error
  if ( ret )
    Serial.println( ESCPID_error( "ESCCMD_arm_all", ret ) );
  #endif
  
  // Switch 3D mode on
  #ifndef ESCPID_DEBUG_MSG
  ESCCMD_3D_on( );
  #else
  ret = ESCCMD_3D_on( );
  // Process error
  if ( ret )
    Serial.println( ESCPID_error( "ESCCMD_3D_on", ret ) );
  #endif

  // Arming ESCs
  #ifndef ESCPID_DEBUG_MSG
  ESCCMD_arm_all( );
  #else
  ret = ESCCMD_arm_all( );
  // Process error
  if ( ret )
    Serial.println( ESCPID_error( "ESCCMD_arm_all", ret ) );
  #endif

  // Start periodic loop
  #ifndef ESCPID_DEBUG_MSG
  ESCCMD_start_timer( );
  #else
  ret = ESCCMD_start_timer( );
  // Process error
  if ( ret )
    Serial.println( ESCPID_error( "ESCCMD_start_timer", ret ) );
  #endif
  
  // Stop all motors
  for ( i = 0; i < ESCPID_NB_ESC; i++ ) {
    #ifndef ESCPID_DEBUG_MSG
    ESCCMD_throttle( i, DSHOT_CMD_MOTOR_STOP );
    #else
    ret = ESCCMD_throttle( i, DSHOT_CMD_MOTOR_STOP );
    
    // Process error
    if ( ret )
      Serial.println( ESCPID_error( "ESCCMD_throttle", ret ) );
    #endif
  }

  // Init finished
  #ifdef ESCPID_DEBUG_MSG
  Serial.println( "ESCPID setup complete" );
  #endif
}

//
//  Arduino main loop
//
void loop( ) {
  static int i, ret;

  // Check for next timer event
  ret = ESCCMD_tic( );
  if ( ret == ESCCMD_TIC_OCCURED )  {

    // Process timer event
    
    // Bidirectional serial exchange with host
    ESCPID_comm_update(  );

    // Read all measurements and compute current control signal
    for ( i = 0; i < ESCPID_NB_ESC; i++ ) {
    
      // Compute control signal only if telemetry is valid
      // In case of invalid telemetry, last control signal is sent
      if ( !( ESCCMD_read_rpm( i, &ESCPID_Measurement[i] ) ) )
        AWPID_control(  i, 
                        ESCPID_Reference[i], 
                        ESCPID_Measurement[i], 
                        &ESCPID_Control[i] );
    
      // Define the sign of the throttle according to the reference sign
      if ( ESCPID_Reference[i] >= 0 )
        ESCPID_Control[i] = fabs( ESCPID_Control[i] );
      else
        ESCPID_Control[i] = -fabs( ESCPID_Control[i] );
      
      // Send control signal if reference has been sufficiently refreshed
      if ( ESCPID_comm_wd <= ESCPID_COMM_WD_LEVEL ) {
        ret = ESCCMD_throttle( i, (int16_t)ESCPID_Control[i] );
        ESCPID_comm_wd++;
        }
        
      // Process error
      #ifdef ESCPID_DEBUG_MSG
      if ( ret )
        Serial.println( ESCPID_error( "ESCCMD_throttle", ret ) );
      #endif
    }
  }
  #ifdef ESCPID_DEBUG_MSG
  else if ( ret ) {
    // Process error
    Serial.println( ESCPID_error( "ESCCMD_tic", ret ) );
  }
  #endif
}
