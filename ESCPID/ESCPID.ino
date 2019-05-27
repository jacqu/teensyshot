/*
 *  ESCPID:   PID control of up to 6 ESCs using teensy 3.5 MCU
 *
 *  Note:     Best viewed using Arduino IDE with tab space = 2
 *
 *  Authors:  Arda Yiğit and Jacques Gangloff
 *  Date:     May 2019
 */

#include <Arduino.h>
#include "ESCPID.h"
#include "ESCCMD.h"

// Defines
#define ESCPID_USB_UART_SPEED           115200      // Baudrate of the teeensy USB serial link
#define ESCPID_ERROR_MSG_LENGTH         80          // Max string length of an error message

// Constants
const double ESCPID_Kp[ESCPID_NB_ESC] =   { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
const double ESCPID_Ki[ESCPID_NB_ESC] =   { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
const double ESCPID_Kd[ESCPID_NB_ESC] =   { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
const double ESCPID_f[ESCPID_NB_ESC] =    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
const double ESCPID_Sat[ESCPID_NB_ESC] =  { 999.0, 999.0, 999.0, 999.0, 999.0, 999.0 };

// Globals
double  ESCPID_Reference[ESCPID_NB_ESC] = {};
double  ESCPID_Measurement[ESCPID_NB_ESC] = {};
double  ESCPID_Control[ESCPID_NB_ESC] = {};
char    ESCPID_error_msg[ESCPID_ERROR_MSG_LENGTH];

//
//  Error processing
//
char *ESCPID_error( char *prefix, int ret ) {
  
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
                "Invalid function call sequence error" );
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
    case ESCCMD_ERROR_CRC:
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
  int i, ret;
  
  // Initialize USB serial link
  Serial.begin( ESCPID_USB_UART_SPEED );
  
  // Initialize DSHOT subsystem
  DSHOT_init( );
  
  // Initialize PID subsystem
  AWPID_init( ESCPID_Kp, ESCPID_Ki, ESCPID_Kd, ESCPID_f, ESCPID_Sat );
  
  // Initialize the CMD subsystem
  ESCCMD_init( );
  
  // Arming ESCs
  ret = ESCCMD_arm_all( );
  
  // Process error
  if ( ret )
    Serial.println( ESCPID_error( "ESCCMD_arm_all", ret ) );
    
  // Switch 3D mode on
  ret = ESCCMD_3D_on( );
  
  // Process error
  if ( ret )
    Serial.println( ESCPID_error( "ESCCMD_3D_on", ret ) );
  
  // Start periodic loop
  ret = ESCCMD_start_timer( );
  
  // Process error
  if ( ret )
    Serial.println( ESCPID_error( "ESCCMD_start_timer", ret ) );
    
  // Start all motors
  for ( i = 0; i < ESCPID_NB_ESC; i++ )
    if ( ( ret = ESCCMD_throttle( i, 0 ) ) )
      Serial.println( ESCPID_error( "ESCCMD_throttle", ret ) );
  
  // Init finished
  Serial.println( "ESCPID setup complete" );
}

//
//  Arduino main loop
//
void loop() {
  int i, ret;
  
  // Check for next timer event
  ret = ESCCMD_tic( );
  if ( ret == ESCCMD_TIC_OCCURED )  {
    
    // Process timer event
    
    // Read all measurements
    for ( i = 0; i < ESCPID_NB_ESC; i++ )
      if ( ( ret = ESCCMD_read_RPM( i, &ESCPID_Measurement[i] ) ) )
        Serial.println( ESCPID_error( "ESCCMD_read_RPM", ret ) );
    
    // Compute current control signal
    AWPID_control( ESCPID_Reference, ESCPID_Measurement, ESCPID_Control );
    
    // Send control signal
    for ( i = 0; i < ESCPID_NB_ESC; i++ )
      if ( ( ret = ESCCMD_throttle( i, (int16_t)ESCPID_Control[i] ) ) )
        Serial.println( ESCPID_error( "ESCCMD_throttle", ret ) );
  }
  else {
    // Process error
    Serial.println( ESCPID_error( "ESCCMD_tic", ret ) );
  }
}
