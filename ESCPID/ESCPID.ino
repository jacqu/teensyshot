/*
 *  ESCPID:   PID control of up to 6 ESCs using teensy 3.5 MCU
 *
 *  Note:     Best viewed using Arduino IDE with tab space = 2
 *
 *  Authors:  Arda Yiğit and Jacques Gangloff
 *  Date:     May 2019
 */

#include <Arduino.h>
#include "DSHOT.h"
#include "ESCCMD.h"
#include "AWPID.h"
#include "ESCPID.h"

// Globals
double  ESCPID_Reference[ESCPID_NB_ESC] = {};
double  ESCPID_Measurement[ESCPID_NB_ESC] = {};
double  ESCPID_Control[ESCPID_NB_ESC] = {};
char    ESCPID_error_msg[ESCPID_ERROR_MSG_LENGTH];

double  ESCPID_Kp[ESCPID_NB_ESC];
double  ESCPID_Ki[ESCPID_NB_ESC];
double  ESCPID_Kd[ESCPID_NB_ESC];
double  ESCPID_f[ESCPID_NB_ESC];
double  ESCPID_Sat[ESCPID_NB_ESC];

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
  
  // Wait for serial link to initialize
  while ( !Serial );

  // Initialize PID gains
  for ( i = 0; i < ESCPID_NB_ESC; i++ ) {
    ESCPID_Kp[i] =  ESCPID_PID_P;
    ESCPID_Ki[i] =  ESCPID_PID_I;
    ESCPID_Kd[i] =  ESCPID_PID_D;
    ESCPID_f[i] =   ESCPID_PID_f;
    ESCPID_Sat[i] = ESCPID_PID_SAT;
  }

  // Initialize PID subsystem
  //AWPID_init( ESCPID_Kp, ESCPID_Ki, ESCPID_Kd, ESCPID_f, ESCPID_Sat, ESCPID_NB_ESC );

  // Initialize the CMD subsystem
  ESCCMD_init( ESCPID_NB_ESC );

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

  // Arming ESCs
  ret = ESCCMD_arm_all( );

  // Process error
  if ( ret )
    Serial.println( ESCPID_error( "ESCCMD_arm_all", ret ) );

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

    // Read all measurements and compute current control signal
    for ( i = 0; i < ESCPID_NB_ESC; i++ )
      if ( !( ESCCMD_read_RPM( i, &ESCPID_Measurement[i] ) ) )
        AWPID_control( ESCPID_Reference, ESCPID_Measurement, ESCPID_Control );

    // Send control signal
    for ( i = 0; i < ESCPID_NB_ESC; i++ )
      if ( ( ret = ESCCMD_throttle( i, (int16_t)ESCPID_Control[i] ) ) )
        Serial.println( ESCPID_error( "ESCCMD_throttle", ret ) );
  }
  else if ( ret ) {
    // Process error
    Serial.println( ESCPID_error( "ESCCMD_tic", ret ) );
  }
}
