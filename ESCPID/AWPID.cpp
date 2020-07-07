/*
 *  AWPID:    Anti-windup PID API
 *
 *  Note:     Best viewed using Arduino IDE with tab space = 2
 *
 *  Authors:  Jacques Gangloff
 *  Date:     May 2019
 */

#define AWPID_FILTERED_MES

// Includes
#include <Arduino.h>
#include <math.h>
#include "AWPID.h"

// Global variables
float   AWPID_Kp[AWPID_MAX_NB];       // Proportional gain
float   AWPID_Ki[AWPID_MAX_NB];       // Integral gain
float   AWPID_Kd[AWPID_MAX_NB];       // Derivative gain
float   AWPID_f[AWPID_MAX_NB];        // Pole of the derivative term lowpass filter
float   AWPID_Min[AWPID_MAX_NB];      // Lower saturation
float   AWPID_Max[AWPID_MAX_NB];      // Upper saturation
float   AWPID_u0[AWPID_MAX_NB];       // Control signal before saturation
float   AWPID_e0[AWPID_MAX_NB];       // Error signal
float   AWPID_e1[AWPID_MAX_NB];       // Last sample error signal
float   AWPID_ui0[AWPID_MAX_NB];      // Integral term
float   AWPID_ui1[AWPID_MAX_NB];      // Last sample integral term
float   AWPID_ud0[AWPID_MAX_NB];      // Derivative term
float   AWPID_ud1[AWPID_MAX_NB];      // Last sample derivative term
#ifdef AWPID_FILTERED_MES
float   AWPID_me1[AWPID_MAX_NB];      // Last sample measurement
uint8_t AWPID_minit[AWPID_MAX_NB];    // Init flag for mes history
#endif
uint8_t AWPID_n = 0;                  // Number of initialized PIDs

//
//  Initialisation of the PID parametrs
//
//  Nb:   number of controllers
//  Kp:   proportional gain
//  Ki:   integral gain
//  Kd:   derivative gain
//  f:    derivative filter pole
//        ( 0 < f < 1, if f == 1, derivative action is cancelled)
//  Sat:  saturation (>0)
//
//  Controller equation:
//
//  C(z) = Kp + Ki z / ( z - 1 ) + Kd( z - 1 ) / (z - f )
//
void AWPID_init(    uint8_t n,
                    float  *Kp,
                    float  *Ki,
                    float  *Kd,
                    float  *f,
                    float  *Min,
                    float  *Max
                     )  {

  int      i;

  if ( n <= AWPID_MAX_NB )
    AWPID_n = n;
  else
    AWPID_n = AWPID_MAX_NB;

  for ( i = 0; i < AWPID_n; i++ )  {

    // Definition of the PID tuning parameters
    AWPID_Kp[i] =   Kp[i];
    AWPID_Ki[i] =   Ki[i];
    AWPID_Kd[i] =   Kd[i];
    AWPID_f[i] =    f[i];
    AWPID_Min[i] =  Min[i];
    AWPID_Max[i] =  Max[i];

    // Initialisation of the PID internal variables
    AWPID_u0[i] =   0.0;

    AWPID_e0[i] =   0.0;
    AWPID_e1[i] =   0.0;

    AWPID_ui0[i] =  0.0;
    AWPID_ui1[i] =  0.0;

    AWPID_ud0[i] =  0.0;
    AWPID_ud1[i] =  0.0;
    
    #ifdef AWPID_FILTERED_MES
    AWPID_me1[i] =  0.0;
    AWPID_minit[i] = 0;
    #endif
    }

  return;
  }

//
//  Reset internal variables of the PID
//
void AWPID_reset( void )  {
  int i;
  
  for ( i = 0; i < AWPID_n; i++ )  {
    // Initialisation of the PID internal variables
    AWPID_u0[i] =   0.0;

    AWPID_e0[i] =   0.0;
    AWPID_e1[i] =   0.0;

    AWPID_ui0[i] =  0.0;
    AWPID_ui1[i] =  0.0;

    AWPID_ud0[i] =  0.0;
    AWPID_ud1[i] =  0.0;
    
    #ifdef AWPID_FILTERED_MES
    AWPID_me1[i] =  0.0;
    AWPID_minit[i] = 0;
    #endif
  }
}

//
//  Computation of the ith control signal
//
void AWPID_control( uint8_t i,
                    float Reference,
                    float Measurement,
                    float *Control )  {

  if ( i >= AWPID_n )
    return;

  // Computation of the error
  AWPID_e1[i] = AWPID_e0[i];
  AWPID_e0[i] = Reference - Measurement;

  // Computation of the derivative term
  AWPID_ud1[i] =  AWPID_ud0[i];
  #ifdef AWPID_FILTERED_MES
    if ( AWPID_minit[i] == 0 )  {
      AWPID_me1[i] = Measurement;
      AWPID_minit[i] = 1;
    }
    AWPID_ud0[i] =  AWPID_f[i] * AWPID_ud1[i] -
                    AWPID_Kd[i] * ( Measurement - AWPID_me1[i] );
    AWPID_me1[i] = Measurement;
  #else
  AWPID_ud0[i] =  AWPID_f[i] * AWPID_ud1[i] +
                  AWPID_Kd[i] * ( AWPID_e0[i] - AWPID_e1[i] );
  #endif

  // Integral term computation

  AWPID_ui1[i] =   AWPID_ui0[i];

  // Anti-windup only if the integral gain is non null
  if ( AWPID_Ki[i] )  {

    AWPID_u0[i] = AWPID_Kp[i] * AWPID_e0[i] +
                  AWPID_ud0[i] +
                  AWPID_ui1[i] + AWPID_Ki[i] * AWPID_e0[i];

    // No saturation
    if ( ( AWPID_u0[i] <= AWPID_Max[i] ) && ( AWPID_u0[i] >= AWPID_Min[i] ) )
      AWPID_ui0[i] = AWPID_ui1[i] + AWPID_Kp[i] * AWPID_Ki[i] * AWPID_e0[i];

    // Upper limit saturation: recalculation of the integral term
    // With this adjusment, the control signal equals exactly the saturation
    if ( AWPID_u0[i] > AWPID_Max[i] )
      AWPID_ui0[i] =  ( AWPID_Max[i] -
                      ( AWPID_Kp[i] * AWPID_e0[i] + AWPID_ud0[i] ) );

    // Lower limit saturation: recalculation of the integral term
    // With this adjusment, the control signal equals exactly the saturation
    if ( AWPID_u0[i] < AWPID_Min[i] )
      AWPID_ui0[i] =  ( AWPID_Min[i] -
                      ( AWPID_Kp[i] * AWPID_e0[i] + AWPID_ud0[i] ) );
    }

  // Control signal computation
  *Control = AWPID_Kp[i] * AWPID_e0[i] + AWPID_ud0[i] + AWPID_ui0[i];

  // Saturation
  if ( *Control > AWPID_Max[i] )
    *Control = AWPID_Max[i];

  if ( *Control < AWPID_Min[i] )
    *Control = AWPID_Min[i];

  return;
  }

//
// Tuning of the ith PID.
// Can be called on-the-fly. 
//
void AWPID_tune(    uint8_t i,
                    float  Kp,
                    float  Ki,
                    float  Kd,
                    float  f,
                    float  Min,
                    float  Max
                     )  {
  if ( i >= AWPID_n )
    return;
  
  // Definition of the PID tuning parameters
  AWPID_Kp[i] =   Kp;
  AWPID_Ki[i] =   Ki;
  AWPID_Kd[i] =   Kd;
  AWPID_f[i] =    f;
  AWPID_Min[i] =  Min;
  AWPID_Max[i] =  Max;

}                  
