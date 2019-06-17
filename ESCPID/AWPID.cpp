/*
 *  AWPID:    Anti-windup PID API
 *
 *  Note:     Best viewed using Arduino IDE with tab space = 2
 *
 *  Authors:  Jacques Gangloff
 *  Date:     May 2019
 */

// Includes
#include <Arduino.h>
#include <math.h>
#include "AWPID.h"

// Global variables
double  AWPID_Kp[AWPID_MAX_NB];       // Proportional gain
double  AWPID_Ki[AWPID_MAX_NB];       // Integral gain
double  AWPID_Kd[AWPID_MAX_NB];       // Derivative gain
double  AWPID_f[AWPID_MAX_NB];        // Pole of the derivative term lowpass filter
double  AWPID_Min[AWPID_MAX_NB];      // Lower saturation
double  AWPID_Max[AWPID_MAX_NB];      // Upper saturation
double  AWPID_u0[AWPID_MAX_NB];       // Control signal before saturation
double  AWPID_e0[AWPID_MAX_NB];       // Error signal
double  AWPID_e1[AWPID_MAX_NB];       // Last sample error signal
double  AWPID_ui0[AWPID_MAX_NB];      // Integral term
double  AWPID_ui1[AWPID_MAX_NB];      // Last sample integral term
double  AWPID_ud0[AWPID_MAX_NB];      // Derivative term
double  AWPID_ud1[AWPID_MAX_NB];      // Last sample derivative term
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
                    double  *Kp,
                    double  *Ki,
                    double  *Kd,
                    double  *f,
                    double  *Min,
                    double  *Max
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

    // Initialisation of the PID intrnal variables
    AWPID_u0[i] =   0;

    AWPID_e0[i] =   0;
    AWPID_e1[i] =   0;

    AWPID_ui0[i] =  0;
    AWPID_ui1[i] =  0;

    AWPID_ud0[i] =  0;
    AWPID_ud1[i] =  0;
    }

  return;
  }

//
// Computation of the ith control signal
//
void AWPID_control( uint8_t i,
                    double Reference,
                    double Measurement,
                    double *Control )  {

  if ( i >= AWPID_n )
    return;

  // Computation of the error
  AWPID_e1[i] = AWPID_e0[i];
  AWPID_e0[i] = Reference - Measurement;

  // Computation of the derivative term
  AWPID_ud1[i] =  AWPID_ud0[i];
  AWPID_ud0[i] =  AWPID_f[i] * AWPID_ud1[i] +
                  AWPID_Kd[i] * ( AWPID_e0[i] - AWPID_e1[i] );

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
                    double  Kp,
                    double  Ki,
                    double  Kd,
                    double  f,
                    double  Min,
                    double  Max
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
