/*
 *  AWPID:    Anti-windup PID API
 *
 *  Note:     Best viewed using Arduino IDE with tab space = 2
 *
 *  Authors:  Jacques Gangloff
 *  Date:     May 2019
 */

#include "AWPID.h"

// Global variables
double  AWPID_Kp;       // Proportional gain
double  AWPID_Ki;       // Integral gain
double  AWPID_Kd;       // Derivative gain
double  AWPID_f;        // Pole of the derivative term lowpass filter
double  AWPID_Sat;      // Saturation
double  AWPID_u0;       // Control signal before saturation
double  AWPID_e0;       // Error signal
double  AWPID_e1;       // Last sample error signal
double  AWPID_ui0;      // Integral term
double  AWPID_ui1;      // Last sample integral term
double  AWPID_ud0;      // Derivative term
double  AWPID_ud1;      // Last sample derivative term
int     AWPID_nb;       // Number of initialized PIDs

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
void AWPID_init(    double  *Kp,
                    double  *Ki,
                    double  *Kd,
                    double  *f,
                    double  *Sat,
                    int n
                     )  {

  int      i;
  
  if ( n < AWPID_MAX_NB )
    AWPID_nb = n;
  else
    AWPID_nb = AWPID_MAX_NB;
    
  for ( i = 0; i < AWPID_nb; i++ )  {

    // Definition of the PID tuning parameters
    AWPID_Kp =   Kp[i];
    AWPID_Ki =   Ki[i];
    AWPID_Kd =   Kd[i];
    AWPID_f =    f[i];
    AWPID_Sat =  Sat[i];

    // Initialisation of the PID intrnal variables
    AWPID_u0 =   0;

    AWPID_e0 =   0;
    AWPID_e1 =   0;

    AWPID_ui0 =  0;
    AWPID_ui1 =  0;

    AWPID_ud0 =  0;
    AWPID_ud1 =  0;
    }

  return;
  }

//
// Computation of the control signal
//
void AWPID_control( double *Reference,
                    double *Measurement,
                    double *Control )  {

  int    i;

  for ( i = 0; i < AWPID_nb; i++ )  {

    // Computation of the error
    AWPID_e1 = AWPID_e0;
    AWPID_e0 = Reference[i] - Measurement[i];

    // Computation of the derivative term
    AWPID_ud1 =   AWPID_ud0;
    AWPID_ud0 =   AWPID_f * AWPID_ud1 +
                  AWPID_Kd * ( AWPID_e0 - AWPID_e1 );

    // Integral term computation

    AWPID_ui1 =   AWPID_ui0;

    // Anti-windup only if the integral gain is non null
    if ( AWPID_Ki )  {

      AWPID_u0 =  AWPID_Kp * AWPID_e0 +
                  AWPID_ud0 +
                  AWPID_ui1 + AWPID_Ki * AWPID_e0;

      // No saturation
      if ( fabs ( AWPID_u0 ) <= AWPID_Sat )
        AWPID_ui0 = AWPID_ui1 + AWPID_Kp * AWPID_Ki * AWPID_e0;

      // Upper limit saturation: recalculation of the integral term
      // With this adjusment, the control signal equals exactly the saturation
      if ( AWPID_u0 > AWPID_Sat )
        AWPID_ui0 = ( AWPID_Sat -
                    ( AWPID_Kp * AWPID_e0 + AWPID_ud0 ) );

      // Lower limit saturation: recalculation of the integral term
      // With this adjusment, the control signal equals exactly the saturation
      if ( AWPID_u0 < -AWPID_Sat )
        AWPID_ui0 = ( -AWPID_Sat -
                    ( AWPID_Kp * AWPID_e0 + AWPID_ud0 ) );
      }

    // Control signal computation
    Control[i] =  AWPID_Kp * AWPID_e0 +
                  AWPID_ud0 +
                  AWPID_ui0;

    // Saturation
    if ( Control[i] > AWPID_Sat )
      Control[i] = AWPID_Sat;

    if ( Control[i] < -AWPID_Sat )
      Control[i] = -AWPID_Sat;
    }

  return;
  }
