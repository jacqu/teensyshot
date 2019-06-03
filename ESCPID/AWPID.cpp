/*
 *  AWPID:    Anti-windup PID API
 *  
 *  Note:     Best viewed using Arduino IDE with tab space = 2
 *
 *  Authors:  Jacques Gangloff
 *  Date:     May 2019
 */
 
// Includes
#include <math.h>
#include "ESCPID.h"

// Defines
#define AWPID_MAX_NB          ESCPID_NB_ESC    // Maximum number of PIDs

// Main structure definition
typedef struct  {
  double Kp;        // Proportional gain
  double Ki;        // Integral gain
  double Kd;        // Derivative gain
  double f;         // Pole of the derivative term lowpass filter
  double Sat;       // Saturation
  double u0;        // Control signal before saturation
  double e0;        // Error signal
  double e1;        // Last sample error signal
  double ui0;       // Integral term
  double ui1;       // Last sample integral term
  double ud0;       // Derivative term
  double ud1;       // Last sample derivative term
  } AWPID_STRUCT;

// Global variables
AWPID_STRUCT  AWPID[AWPID_MAX_NB];

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
                    double  *Sat
                     )  {
  
  int      i;
  
  for ( i = 0; i < AWPID_MAX_NB; i++ )  {
    
    // Definition of the PID tuning parameters
    AWPID[i].Kp =   Kp[i];
    AWPID[i].Ki =   Ki[i];
    AWPID[i].Kd =   Kd[i];
    AWPID[i].f =    f[i];
    AWPID[i].Sat =  Sat[i];
    
    // Initialisation of the PID intrnal variables
    AWPID[i].u0 =   0;
    
    AWPID[i].e0 =   0;
    AWPID[i].e1 =   0;
    
    AWPID[i].ui0 =  0;
    AWPID[i].ui1 =  0;
    
    AWPID[i].ud0 =  0;
    AWPID[i].ud1 =  0;
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
  
  for ( i = 0; i < AWPID_MAX_NB; i++ )  {
    
    // Computation of the error
    AWPID[i].e1 = AWPID[i].e0;
    AWPID[i].e0 = Reference[i] - Measurement[i];
    
    // Computation of the derivative term
    AWPID[i].ud1 =   AWPID[i].ud0;
    AWPID[i].ud0 =   AWPID[i].f * AWPID[i].ud1 +
                    AWPID[i].Kd * ( AWPID[i].e0 - AWPID[i].e1 );
    
    // Integral term computation
    
    AWPID[i].ui1 =   AWPID[i].ui0;
    
    // Anti-windup only if the integral gain is non null
    if ( AWPID[i].Ki )  {
    
      AWPID[i].u0 =   AWPID[i].Kp * AWPID[i].e0 +
                      AWPID[i].ud0 +
                      AWPID[i].ui1 + AWPID[i].Ki * AWPID[i].e0;
      
      // No saturation
      if ( fabs ( AWPID[i].u0 ) <= AWPID[i].Sat )  
        AWPID[i].ui0 = AWPID[i].ui1 + AWPID[i].Kp * AWPID[i].Ki * AWPID[i].e0;
      
      // Upper limit saturation: recalculation of the integral term
      // With this adjusment, the control signal equals exactly the saturation
      if ( AWPID[i].u0 > AWPID[i].Sat )  
        AWPID[i].ui0 = ( AWPID[i].Sat - 
                  ( AWPID[i].Kp * AWPID[i].e0 + AWPID[i].ud0 ) );
      
      // Lower limit saturation: recalculation of the integral term
      // With this adjusment, the control signal equals exactly the saturation
      if ( AWPID[i].u0 < -AWPID[i].Sat )  
        AWPID[i].ui0 = ( -AWPID[i].Sat - 
                  ( AWPID[i].Kp * AWPID[i].e0 + AWPID[i].ud0 ) );
      }
      
    // Control signal computation
    Control[i] =   AWPID[i].Kp * AWPID[i].e0 + 
                  AWPID[i].ud0 + 
                  AWPID[i].ui0;
            
    // Saturation
    if ( Control[i] > AWPID[i].Sat )
      Control[i] = AWPID[i].Sat;
      
    if ( Control[i] < -AWPID[i].Sat )
      Control[i] = -AWPID[i].Sat;
    }
  
  return;
  }
