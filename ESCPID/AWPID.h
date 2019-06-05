/*
 *  Definitions for AWPID.cpp
 *
 */

#ifndef __AWPID_H
#define __AWPID_H

// Defines
#define AWPID_MAX_NB          6    // Maximum number of PIDs

// Includes
#include <math.h>

// Function prototypes
void AWPID_init( double*, double*, double*, double*, double*, int );
void AWPID_control( double*, double*, double* );

#endif
