/*
 *  Definitions for AWPID.cpp
 *
 */
 
#ifndef __ESCCMD_H
#define __ESCCMD_H

// Defines
#define AWPID_MAX_NB          6    // Maximum number of PIDs

// Functions
void AWPID_init( double*, double*, double*, double*, double* );
void AWPID_control( double*, double*, double* );

#endif