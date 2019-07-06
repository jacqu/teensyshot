/*
 *  Definitions for AWPID.cpp
 *
 */

#ifndef __AWPID_H
#define __AWPID_H

// Defines
#define AWPID_MAX_NB          6    // Maximum number of PIDs

// Function prototypes
void AWPID_init( uint8_t, float*, float*, float*, float*, float*, float* );
void AWPID_reset( void );
void AWPID_control( uint8_t, float, float, float* );
void AWPID_tune( uint8_t, float, float, float, float, float, float );

#endif
