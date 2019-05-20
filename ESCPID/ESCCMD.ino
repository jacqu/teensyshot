/*
 *  ESCCMD:		ESC DSHOT command packets formatting API
 *  
 *  Note:     Best viewed using Arduino IDE with tab space = 2
 *
 *  Authors:  Arda Yiğit and Jacques Gangloff
 *  Date:     May 2019
 */

#include "ESCPID.h"

// Defines
#define ESCCMD_MAX_ESC					ESCPID_NB_ESC				// Max number of ESCs

#define ESCCMD_STATE_INIT				0										// Initial state of the ESC after power on	
#define ESCCMD_STATE_3D_ON			1										// 


// Main structure definition
typedef struct	{
	uint16_t			ESC_state;						// Current state of the ESC
	uint16_t			CRC_errors;						// Overall number of CRC error since start
	uint16_t			last_error;						// Last error code
	uint8_t 			tlm_deg;							// ESC temperature (°C)
  uint16_t 			tlm_volt;							// Voltage of the ESC power supply (0.01V)
  uint16_t 			tlm_amp;							// ESC current (0.01A)
  uint16_t 			tlm_mah;							// ESC consumption (mAh)
  uint16_t 			tlm_rpm;							// ESC electrical rpm (100rpm)
  uint8_t				tlm_valid;						// Flag indicating the validity of telemetry data
	} ESCCMD_STRUCT;
 
//
//	Global variables
//
ESCCMD_STRUCT	ESCCMD[ESCCMD_MAX_ESC];

//
//	Initialization
//
int ESCCMD_init( void )	{
	int i;
	
	// Initialize structures to zero
	for ( i = 0; i < ESCCMD_MAX_ESC; i++ )
		ESCCMD[i] = {};
	
	
	return 0;
}