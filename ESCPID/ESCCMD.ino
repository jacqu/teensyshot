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
#define ESCCMD_MAX_ESC					ESCPID_NB_ESC							\\ Max number of ESCs
 
// Main structure definition
typedef struct	{
	uint16_t			ESC_state;
	uint16_t			CRC_errors;
	uint16_t			last_error;
	uint8_t 			tlm_temp;
  uint16_t 			tlm_volt;
  uint16_t 			tlm_amp;
  uint16_t 			tlm_mah;
  uint16_t 			tlm_rpm;
	} ESCCMD_STRUCT;
 
//
//	
//