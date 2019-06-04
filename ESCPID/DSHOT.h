/*
 *  Definitions for DSHOT.cpp
 *
 */

#ifndef __DSHOT_H
#define __DSHOT_H

//
// Defines
//

#define DSHOT_MAX_OUTPUTS         ESCPID_NB_ESC // Maximum number of DSHOT outputs
#define DSHOT_NB_DMA_CHAN         6             // Number of accessible DMA channels
#if DSHOT_NB_DMA_CHAN < DSHOT_MAX_OUTPUTS
  #error ESCCMD_NB_UART should be >= DSHOT_MAX_OUTPUTS
#endif
#define DSHOT_DMA_LENGTH          18            // Number of steps of one DMA sequence (the two last values are zero)
#define DSHOT_DMA_MARGIN          2              // Number of additional bit duration to wait until checking if DMA is over
#define DSHOT_DSHOT_LENGTH        16            // Number of bits in a DSHOT sequence
#define DSHOT_BT_DURATION         1670          // Duration of 1 DSHOT600 bit in ns
#define DSHOT_LP_DURATION         1250          // Duration of a DSHOT600 long pulse in ns
#define DSHOT_SP_DURATION         625           // Duration of a DSHOT600 short pulse in ns


#endif