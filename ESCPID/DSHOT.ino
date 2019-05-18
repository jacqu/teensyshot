/*
 *  DSHOT:    Generation of up to 6 DSHOT signals using DMA
 *  
 *  Note:     Best viewed using Arduino IDE with tab space = 2
 *
 *  Authors:  Arda Yiğit and Jacques Gangloff
 *  Date:     May 2019
 */

#include <Arduino.h>
#include "DMAChannel.h"

//
// Defines
//

#define DSHOT_MAX_OUTPUTS         6             // Maximum number of DSHOT outputs
#define DSHOT_DMA_LENGTH          18            // Number of steps of one DMA sequence (the two last values are zero)
#define DSHOT_DSHOT_LENGTH        16            // Number of bits in a DSHOT sequence
#define DSHOT_BT_DURATION         1670          // Duration of 1 DSHOT600 bit in ns
#define DSHOT_LP_DURATION         1250          // Duration of a DSHOT600 long pulse in ns
#define DSHOT_SP_DURATION         625           // Duration of a DSHOT600 short pulse in ns

//
//  Constants
//

// Defining DSHOT600 timings expressed in F_BUS periods
// DSHOT600 has the following timings:
//
//          1670ns
//          --------->
//          ______
// 1 bit :  |     |___|
//          1250ns
//          ____
// 0 bit :  |   |_____|
//          625ns
//
// On the teensy 3.5, F_BUS == 60000000
const uint16_t DSHOT_short_pulse  = uint64_t(F_BUS) * DSHOT_SP_DURATION / 1000000000;     // DSHOT short pulse duration (nb of F_BUS periods)
const uint16_t DSHOT_long_pulse   = uint64_t(F_BUS) * DSHOT_LP_DURATION / 1000000000;     // DSHOT long pulse duration (nb of F_BUS periods)
const uint16_t DSHOT_bit_length   = uint64_t(F_BUS) * DSHOT_BT_DURATION / 1000000000;     // DSHOT bit duration (nb of F_BUS periods)

//
//  Global variables
//

// DMA FTM channel values references
volatile uint32_t*  DSHOT_DMA_channnel_val[DSHOT_MAX_OUTPUTS] = { &FTM0_C0V,
                                                                  &FTM0_C1V, 
                                                                  &FTM0_C2V, 
                                                                  &FTM0_C3V, 
                                                                  &FTM0_C4V,
                                                                  &FTM0_C5V };

// DMA objects
DMAChannel          dma[DSHOT_MAX_OUTPUTS];

// DMA data
volatile uint16_t   DSHOT_dma_data[DSHOT_MAX_OUTPUTS][DSHOT_DMA_LENGTH];

//
//  DMA termination interrupt service routine (ISR)
//
void DSHOT_DMA_interrupt_routine( void ) {
  
  dma[0].clearInterrupt( );

  // Disable FTM0
  FTM0_SC = 0;
}

//
//  Initialize the DMA hardware in order to be able
//  to generate 6 DSHOT outputs.
//
void DSHOT_init( void ) {
  int i, j;

  // Initialize DMA data
  for ( i = 0; i < DSHOT_MAX_OUTPUTS; i++ )
    for ( j = 0; j < DSHOT_DMA_LENGTH; j++ )
      DSHOT_dma_data[i][j] = 0;

  // Configure pins 6, 9, 10, 20, 22, 23 on the board as DSHOT outputs
  // These pins are configured as FlexTimer (FTM) PWM outputs (FTM0 channel 0-5)
  // PORT_PCR_DSE: high current output
  // PORT_PCR_SRE: slow slew rate
  CORE_PIN22_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;  // FTM0_CH0
  CORE_PIN23_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;  // FTM0_CH1
  CORE_PIN9_CONFIG  = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;  // FTM0_CH2
  CORE_PIN10_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;  // FTM0_CH3
  CORE_PIN6_CONFIG  = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;  // FTM0_CH4
  CORE_PIN20_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;  // FTM0_CH5

  // First DMA channel is the only one triggered by the bit clock
  dma[0].sourceBuffer( DSHOT_dma_data[0], DSHOT_DMA_LENGTH );
  dma[0].destination( (uint16_t&) *DSHOT_DMA_channnel_val[0] );
  dma[0].triggerAtHardwareEvent( DMAMUX_SOURCE_FTM0_CH7 );
  dma[0].interruptAtCompletion( );
  dma[0].attachInterrupt( DSHOT_DMA_interrupt_routine );
  dma[0].enable( );

  // Other DMA channels are trigered by the previoux DMA channel
  for ( i = 1; i < DSHOT_MAX_OUTPUTS; i++ ) {
    dma[i].sourceBuffer( DSHOT_dma_data[i], DSHOT_DMA_LENGTH );
    dma[i].destination( (uint16_t&) *DSHOT_DMA_channnel_val[i] );
    dma[i].triggerAtTransfersOf( dma[i-1] );
    dma[i].triggerAtCompletionOf( dma[i-1] );
    dma[i].enable( );
  }

  // FTM0_CNSC: status and control register
  // FTM0_CNV = 0: initialize the counter channel N at 0
  // FTM_CSC_MSB | FTM_CSC_ELSB | FTM_CSC_CHIE:
  // edge aligned PWM with high-true pulses, interrupt enable
  FTM0_C0SC = FTM_CSC_MSB | FTM_CSC_ELSB | FTM_CSC_CHIE;    
  FTM0_C0V = 0;
  FTM0_C1SC = FTM_CSC_MSB | FTM_CSC_ELSB | FTM_CSC_CHIE;
  FTM0_C1V = 0;
  FTM0_C2SC = FTM_CSC_MSB | FTM_CSC_ELSB | FTM_CSC_CHIE;
  FTM0_C2V = 0;
  FTM0_C3SC = FTM_CSC_MSB | FTM_CSC_ELSB | FTM_CSC_CHIE;
  FTM0_C3V = 0;
  FTM0_C4SC = FTM_CSC_MSB | FTM_CSC_ELSB | FTM_CSC_CHIE;
  FTM0_C4V = 0;
  FTM0_C5SC = FTM_CSC_MSB | FTM_CSC_ELSB | FTM_CSC_CHIE;
  FTM0_C5V = 0;

  // FTM0 channel 7 is the main clock
  // FTM_CSC_CHIE: enable interrupt
  // FTM_CSC_DMA: enable DMA
  // FTM_CSC_MSA: toggle output on match
  // FTM0_C7V = 0: initialize the counter channel 7 at 0
  FTM0_C7SC = FTM_CSC_CHIE | FTM_CSC_DMA | FTM_CSC_MSA | FTM_CSC_ELSA;
  FTM0_C7V = 0;

  // Initialize FTM0
  FTM0_SC = 0;                  // Disable FTM0
  FTM0_CNT = 0;                 // Contains the FTM counter value
  FTM0_MOD = DSHOT_bit_length;  // The modulo value for the FTM counter
  FTM0_CNTIN = 0;               // Counter initial value
  
}

//
//  Send the DSHOT signal through all the configured channels
//  "cmd" points to the DSHOT_MAX_OUTPUTS DSHOT commands to send
//  Telemetry is requested with "tlm", CRC bits are added
//
//  Returns an error code in case of failure, 0 otherwise:
//  -1: DMA error
//  -2: Timeout : DMA duration is abnormally great
//  -3: Value out of range
//  -4: Internal error 
//
int DSHOT_send( uint16_t *cmd, uint8_t *tlm ) {
  int       i, j;
  uint16_t  data;

  // Initialize DMA buffers
  for ( i = 0; i < DSHOT_MAX_OUTPUTS; i++ ) {
    
    // Check cmd value
    if ( cmd[i] > 2047 )
      return -3;
      
    // Compute the packet to send
    // 11 first MSB = command
    // 12th MSB = telemetry request
    // 4 LSB = CRC
    data = ( cmd[i] << 5 ) | ( tlm[i] << 4 );
    data |= ( ( data >> 4 ) ^ ( data >> 8 ) ^ ( data >> 12 ) ) & 0x0f;

    // Generate DSHOT timings corresponding to the packet
    for ( j = 0; j < DSHOT_DSHOT_LENGTH; j++ )  {
      if ( data & ( 1 << ( DSHOT_DSHOT_LENGTH - 1 - j ) ) )
        DSHOT_dma_data[i][j] = DSHOT_long_pulse;
      else
        DSHOT_dma_data[i][j] = DSHOT_short_pulse;
    }
  }
  
  // Clear error flag on all DMA channels
  for ( i = 0; i < DSHOT_MAX_OUTPUTS; i++ )
    dma[i].clearError( );
  
  // Start DMA by activating the clock
  // The clock is disabled again by the DMA interrupt on channel 0
  FTM0_SC = FTM_SC_CLKS(1);

  // Wait the theoretical time needed by DMA + some margin
  delayMicroseconds( (unsigned int)( ( DSHOT_BT_DURATION * ( DSHOT_DMA_LENGTH + 5 ) ) / 1000 ) );

  // Check if FMT0 was disabled by the DMA ISR
  if ( FTM0_SC )
    return -2;

  // Check if there is a DMA error
  for ( i = 0; i < DSHOT_MAX_OUTPUTS; i++ )
    if ( dma[i].error( ) )
      return -1;
  
  return 0;
}