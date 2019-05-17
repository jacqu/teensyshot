/*
 *  DSHOT:    Generation of up to 6 DSHOT signals using DMA
 *  
 *  Note:     Best viewed using Arduino IDE with tab space = 2
 *
 *  Authors:  Arda Yiğit and Jacques Gangloff
 *  Date:     May 2019
 */
 
//
// Defines
//

#define DSHOT_MAX_OUTPUTS         6             // Maximum number of DSHOT outputs
#define DSHOT_DMA_LENGTH          18            // Number of steps of one DMA sequence (the two last values are zero)
#define DSHOT_DSHOT_LENGTH        16            // Number of bits in a DSHOT sequence

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
const uint16_t DSHOT_short_pulse  = uint64_t(F_BUS) * 625 / 1000000000;     // DSHOT short pulse duration (nb of F_BUS periods)
const uint16_t DSHOT_long_pulse   = uint64_t(F_BUS) * 1250 / 1000000000;    // DSHOT long pulse duration (nb of F_BUS periods)
const uint16_t DSHOT_bit_length   = uint64_t(F_BUS) * 1670 / 1000000000;    // DSHOT bit duration (nb of F_BUS periods)

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
  
  dma.clearInterrupt( );

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

  for ( i = 1; i < DSHOT_MAX_OUTPUTS; i++ ) {
    // Other DMA channels are trigered by the previoux DMA channel
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
