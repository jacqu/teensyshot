#include "DMAChannel.h"
#include <array>

IntervalTimer timer;

DMAChannel dma;
DMAChannel dma2;
DMAChannel dma3;
DMAChannel dma4;
DMAChannel dma5;
DMAChannel dma6;

const uint16_t short_pulse = uint64_t(F_BUS) * 625 / 1000000000;
const uint16_t long_pulse = uint64_t(F_BUS) * 1250 / 1000000000;
const uint16_t bit_length = uint64_t(F_BUS) * 1670 / 1000000000;

std::array<volatile uint16_t, 18> dma_source = {
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  0, 0
};

std::array<volatile uint16_t, 18> dma_source2 = {
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  0, 0
};

std::array<volatile uint16_t, 18> dma_source3 = {
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  0, 0
};

std::array<volatile uint16_t, 18> dma_source4 = {
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  0, 0
};

std::array<volatile uint16_t, 18> dma_source5 = {
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  0, 0
};

std::array<volatile uint16_t, 18> dma_source6 = {
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  short_pulse, long_pulse,
  0, 0
};

void myISR() {
  dma.clearInterrupt();
  FTM0_SC = 0; // disable FTM0
}

void setup() {
  setupDMA();

  timer.begin(myFunc, 200);

  // FTM0_SC = FTM_SC_CLKS(1); // enable timer with busclock
}

void loop() {
}

void myFunc() {
  FTM0_SC = FTM_SC_CLKS(1);
}


void setupDMA() {
  CORE_PIN22_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; // FTM0_CH0
  CORE_PIN23_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; // FTM0_CH1
  CORE_PIN9_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; // FTM0_CH2
  CORE_PIN10_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; // FTM0_CH3
  CORE_PIN6_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; // FTM0_CH4
  CORE_PIN20_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; // FTM0_CH5

  // CORE_PIN5_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;

  dma.sourceBuffer(dma_source.data(), sizeof(dma_source));
  dma.destination((uint16_t&) FTM0_C0V);
  dma.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM0_CH7);
  dma.interruptAtCompletion();
  dma.attachInterrupt(myISR);
  dma.enable();

  dma2.sourceBuffer(dma_source2.data(), sizeof(dma_source2));
  dma2.destination((uint16_t&) FTM0_C1V);
  dma2.triggerAtTransfersOf(dma);
  dma2.triggerAtCompletionOf(dma);
  dma2.enable();

  dma3.sourceBuffer(dma_source3.data(), sizeof(dma_source3));
  dma3.destination((uint16_t&) FTM0_C2V);
  dma3.triggerAtTransfersOf(dma2);
  dma3.triggerAtCompletionOf(dma2);
  dma3.enable();

  dma4.sourceBuffer(dma_source4.data(), sizeof(dma_source4));
  dma4.destination((uint16_t&) FTM0_C3V);
  dma4.triggerAtTransfersOf(dma3);
  dma4.triggerAtCompletionOf(dma3);
  dma4.enable();

  dma5.sourceBuffer(dma_source5.data(), sizeof(dma_source5));
  dma5.destination((uint16_t&) FTM0_C4V);
  dma5.triggerAtTransfersOf(dma4);
  dma5.triggerAtCompletionOf(dma4);
  dma5.enable();

  dma6.sourceBuffer(dma_source6.data(), sizeof(dma_source6));
  dma6.destination((uint16_t&) FTM0_C5V);
  dma6.triggerAtTransfersOf(dma5);
  dma6.triggerAtCompletionOf(dma5);
  dma6.enable();

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

  FTM0_C7SC = FTM_CSC_CHIE | FTM_CSC_DMA | FTM_CSC_MSA | FTM_CSC_ELSA; // output compare, enable DMA trigger
  FTM0_C7V = 0;


  FTM0_SC = 0; // disable FTM0
  FTM0_CNT = 0; // contains the FTM counter value
  FTM0_MOD = bit_length; // the modulo value for the FTM counter
  FTM0_CNTIN = 0; // counter initial value
}
