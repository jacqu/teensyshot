#include "DMAChannel.h"

#define DSHOT_FREQ 500

#define TLM_LENGTH 10
#define DSHOT_LENGTH 16
#define DMA_LENGTH 18

const uint16_t SP = uint64_t(F_BUS) * 625 / 1000000000; //short pulse
const uint16_t LP = uint64_t(F_BUS) * 1250 / 1000000000; // long pulse
const uint16_t BIT_LENGTH = uint64_t(F_BUS) * 1670 / 1000000000;

IntervalTimer timer; 

DMAChannel dma;
volatile uint16_t dma_source[DMA_LENGTH];
volatile uint16_t dataToSend;
volatile bool flagTlm = false;
volatile bool requestTlm = false; 
volatile uint16_t throttle;

volatile long counter = 0;

volatile bool armed = false;
volatile bool commandSent = false; 

#define DEFAULT_MODE 0
#define ARMING_MODE 1
#define SEND_COMMAND_MODE 2
#define STOP_MODE 3

volatile uint8_t mode = DEFAULT_MODE; 

void myISR() {
  dma.clearInterrupt();
  FTM0_SC = 0;
  //Serial1.flush(); 
  while(Serial1.available()){Serial1.read();};
}

void sendDshot() {
  dataToSend = uint16_t((throttle << 5)) | uint16_t((requestTlm << 4));
  dataToSend |= ((dataToSend >> 4) ^ (dataToSend >> 8) ^ (dataToSend >> 12)) & 0x0f; // csc
  for (int i = 0; i < DSHOT_LENGTH; i++) {
    dma_source[i] = (dataToSend & (1 << (DSHOT_LENGTH - 1 - i))) ? LP : SP;
  }
  FTM0_SC = FTM_SC_CLKS(1);
  if (requestTlm) {
    flagTlm = true; 
  }
}

void ISR_timer() {
  sendDshot();
  if (mode == ARMING_MODE) {
    if (!armed && ++counter > 10) {
      armed = true;
      counter = 0; 
    }
  } else if (mode == SEND_COMMAND_MODE) {
    if (!commandSent && counter <= 10) {
      counter++; 
    }
    if (!commandSent && counter > 10) {
      commandSent = true; 
      counter = 0; 
    }
  }
}

void setupDMA() {
  CORE_PIN9_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;

  dma.sourceBuffer(dma_source, sizeof(dma_source));
  dma.destination((uint16_t&) FTM0_C2V);
  dma.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM0_CH7);
  dma.interruptAtCompletion();
  dma.attachInterrupt(myISR);
  dma.enable();

  FTM0_C2SC = FTM_CSC_MSB | FTM_CSC_ELSB; // edge-aligned pwm output
  FTM0_C2V = 0;
  FTM0_C7SC = FTM_CSC_CHIE | FTM_CSC_DMA | FTM_CSC_MSA | FTM_CSC_ELSA;
  FTM0_C7V = 0;

  FTM0_SC = 0;
  FTM0_CNT = 0;
  FTM0_MOD = BIT_LENGTH;
  FTM0_CNTIN = 0;
}

uint8_t updateCrc8(uint8_t crc, uint8_t crc_seed)
{
  uint8_t crc_u = crc;
  crc_u ^= crc_seed;

  for (int i = 0; i < 8; i++) {
    crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  }

  return (crc_u);
}

uint8_t calculateCrc8(const uint8_t *Buf, const uint8_t BufLen)
{
  uint8_t crc = 0;
  for (int i = 0; i < BufLen; i++) {
    crc = updateCrc8(Buf[i], crc);
  }

  return crc;
}
