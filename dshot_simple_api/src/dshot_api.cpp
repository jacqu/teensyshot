#include <Arduino.h>
#include "dshot_api.h"

DMAChannel dma;
volatile uint16_t dma_source[DMA_LENGTH];
volatile uint16_t dataToSend;


volatile bool flagTlm = false;
volatile bool requestTlm = false;
volatile uint16_t throttle;

volatile uint8_t mode = DEFAULT_MODE;

volatile long counter = 0;
volatile bool armed = false;
volatile bool commandSent = false;

IntervalTimer timer;

struct TlmData tlmData;

void readTlm() {
  noInterrupts();
  if (isTlmAvailable()) {
    resetTlmFlag();
    interrupts();
    static uint8_t bufferTlm[TLM_LENGTH];
    while (Serial1.available() < TLM_LENGTH);
    // noInterrupts();
    for (int i = 0; i < TLM_LENGTH; i++) {
      bufferTlm[i] = Serial1.read();
    }
    tlmData.temperature = bufferTlm[0];
    tlmData.voltage = (bufferTlm[1] << 8) | bufferTlm[2];
    tlmData.current = (bufferTlm[3] << 8) | bufferTlm[4];
    tlmData.consumption = (bufferTlm[5] << 8) | bufferTlm[6];
    tlmData.rpm = (bufferTlm[7] << 8) | bufferTlm[8];
    tlmData.crcCheck = bufferTlm[9] == calculateCrc8(bufferTlm, TLM_LENGTH-1);

    /* TODO: stat for crcError */
    if (tlmData.crcCheck) {
      /* no error */
    } else {
      /* crc error */
    }
    /*for (int i = 0; i < TLM_LENGTH; i++) { // for debug
      Serial.print(bufferTlm[i]);
      Serial.print(" ");
    }
    Serial.println();*/
  } else {
    interrupts(); 
  }
}

uint16_t getRpm() {
  return tlmData.rpm;
}

void startDshot() {
  setupDMA();
  delay(100);
  disableTlm();
}

bool isTlmAvailable() {
  return flagTlm;
}

void resetTlmFlag() {
  flagTlm = false;
}

void armingMotor() {
  if (!armed) {
    mode = ARMING_MODE;
    throttle = 0;
    timer.begin(ISR_timer, 2000); // 2ms (500Hz)
    while (!armed);
    noInterrupts();
    mode = DEFAULT_MODE;
    interrupts();
  }
}

void enable3d() {
  uint16_t copyThrottle = throttle;
  noInterrupts();
  mode = SEND_COMMAND_MODE;
  throttle = 10;
  interrupts();
  while(!commandSent);
  noInterrupts();
  commandSent = false;
  throttle = 12;
  interrupts();
  while(!commandSent);
  noInterrupts();
  commandSent = false;
  mode = DEFAULT_MODE;
  throttle = copyThrottle;
  interrupts();
}

void disable3d() {
  uint16_t copyThrottle = throttle;
  noInterrupts();
  mode = SEND_COMMAND_MODE;
  throttle = 9;
  interrupts();
  while(!commandSent);
  noInterrupts();
  commandSent = false;
  throttle = 12;
  interrupts();
  while(!commandSent);
  noInterrupts();
  commandSent = false;
  mode = DEFAULT_MODE;
  throttle = copyThrottle;
  interrupts();
}

void setControlLoopFrequency(uint16_t freq) {
  timer.update(1000000 / freq);
}

void enableTlm() {
  if (!requestTlm) {
    noInterrupts();
    requestTlm = true;
    interrupts();
  }
}

void disableTlm() {
  if (requestTlm) {
    noInterrupts();
    requestTlm = false;
    interrupts();
  }
}

void setThrottle(uint16_t newThrottle) {
  // maybe add saturation
  if (armed) {
    noInterrupts();
    throttle = newThrottle;
    requestTlm = true;
    interrupts();
  }
}

void stopMotor() {
  throttle = 0;
  mode = STOP_MODE;
}




void sendDshot() {
  dataToSend = uint16_t((throttle << 5)) | uint16_t((requestTlm << 4));
  dataToSend |= ((dataToSend >> 4) ^ (dataToSend >> 8) ^ (dataToSend >> 12)) & 0x0f; // crc
  for (int i = 0; i < DSHOT_LENGTH; i++) {
    dma_source[i] = (dataToSend & (1 << (DSHOT_LENGTH - 1 - i))) ? LP : SP;
  }
  FTM0_SC = FTM_SC_CLKS(1);
  if (requestTlm) {
    flagTlm = true;
  }
}

// modify this function for control loop
void ISR_timer() {
  /* TODO: add PID control */
  // throttle = ...;

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

void ISR_DMA() {
  dma.clearInterrupt();
  FTM0_SC = 0;
  while (Serial1.available()) {
    Serial1.read();
  }
}

void setupDMA() {
  CORE_PIN9_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;

  dma.sourceBuffer(dma_source, sizeof(dma_source));
  dma.destination((uint16_t&) FTM0_C2V);
  dma.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM0_CH7);
  dma.interruptAtCompletion();
  dma.attachInterrupt(ISR_DMA);
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

uint8_t updateCrc8(uint8_t crc, uint8_t crc_seed) {
  uint8_t crc_u = crc;
  crc_u ^= crc_seed;

  for (int i = 0; i < 8; i++) {
    crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  }

  return (crc_u);
}

uint8_t calculateCrc8(const uint8_t *Buf, const uint8_t BufLen) {
  uint8_t crc = 0;
  for (int i = 0; i < BufLen; i++) {
    crc = updateCrc8(Buf[i], crc);
  }

  return crc;
}
