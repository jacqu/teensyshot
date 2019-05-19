#ifndef DHOT_API_H
#define DHOT_API_H

#include <Arduino.h>
#include "DMAChannel.h"

#define DSHOT_FREQ 500

#define TLM_LENGTH 10
#define DSHOT_LENGTH 16
#define DMA_LENGTH 18

//const uint16_t SP = uint64_t(F_BUS) * 625 / 1000000000; //short pulse
//const uint16_t LP = uint64_t(F_BUS) * 1250 / 1000000000; // long pulse
//const uint16_t BIT_LENGTH = uint64_t(F_BUS) * 1670 / 1000000000;
// F_BUS is 60MHz
#define SP 37
#define LP 75
#define BIT_LENGTH 100

#define DEFAULT_MODE 0
#define ARMING_MODE 1
#define SEND_COMMAND_MODE 2
#define STOP_MODE 3

struct TlmData {
  uint8_t temperature;
  uint16_t voltage;
  uint16_t current;
  uint16_t consumption;
  uint16_t rpm;
  bool crcCheck;
};

void readTlm();
uint16_t getRpm(); 
void startDshot();
bool isTlmAvailable();
void resetTlmFlag();
void armingMotor();
void enable3d();
void disable3d();
void setControlLoopFrequency(uint16_t freq);
void enableTlm();
void disableTlm();
void setThrottle(uint16_t newThrottle);
void stopMotor();

void sendDshot();
void ISR_timer();
void ISR_DMA();
void setupDMA();
uint8_t updateCrc8(uint8_t crc, uint8_t crc_seed);
uint8_t calculateCrc8(const uint8_t *Buf, const uint8_t BufLen);

#endif
