#include <Arduino.h>
#include "dshot_api.h"

bool flagErr = false;

uint8_t bufferTlm[TLM_LENGTH];

void setup() {
  // configuring serial communications
  Serial.begin(115200); // communicating over usb
  Serial1.begin(115200); // communicating over UART1
  while(!Serial); // waits for the usb communication to be established
  delay(100);
  Serial.println("start"); // for debug

  startDshot(); // sets the DMA up and disables telemetry for initialization

  armingMotor(); // arms the motor by sending 0 command at least 10 times
  delay(500); // for debug - giving time to esc to say "ARMED"

  enable3d(); // enables the 3D flight mode
  setControlLoopFrequency(DSHOT_FREQ); // sets the control loop frequency for rpm control
  enableTlm(); // enables the telemetry
}

void loop() {
  if (Serial.available()) {
    String sentThrottle = "";
    char temp;
    while (Serial.available()) {
      temp = Serial.read();
      if (temp != '\n') {
        sentThrottle += temp;
      }
    }
    setThrottle(sentThrottle.toInt()); // updates the throttle value
  }

  noInterrupts();
  if (isTlmAvailable()) {
    resetTlmFlag();
    interrupts();
    //static uint8_t bufferTlm[TLM_LENGTH];
    while (Serial1.available() < TLM_LENGTH);
    // noInterrupts();
    for (int i = 0; i < TLM_LENGTH; i++) {
      bufferTlm[i] = Serial1.read();
    }
    // interrupts();

    /*for (int i = 0; i < TLM_LENGTH; i++) { // for debug
      Serial.print(bufferTlm[i]);
      Serial.print(" ");
    }
    Serial.println();*/

    // CRC verification
    if (bufferTlm[TLM_LENGTH-1] == calculateCrc8(bufferTlm, TLM_LENGTH-1)) {
      //Serial.println("ok");
    } else {
      //Serial.println("crc error");
      flagErr = true;
    }

  } else {
    interrupts();
  }
  if (flagErr) {
    //Serial.println("FALSE");
  }
}
