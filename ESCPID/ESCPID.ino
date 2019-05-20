/*
 *  ESCPID:   PID control of up to 6 ESCs using teensy 3.5 MCU
 *
 *  Note:     Best viewed using Arduino IDE with tab space = 2
 *
 *  Authors:  Arda Yiğit and Jacques Gangloff
 *  Date:     May 2019
 */

#include <Arduino.h>

/* FOR DEBUG */
uint16_t armingCmd[6] = {0};
uint8_t tlm[6] = {0};
int temp;
/* FOR DEBUG */


//
//  Arduino setup function
//
void setup() {
  Serial.begin(115200);
  while(!Serial);
  delay(10);
  Serial.println("start");
  DSHOT_init();
}

//
//  Arduino main loop
//
void loop() {
  temp = DSHOT_send(armingCmd, tlm);
  delay(1);
  if (temp)
    Serial.println(temp);
  delay(1);
}
