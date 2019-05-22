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
  pinMode(5,OUTPUT);
  Serial.begin(115200);
  while(!Serial);
  delay(10);
  ESCCMD_init();
  temp = ESCCMD_arm(); // TODO: check the return
  if (temp)
    Serial.println(temp);
  temp = ESCCMD_3D_on();
  if (temp)
    Serial.println(temp);
  temp = ESCCMD_start();
  if (temp)
    Serial.println(temp);
}

//
//  Arduino main loop
//
void loop() {
  temp = getError();
  if (temp) {
    Serial.println(temp);
  }
  delay(1);
}
