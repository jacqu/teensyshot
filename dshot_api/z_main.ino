unsigned long oldT;
bool flagErr = false; 

void setup() {
  pinMode(10,OUTPUT); // debug
  digitalWrite(10, LOW); // debug
  
  Serial.begin(115200);
  Serial1.begin(115200);
  while(!Serial); // waits for the usb communication to be established
  
  delay(500);
  Serial.println("start"); 
  setupDMA();
  delay(500);

  disableTlm();
  armingMotor();
  delay(500);
  enable3d();

  setControlLoopFrequency(DSHOT_FREQ);

  delay(500);
  enableTlm();
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
    setThrottle(sentThrottle.toInt()); 
  }
  
  noInterrupts();
  if (flagTlm) {
    flagTlm = false;
    interrupts();
    static uint8_t bufferTlm[TLM_LENGTH];
    digitalWrite(10, HIGH); 
    while (Serial1.available() < TLM_LENGTH); 
    digitalWrite(10, LOW); 
    noInterrupts();
    for (int i = 0; i < TLM_LENGTH; i++) {
      bufferTlm[i] = Serial1.read(); 
    }
    interrupts();
    for (int i = 0; i < TLM_LENGTH; i++) {
      Serial.print(bufferTlm[i]); 
      Serial.print(i); 
      Serial.print(" ");
    }
    Serial.println(); 
    if (bufferTlm[TLM_LENGTH-1] == calculateCrc8(bufferTlm, TLM_LENGTH-1)) {
      Serial.println("ok");
    } else {
      Serial.println("csc error");
      flagErr = true; 
    }
  } else {
    interrupts();
  }
  if (flagErr) {
    Serial.println("FALSE"); 
  }
}
