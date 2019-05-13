
// arms the motor if it is not already armed
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

// enables the 3D flight mode
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

// disables the 3D flight mode
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

// sets the global timer's frequency in Hz (default value: 500Hz)
void setControlLoopFrequency(uint16_t freq) {
  timer.update(1000000 / freq); 
}

// asks for telemetry response after each dshot frame
void enableTlm() {
  if (!requestTlm) {
    noInterrupts(); 
    requestTlm = true; 
    interrupts(); 
  }
}

// stops asking for telemetry response after each dshot frame
void disableTlm() {
  if (requestTlm) {
    noInterrupts(); 
    requestTlm = false; 
    interrupts(); 
  }
}

// sets the new throttle value
void setThrottle(uint16_t newThrottle) {
  // maybe add saturation
  if (armed) {
    noInterrupts(); 
    throttle = newThrottle; 
    requestTlm = true; 
    interrupts(); 
  }
}

// stops the motor
void stopMotor() {
  throttle = 0; 
  mode = STOP_MODE; 
}
