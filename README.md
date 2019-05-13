# teensyshot
============
* DSHOT communication with ESC using a teensy MCU
* DSHOT is generated using DMA
* Telemetry is received through UART
* Velocity PID control running at 500Hz is implemented for up to 6 motors

## API description
==================
*
* Implemented Dshot commands:
..* Arming
..* 3d mode enable/disable
..* Setting throttle value
..* Stopping the motor
