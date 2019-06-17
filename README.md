# teensyshot
- DSHOT communication with ESC using a teensy 3.5 MCU
- DSHOT600 is generated using DMA
- Telemetry is received through UART at 115200bps
- Velocity PID control running at 500Hz is implemented for up to 6 motors
## File description
- ESCPID: code running on the teensy
	- DSHOT.cpp: DSHOT600 communication.
	- AWPID.cpp: Anti-Windup PID.
	- ESCCMD.cpp: Bidirectional communication between teensy and ESC.
	- ESCPID.ino: main program.
- host: code running on a Linux platform connected to the teensy
## API description
### Incoming data
int16_t:
- -1000 to 999: throttle
- 1000+n: n is the command number
### Out-coming data
Data structure containing telemetry mean voltage and error code. 
## Authors
- [Arda Yiğit](mailto:arda.yigit@unistra.fr): DMA programming and ESC communication
- [Jacques Gangloff:](mailto:jacques.gangloff@unistra.fr) higher level API, debugging
