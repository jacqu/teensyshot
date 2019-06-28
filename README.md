# teensyshot
- DSHOT communication with ESC using a teensy 3.5 MCU
- DSHOT600 is generated using DMA
- Telemetry is received through UART at 115200bps
- Velocity PID control running at 500Hz is implemented for up to 6 motors
## File Description
- ESCPID: code running on the teensy
	- DSHOT.cpp: DSHOT600 communication.
	- AWPID.cpp: Anti-Windup PID.
	- ESCCMD.cpp: Bidirectional communication between teensy and ESC.
	- ESCPID.ino: main program.
- host: code running on a Linux platform connected to the teensy
## API Description
### Incoming data
Velocity reference and PID parameters.
### Out-coming Data
Data structure containing telemetry data, error code and last DSHOT command.
### Tunable Macros
- ESCCMD_ESC_EMULATION (ESCCMD.cpp): flag to enable or disable ESC emulation. When enabled, the telemetry data is emulated. Packet loss can also be emulated. This helps to debug the code without ESC.
- ESCPID_NB_ESC (ESCPID.h): define the number of ESCs that are handled by teensyshot.
- ESCPID_PID_MAX (ESCPID.h): Maximum PID control value. The maximum allowed is 999. A lower value may limit the motor maximum RPM in case of controller instability. Start testing with a low value and when everything is ok, raise it to the max. 
## Wiring
Teensyshot can communicate with up to 6 ESCs. The wiring should respect the order.

|   | DSHOT | TLM/RX |
|:-:|:-----:|:------:|
| 1 |   22  |    0   |
| 2 |   23  |    9   |
| 3 |   6   |    7   |
| 4 |   20  |   31   |
| 5 |   21  |   34   |
| 6 |   5   |   47   |
## Authors
- [Arda Yiğit](mailto:arda.yigit@unistra.fr): DMA programming and ESC communication
- [Jacques Gangloff:](mailto:jacques.gangloff@unistra.fr) higher level API, debugging
