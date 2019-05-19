# teensyshot
- DSHOT communication with ESC using a teensy 3.5 MCU
- DSHOT600 is generated using DMA
- Telemetry is received through UART at 115200bps
- Velocity PID control running at 500Hz is implemented for up to 6 motors
## API description
- DSHOT.ino: DSHOT600 communication.
## Authors
- [Arda Yiğit](mailto:arda.yigit@insa-strasbourg.fr): DMA programming and ESC communication
- [Jacques Gangloff:](mailto:jacques.gangloff@unistra.fr) higher level API, debugging 