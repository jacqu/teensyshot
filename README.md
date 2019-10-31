# teensySHOT
- DSHOT communication with ESC using Teensy 3.5 or Teensy 4.0 development boards
- DSHOT600 is generated using DMA
- Telemetry is received through UART at 115200bps
- Velocity PID control running at 500Hz is implemented for up to 6 motors

## File Description
- ESCPID: code running on the Teensy
	- DSHOT.cpp: DSHOT600 communication.
	- AWPID.cpp: Anti-Windup PID.
	- ESCCMD.cpp: Bidirectional communication between Teensy and ESC.
	- ESCPID.ino: main program.
- host: code running on a Linux platform connected to the teensy. 
- docs: useful documentation (MCU datasheets, DSHOT command description, telemetry protocol explanation, PID description...). 

## API Description

### Incoming data
Velocity reference and PID parameters.
### Out-coming Data
Data structure containing telemetry data, error code and last DSHOT command.
### Tunable Macros
- ESCCMD_ESC_EMULATION (ESCCMD.cpp): flag to enable or disable ESC emulation. When enabled, the telemetry data is emulated. Packet loss can also be emulated. This helps to debug the code without ESC.
- ESCPID_NB_ESC (ESCPID.h): define the number of ESCs that are handled by teensySHOT.
- ESCPID_PID_MAX (ESCPID.h): Maximum PID control value. The maximum allowed is 999. A lower value may limit the motor maximum RPM in case of controller instability. Start testing with a low value and when everything is ok, raise it to the max. 

## Wiring
teensySHOT can communicate with up to 6 ESCs. The following pins are to be used with the current version. It is possible to change some DSHOT pins using muxing options of the MCU (tutorial in progress). 
<table>
  <tr> <td>   </td> <td colspan="2">  Teensy 3.5 </td> <td colspan="2">  Teensy 4.0 </td> </tr>
  <tr> <td>   </td> <td> DSHOT </td> <td> TLM/RX </td> <td> DSHOT </td> <td> TLM/RX </td> </tr>
  <tr> <td> 1 </td> <td>   22  </td> <td>    0   </td> <td>   4   </td> <td>    0   </td> </tr>
  <tr> <td> 2 </td> <td>   23  </td> <td>    9   </td> <td>   8   </td> <td>    7   </td> </tr>
  <tr> <td> 3 </td> <td>    6  </td> <td>    7   </td> <td>  24   </td> <td>   15   </td> </tr>
  <tr> <td> 4 </td> <td>   20  </td> <td>   31   </td> <td>  22   </td> <td>   16   </td> </tr>
  <tr> <td> 5 </td> <td>   21  </td> <td>   34   </td> <td>  23   </td> <td>   21   </td> </tr>
  <tr> <td> 6 </td> <td>    5  </td> <td>   47   </td> <td>   9   </td> <td>   25   </td> </tr>
</table>

## Authors
- [Arda Yiğit](mailto:arda.yigit@unistra.fr): DMA programming and ESC communication
- [Jacques Gangloff](mailto:jacques.gangloff@unistra.fr): higher level API, debugging
