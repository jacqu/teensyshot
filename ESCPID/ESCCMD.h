/*
 *  Global definitions for ESCCMD
 *  NB: Arduino IDE automatically adds prototypes of functions
 *      found in all .ino files. 
 *
 */

#ifndef __ESCCMD_H
#define __ESCCMD_H

// Defines
#define ESCCMD_MAX_ESC          ESCPID_NB_ESC     // Max number of ESCs
#define ESCCMD_NB_UART          6                 // Number of UARTS available
#if ESCCMD_NB_UART < ESCCMD_MAX_ESC
  #error ESCCMD_NB_UART should be >= ESCCMD_MAX_ESC
#endif

#define ESCCMD_STATE_ARMED      1                 // Mask for the arming flag
#define ESCCMD_STATE_3D         2                 // Mask for the default/3D mode
#define ESCCMD_STATE_START      4                 // Mask for the motor start/stop bit
#define ESCCMD_STATE_ERROR      128               // Mask for the error flag

#define ESCCMD_CMD_REPETITION   10                // Number of time commands have to be repeated to be acknowledged by ESC
#define ESCCMD_CMD_DELAY        10                // Delay between two consecutive DSHOT transmissions (us)
#define ESCCMD_CMD_SAVE_DELAY   35000             // Minimum time to wait after a save command (us) 

#define ESCCMD_TIMER_PERIOD     2000              // Periodic loop period (us)
#define ESCCMD_ESC_WATCHDOG     250000            // ESC arming watchdog timer (us)
#define ESCCMD_TIMER_MAX_MISS   ( ESCCMD_ESC_WATCHDOG / ESCCMD_TIMER_PERIOD )
                                                  // Maximum missed tics before watchdog is triggered
#define ESCCMD_TLM_UART_SPEED   115200            // Baudrate of the telemetry serial transmission
#define ESCCMD_TLM_LENGTH       10                // Number of bytes in the telemetry packet

#define ESCCMD_MAX_THROTTLE     1999              // Max default throttle value
#define ESCCMD_MAX_3D_THROTTLE  999               // Max 3D throttle value
#define ESCCMD_MIN_3D_THROTTLE  -999              // Min 3D throttle value

#define ESCCMD_ERROR_DSHOT      -1                // DSHOT error
#define ESCCMD_ERROR_SEQ        -2                // Invalid function call sequence error
#define ESCCMD_ERROR_INIT       -3                // Call of non initialized function
#define ESCCMD_ERROR_PARAM      -4                // Invalid parameter error
#define ESCCMD_ERROR_CRC        -5                // CRC error
#define ESCCMD_ERROR_TLM_INVAL  -6                // Invalid telemetry error

#define ESCCMD_TIC_OCCURED      1                 // A new timer tic has occured

#endif