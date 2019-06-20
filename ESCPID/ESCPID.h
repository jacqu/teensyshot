/*
 *  Definitions for ESCPID.ino
 *
 */

#ifndef __ESCPID_H
#define __ESCPID_H

// Defines
#define ESCPID_NB_ESC             1                 // Number of ESCs
#define ESCPID_MAX_ESC            6                 // Max number of ESCs

#define ESCPID_USB_UART_SPEED     115200            // Baudrate of the teeensy USB serial link
#define ESCPID_ERROR_MSG_LENGTH   80                // Max string length of an error message

#define ESCPID_PID_P              400               // Default PID proportional gain
#define ESCPID_PID_I              1                 // Default PID integral gain
#define ESCPID_PID_D              0                 // Default PID derivative gain
#define ESCPID_PID_F              0                 // Default PID derivative filtering pole
#define ESCPID_PID_MIN            1                 // Default PID min control input value
#define ESCPID_PID_MAX            999               // Default PID max control input value
#define ESCPID_PID_ADAPT_GAIN     0.01              // Range adaptation gain for PID coefficient

#define ESCPID_COMM_MAGIC         0x43305735        // Magic number: "teensy35" in leet speech
#define ESCPID_COMM_WD_LEVEL      100               // Maximum number of periods without reference refresh

#define ESCPID_ERROR_MAGIC        -1                // Magic number error code

// Teensy->host communication data structure
// sizeof(ESCPID_comm)=64 to match USB 1.0 buffer size
typedef struct {
  uint32_t      magic;                        // Magic number
  int8_t        err[ESCPID_MAX_ESC];          // Last error number
  uint8_t       deg[ESCPID_MAX_ESC];          // ESC temperature (°C)
  uint16_t      cmd[ESCPID_MAX_ESC];          // Current ESC command value
  uint16_t      volt[ESCPID_MAX_ESC];         // Voltage of the ESC power supply (0.01V)
  uint16_t      amp[ESCPID_MAX_ESC];          // ESC current (0.01A)
  int16_t       rpm[ESCPID_MAX_ESC];          // Motor rpm (10 rpm)
} ESCPIDcomm_struct_t;

// Host->teensy communication data structure
// sizeof(Host_comm)=64 to match USB 1.0 buffer size
typedef struct {
  uint32_t      magic;                        // Magic number
  int16_t       RPM_r[ESCPID_MAX_ESC];        // Velocity reference (10 rpm)
  uint16_t      PID_P[ESCPID_MAX_ESC];        // PID proportional gain
  uint16_t      PID_I[ESCPID_MAX_ESC];        // PID integral gain
  uint16_t      PID_D[ESCPID_MAX_ESC];        // PID derivative gain
  uint16_t      PID_f[ESCPID_MAX_ESC];        // PID filtering pole
} Hostcomm_struct_t;

#endif
