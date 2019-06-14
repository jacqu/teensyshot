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

#define ESCPID_PID_P              1.0               // PID proportional gain
#define ESCPID_PID_I              1.0               // PID integral gain
#define ESCPID_PID_D              0.0               // PID derivative gain
#define ESCPID_PID_F              0.0               // PID derivative filtering pole
#define ESCPID_PID_MIN            1.0               // PID min control input value
#define ESCPID_PID_MAX            999.0             // PID max control input value

#define ESCPID_COMM_MAGIC         0x43305735        // Magic number: "teensy35" in leet speech
#define ESCPID_COMM_WD_LEVEL      100               // Maximum number of periods without reference refresh

#define ESCPID_ERROR_MAGIC        -1                // Magic number error code

// Teensy->host communication data structure
// Data structure is robust to 4-bytes memory alignment
typedef struct {
  uint32_t      magic;                        // Magic number
  int           err[ESCPID_MAX_ESC];          // Last error number
  int           cmd[ESCPID_MAX_ESC];          // Current ESC command value
  float         tlm_deg[ESCPID_MAX_ESC];      // ESC temperature (°C)
  float         tlm_volt[ESCPID_MAX_ESC];     // Voltage of the ESC power supply (V)
  float         tlm_amp[ESCPID_MAX_ESC];      // ESC current (A)
  float         tlm_mah[ESCPID_MAX_ESC];      // ESC consumption (mAh)
  float         tlm_rpm[ESCPID_MAX_ESC];      // Motor rpm (rpm)
} ESCPIDcomm_struct_t;

// Host->teensy communication data structure
typedef struct {
  uint32_t      magic;                        // Magic number
  float         RPM_r[ESCPID_MAX_ESC];        // Velocity reference (rpm)
  float         PID_P[ESCPID_MAX_ESC];        // PID proportional gain
  float         PID_I[ESCPID_MAX_ESC];        // PID integral gain
  float         PID_D[ESCPID_MAX_ESC];        // PID derivative gain
  float         PID_f[ESCPID_MAX_ESC];        // PID filtering pole
} Hostcomm_struct_t;

#endif
