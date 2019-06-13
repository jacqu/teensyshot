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

// Teensy->host communication data structure
// Data structure is robust to 4-bytes memory alignment
typedef struct {
  int           ESCPID_err[ESCPID_MAX_ESC];            // Last error number
  float         ESCCMD_tlm_deg[ESCPID_MAX_ESC];        // ESC temperature (°C)
  float         ESCCMD_tlm_volt[ESCPID_MAX_ESC];       // Voltage of the ESC power supply (V)
  float         ESCCMD_tlm_amp[ESCPID_MAX_ESC];        // ESC current (A)
  float         ESCCMD_tlm_mah[ESCPID_MAX_ESC];        // ESC consumption (Ah)
  float         ESCCMD_tlm_rpm[ESCPID_MAX_ESC];        // Motor rpm (rpm)
} ESCPIDcomm_struct_t;

// Host->teensy communication data structure
typedef struct {
  float         Velocity_ref[ESCPID_MAX_ESC];         // Velocity reference (rpm)
} Hostcomm_struct_t;

#endif
