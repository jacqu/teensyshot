/*
 *  Definitions for ESCPID.ino
 *
 */

#ifndef __ESCPID_H
#define __ESCPID_H

// Defines
#define ESCPID_NB_ESC             1                 // Number of ESCs

#define ESCPID_USB_UART_SPEED     115200            // Baudrate of the teeensy USB serial link
#define ESCPID_ERROR_MSG_LENGTH   80                // Max string length of an error message

#define ESCPID_PID_P              1.0               // PID proportional gain
#define ESCPID_PID_I              1.0               // PID integral gain
#define ESCPID_PID_D              0.0               // PID derivative gain
#define ESCPID_PID_f              0.0               // PID derivative filtering pole
#define ESCPID_PID_MIN            1.0               // PID min control input value
#define ESCPID_PID_MAX            999.0             // PID max control input value

#endif
