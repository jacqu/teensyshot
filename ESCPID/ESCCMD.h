/*
 *  Global definitions for ESCCMD
 *  NB: Arduino IDE automatically adds prototypes of functions
 *      found in all .ino files.
 *
 */

#ifndef __ESCCMD_H
#define __ESCCMD_H

// Defines
#define ESCCMD_MAX_ESC          DSHOT_MAX_OUTPUTS // Max number of ESCs
#define ESCCMD_NB_UART          6                 // Number of UARTS available
#if ESCCMD_NB_UART < ESCCMD_MAX_ESC
  #error ESCCMD_NB_UART should be >= ESCCMD_MAX_ESC
#endif

#define ESCCMD_STATE_ARMED      1                 // Mask for the arming flag
#define ESCCMD_STATE_3D         2                 // Mask for the default/3D mode
#define ESCCMD_STATE_START      4                 // Mask for the motor start/stop bit

#define ESCCMD_CMD_REPETITION   10                // Number of time commands have to be repeated to be acknowledged by ESC
#define ESCCMD_CMD_ARMING_REP   25                // Number of command repetition to arm
#define ESCCMD_CMD_DELAY        50                // Delay between two consecutive DSHOT transmissions (us)
#define ESCCMD_CMD_SAVE_DELAY   250000            // Minimum time to wait after a save command (us)

#define ESCCMD_TIMER_PERIOD     2000              // Periodic loop period (us)
#define ESCCMD_ESC_WATCHDOG     250000            // ESC arming watchdog timer (us)
#define ESCCMD_TIMER_MAX_MISS   ( ESCCMD_ESC_WATCHDOG / ESCCMD_TIMER_PERIOD )
                                                  // Maximum missed tics before watchdog is triggered
#define ESCCMD_THWD_LEVEL       20                // Maximum number of periods without throttle refresh

#define ESCCMD_TLM_UART_SPEED   115200            // Baudrate of the telemetry serial transmission
#define ESCCMD_TLM_BYTE_TIME    87                // Duration of one byte transmission (us)
#define ESCCMD_TLM_LENGTH       10                // Number of bytes in the telemetry packet
#define ESCCMD_TLM_MAX_PEND     1                 // Maximum number of telemetry packet pending
#define ESCCMD_TLM_NB_POLES     14                // Number of motor poles
#define ESCCMD_TLM_MAX_TEMP     100               // Maximum ESC temperature (°C)
#define ESCCMD_TLM_MAX_CRC_ERR  5                 // Maximum CRC errors (per ESCCMD_TLM_PER iterations)
#define ESCCMD_TLM_MAX_PKT_LOSS 50                // Maximum packet loss (per ESCCMD_TLM_PER iterations)
#define ESCCMD_TLM_PER          1000              // If 100, error thresholds are in percent     

#define ESCCMD_MAX_THROTTLE     1999              // Max default throttle value
#define ESCCMD_MAX_3D_THROTTLE  999               // Max 3D throttle value
#define ESCCMD_MIN_3D_THROTTLE  -999              // Min 3D throttle value

#define ESCCMD_BEEP_DURATION    50                // Duration of a beep (ms)
#define ESCCMD_ERROR_DSHOT      -1                // DSHOT error
#define ESCCMD_ERROR_SEQ        -2                // Invalid function call sequence error
#define ESCCMD_ERROR_INIT       -3                // Call of non initialized function
#define ESCCMD_ERROR_PARAM      -4                // Invalid parameter error
#define ESCCMD_ERROR_TLM_CRC    -5                // CRC error in telemetry packet
#define ESCCMD_ERROR_TLM_INVAL  -6                // Invalid telemetry error
#define ESCCMD_ERROR_TLM_PEND   -7                // Maximum number of pending telemetry packet reached
#define ESCCMD_ERROR_TLM_TEMP   -8                // Maximum ESC temperature reached
#define ESCCMD_ERROR_TLM_CRCMAX -9                // Maximum allowed CRC errors reached
#define ESCCMD_ERROR_TLM_LOST   -10               // Lost packet(s) detected

#define ESCCMD_TIC_OCCURED      1                 // A new timer tic has occured

// enums: borrowed from betaflight pwm_output.h
typedef enum {
  DSHOT_CMD_MOTOR_STOP = 0,
  DSHOT_CMD_BEACON1,
  DSHOT_CMD_BEACON2,
  DSHOT_CMD_BEACON3,
  DSHOT_CMD_BEACON4,
  DSHOT_CMD_BEACON5,
  DSHOT_CMD_ESC_INFO,                       // V2 includes settings
  DSHOT_CMD_SPIN_DIRECTION_1,
  DSHOT_CMD_SPIN_DIRECTION_2,
  DSHOT_CMD_3D_MODE_OFF,                    // 9
  DSHOT_CMD_3D_MODE_ON,                     // 10
  DSHOT_CMD_SETTINGS_REQUEST,               // Currently not implemented
  DSHOT_CMD_SAVE_SETTINGS,                  // 12
  DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
  DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
  DSHOT_CMD_LED0_ON,                        // BLHeli32 only
  DSHOT_CMD_LED1_ON,                        // BLHeli32 only
  DSHOT_CMD_LED2_ON,                        // BLHeli32 only
  DSHOT_CMD_LED3_ON,                        // BLHeli32 only
  DSHOT_CMD_LED0_OFF,                       // BLHeli32 only
  DSHOT_CMD_LED1_OFF,                       // BLHeli32 only
  DSHOT_CMD_LED2_OFF,                       // BLHeli32 only
  DSHOT_CMD_LED3_OFF,                       // BLHeli32 only
  DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30,  // KISS audio Stream mode on/Off
  DSHOT_CMD_SILENT_MODE_ON_OFF = 31,        // KISS silent Mode on/Off
  DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE = 32,
  DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY = 33,
  DSHOT_CMD_MAX = 47
} ESCCMD_codes;

// Function prototypes
void    ESCCMD_init( uint8_t );
int     ESCCMD_arm_all( void );
int     ESCCMD_3D_on( void );
int     ESCCMD_3D_off( void );
int     ESCCMD_start_timer( void );
int     ESCCMD_stop_timer( void );
int     ESCCMD_throttle( uint8_t, int16_t );
int     ESCCMD_stop( uint8_t );
int     ESCCMD_read_err( uint8_t, int8_t* );
int     ESCCMD_read_cmd( uint8_t, uint16_t* );
int     ESCCMD_read_tlm_status( uint8_t );
int     ESCCMD_read_deg( uint8_t, uint8_t* );
int     ESCCMD_read_volt( uint8_t, uint16_t* );
int     ESCCMD_read_amp( uint8_t, uint16_t* );
int     ESCCMD_read_mah( uint8_t, uint16_t* );
int     ESCCMD_read_rpm( uint8_t, int16_t* );
uint8_t *ESCCMD_read_packet( uint8_t );
int     ESCCMD_extract_packet_data( uint8_t );
int     ESCCMD_tic( void );
uint8_t ESCCMD_update_crc8( uint8_t, uint8_t );
uint8_t ESCCMD_crc8( uint8_t*, uint8_t );
void    ESCCMD_ISR_timer( void );

#endif
