/*
 *  Communication with ESCPID code running on teensy 3.5
 *   JG, June 2019
 *   To compile : gcc -Wall -o host
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <strings.h>
#include <sys/ioctl.h>
#if defined(__linux__)
#include <linux/serial.h>
#endif
#include "host.h"

// Flags
#define HOST_STANDALONE                         // main is added

// Defines
// Note on USB port <-> devices relationship on RPI 3b+:
// Bottom, away from RJ45 : platform-3f980000.usb-usb-0:1.2:1.0
// Bottom, next RJ45      : platform-3f980000.usb-usb-0:1.1.3:1.0
// Top, away from RJ45    : platform-3f980000.usb-usb-0:1.3:1.0
// Top, next RJ45         : platform-3f980000.usb-usb-0:1.1.2:1.0
//#define HOST_MODEMDEVICE    "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.2:1.0"
//#define HOST_MODEMDEVICE    "/dev/ttyACM0"
#define HOST_MODEMDEVICE    "/dev/tty.usbmodem43677001"
#define HOST_BAUDRATE       B115200             // Serial baudrate
#define HOST_READ_TIMEOUT   5                   // Tenth of second
#define HOST_NB_PING        100                 // Nb roundtrip communication

// Globals
int                 Host_fd = -1;               // Serial port file descriptor
struct termios      Host_oldtio;                // Backup of old configuration

ESCPIDcomm_struct_t ESCPID_comm = {
                                  ESCPID_COMM_MAGIC,
                                  {},
                                  {},
                                  {},
                                  {},
                                  {}
                                  };
Hostcomm_struct_t   Host_comm =   {
                                  ESCPID_COMM_MAGIC,
                                  {},
                                  {},
                                  {},
                                  {},
                                  {}
                                  };

//
//  Initialize serial port
//
int Host_init_port( char *portname )  {
  struct termios        newtio;
fprintf( stderr, "test1\n" );
  // Open device
  //Host_fd = open( portname, O_RDWR | O_NOCTTY );
  Host_fd = open( portname, O_RDWR | O_NOCTTY | O_NONBLOCK );
  if ( Host_fd < 0 )  {
    perror( portname );
    return -1;
  }
fprintf( stderr, "test2\n" );
  /* Save current port settings */
  tcgetattr( Host_fd, &  Host_oldtio );
fprintf( stderr, "test3\n" );
  /* Define new settings */
  bzero( &newtio, sizeof(newtio) );
  cfmakeraw( &newtio );
  newtio.c_cflag = HOST_BAUDRATE | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;

  /* Set input mode (non-canonical, no echo,...) */
  newtio.c_lflag = 0;

  /* Inter-character timer  */
  newtio.c_cc[VTIME] = HOST_READ_TIMEOUT;
  newtio.c_cc[VMIN] = 0;
fprintf( stderr, "test4\n" );
  /* Apply the settings */
  tcflush( Host_fd, TCIFLUSH );
  tcsetattr( Host_fd, TCSANOW, &newtio );
fprintf( stderr, "test5\n" );
  return 0;
}

//
//  Release serial port
//
void Host_release_port( void )  {

  /* Restore initial settings */
  tcsetattr( Host_fd, TCSANOW, &  Host_oldtio );
  close( Host_fd );
  Host_fd = -1;
}

//
//  main
//
int main( int argc, char *argv[] )  {

  int                 i, ret, res = 0;
  uint8_t             *pt_in = (uint8_t*)(&ESCPID_comm);
  struct timespec     start, cur;
  unsigned long long  elapsed_us;

  // Initialize serial port
  if ( Host_init_port( HOST_MODEMDEVICE ) )  {
    fprintf( stderr, "Error initializing serial port.\n" );
    exit( -1 );
  }

  // Testing roundtrip serial link duration
  for ( i = 0; i < HOST_NB_PING; i++ )  {

    // Send output structure
    res = write(   Host_fd, &Host_comm, sizeof( Host_comm ) );
    if ( res < 0 )  {
      perror( "write Host_comm" );
      exit( -2 );
    }
    fsync( Host_fd );

    // Wait for response

    // Get current time
    clock_gettime( CLOCK_MONOTONIC, &start );

    // Reset byte counter and magic number
    res = 0;
    ESCPID_comm.magic = 0;

    do  {
      ret = read( Host_fd, &pt_in[res], 1 );

      // Data received
      if ( ret > 0 )  {
        res += ret;
      }

      // Read error
      if ( ret < 0 )
        break;

      // Compute time elapsed
      clock_gettime( CLOCK_MONOTONIC, &cur );
      elapsed_us =  ( cur.tv_sec * 1e6 + cur.tv_nsec / 1e3 ) -
                    ( start.tv_sec * 1e6 + start.tv_nsec / 1e3 );

      // Timeout
      if ( elapsed_us / 100000 > HOST_READ_TIMEOUT )
        break;

    } while ( res < sizeof( ESCPID_comm ) );

    // Check response size
    if ( res != sizeof( ESCPID_comm ) )  {
      fprintf( stderr, "Packet with bad size received.\n" );

      // Flush input buffer
      while ( ( ret = read( Host_fd, pt_in, 1 ) ) )
        if ( ret < 0 )
          break;
    }

    // Check magic number
    if ( ESCPID_comm.magic !=  ESCPID_COMM_MAGIC )
      fprintf( stderr, "Invalid magic number.\n" );

    // Print rountrip duration
    fprintf( stderr, "Delay: %llu us\n", elapsed_us );
  }

  // Restoring serial port initial configuration
  Host_release_port( );

  return 0;
}
