/*
 *  Communication with ESCPID code running on teensy 3.5
 *   JG, June 2019
 *   To compile : gcc -Wall -o host host.c
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
#include <sys/param.h>
#if defined(__linux__)
#include <linux/serial.h>
#include <linux/input.h>
#endif
#include "host.h"

// Flags
#define HOST_STANDALONE                         // main is added

#define HOST_MAX_DEVICES    5                   // Max number of serial devices

// Defines
// Note on USB port <-> devices relationship on RPI 3b+:
// Bottom, away from RJ45 : platform-3f980000.usb-usb-0:1.2:1.0
// Bottom, next RJ45      : platform-3f980000.usb-usb-0:1.1.3:1.0
// Top, away from RJ45    : platform-3f980000.usb-usb-0:1.3:1.0
// Top, next RJ45         : platform-3f980000.usb-usb-0:1.1.2:1.0
//#define HOST_MODEMDEVICE    "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.2:1.0"
#define HOST_MODEMDEVICE    "/dev/ttyACM0"
//#define HOST_MODEMDEVICE    "/dev/tty.usbmodem43677001"
#define HOST_BAUDRATE       B115200             // Serial baudrate
#define HOST_READ_TIMEOUT   5                   // Tenth of second
#define HOST_NB_PING        100                 // Nb roundtrip communication

// Globals
int             Host_fd[HOST_MAX_DEVICES] = 
                { -1, -1, -1, -1, -1 };         // Serial port file descriptor
struct termios  Host_oldtio[HOST_MAX_DEVICES];  // Backup of initial tty configuration

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
//  Get the file descriptor index of the device name
//  Returns -1 if no matching fd is found
//
int Host_get_fd( char *portname ) {
  int   i;
  char  devname[MAXPATHLEN];
  #if defined(__linux__)
  char  procname[MAXPATHLEN];
  #endif
  
  for ( i = 0; i < HOST_MAX_DEVICES; i++ )  {
    if ( Host_fd[i] != -1 ) {
      #if defined(__linux__)
      snprintf( procname, MAXPATHLEN, "/proc/self/fd/%d", Host_fd[i] );
      if ( readlink( procname, devname, MAXPATHLEN ) != -1 )
        if ( !strcmp( devname, portname ) )
          return i;
      #endif
      #if defined(__APPLE__)
      if ( fcntl( Host_fd[i], F_GETPATH, devname ) != -1 )
        if ( !strcmp( devname, portname ) )
          return i;
      #endif
    }
  }

  return -1;
}

//
//  Initialize serial port
//
int Host_init_port( char *portname )  {
  struct  termios newtio;
  int     check_fd;
  int     i;

  // Open device
  check_fd = open( portname, O_RDWR | O_NOCTTY | O_NONBLOCK );

  if ( check_fd < 0 )  {
    perror( portname );
    return -1;
  }
  
  // Look for an empty slot to store the fd
  for ( i = 0; i < HOST_MAX_DEVICES; i++ )
    if ( Host_fd[i] != - 1 )
      break;
      
  // Close fd and throw an error if all slots are used
  if ( i == HOST_MAX_DEVICES ) {
    close( check_fd );
    return -2;
  }
    
  Host_fd[i] = check_fd;

  /* Save current port settings */
  tcgetattr( check_fd, &  Host_oldtio );

  /* Define new settings */
  bzero( &newtio, sizeof(newtio) );
  cfmakeraw( &newtio );

  newtio.c_cflag =      HOST_BAUDRATE | CS8 | CLOCAL | CREAD;
  newtio.c_iflag =      IGNPAR;
  newtio.c_oflag =      0;
  newtio.c_lflag =      0;
  newtio.c_cc[VTIME] =  0;
  newtio.c_cc[VMIN] =   0;

  #if defined(__APPLE__)
  cfsetispeed( &newtio, HOST_BAUDRATE );
  cfsetospeed( &newtio, HOST_BAUDRATE );
  #endif

  /* Apply the settings */
  tcflush( check_fd, TCIFLUSH );
  tcsetattr( check_fd, TCSANOW, &newtio );

  return 0;
}

//
//  Release serial port
//
void Host_release_port( char *portname )  {
  int fd_idx;
  
  // Get fd index from name
  fd_idx = Host_get_fd( portname );
  
  if ( fd_idx != -1 ) {
    // Restore initial settings if needed
    tcsetattr( Host_fd[fd_idx], TCSANOW, &  Host_oldtio );
    close( Host_fd[fd_idx] );
    Host_fd[fd_idx] = -1;
  }
}

//
//  main
//
int main( int argc, char *argv[] )  {

  int                 i, ret, res = 0, fd_idx;
  uint8_t             *pt_in = (uint8_t*)(&ESCPID_comm);
  struct timespec     start, cur;
  unsigned long long  elapsed_us;

  // Initialize serial port
  if ( Host_init_port( HOST_MODEMDEVICE ) )  {
    fprintf( stderr, "Error initializing serial port.\n" );
    exit( -1 );
  }

  // Get fd index
  fd_idx = Host_get_fd( HOST_MODEMDEVICE );
  
  // Check if fd index is valid
  if ( fd_idx < 0 ) {
    fprintf( stderr, "Unable to get file descriptor index.\n" );
    exit( -2 );
  }
  // Testing roundtrip serial link duration
  for ( i = 0; i < HOST_NB_PING; i++ )  {

    // Send output structure
    res = write( Host_fd[fd_idx], &Host_comm, sizeof( Host_comm ) );
    if ( res < 0 )  {
      perror( "write Host_comm" );
      exit( -3 );
    }
    fsync( Host_fd[fd_idx] );

    // Wait for response

    // Get current time
    clock_gettime( CLOCK_MONOTONIC, &start );

    // Reset byte counter and magic number
    res = 0;
    ESCPID_comm.magic = 0;

    do  {
      ret = read( Host_fd[fd_idx], &pt_in[res], 1 );

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
      while ( ( ret = read( Host_fd[fd_idx], pt_in, 1 ) ) )
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
  Host_release_port( HOST_MODEMDEVICE );

  return 0;
}
