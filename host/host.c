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
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <strings.h>
#include <sys/ioctl.h> 
#include <linux/serial.h>
#include "host.h"

// Flags
#define HOST_STANDALONE                         // main is added

// Defines
// Note on USB port <-> devices relationship on RPI 3b+:
// Bottom, away from RJ45 : platform-3f980000.usb-usb-0:1.2:1.0
// Bottom, next RJ45      : platform-3f980000.usb-usb-0:1.1.3:1.0
// Top, away from RJ45    : platform-3f980000.usb-usb-0:1.3:1.0
// Top, next RJ45         : platform-3f980000.usb-usb-0:1.1.2:1.0
#define HOST_MODEMDEVICE    "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.2:1.0"
#define HOST_BAUDRATE       B115200             // Serial baudrate
#define HOST_READ_TIMEOUT   5                   // Tenth of second
#define HOST_NB_PING        100                 // Nb roundtrip communication

// Globals
int                 host_fd = -1;                 // Serial port file descriptor
struct termios      host_oldtio;                  // Backup of old configuration

//
//  Initialize serial port
//
int host_init_port( char *portname )  {
  struct termios        newtio;
  struct serial_struct  serial;

  // Open device
  host_fd = open( portname, O_RDWR | O_NOCTTY ); 
  if ( host_fd < 0 )  { 
    perror( portname ); 
    return -1; 
  }
  
  /* Set the low_latency flag */
  if ( ioctl( host_fd, TIOCGSERIAL, &serial ) == -1 )  {
    perror( "TIOCGSERIAL" );
  } 
  serial.flags |= ASYNC_LOW_LATENCY;
  if ( ioctl( host_fd, TIOCSSERIAL, &serial ) == -1 )  {
    perror( "TIOCSSERIAL" );
  }

  /* Save current port settings */
  tcgetattr( host_fd, &  host_oldtio ); 

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

  /* Apply the settings */
  tcflush( host_fd, TCIFLUSH );
  tcsetattr( host_fd, TCSANOW, &newtio );

  return 0;
}

//
//  Release serial port
//
void host_release_port( void )  {

  /* Restore initial settings */
  tcsetattr( host_fd, TCSANOW, &  host_oldtio );
  close( host_fd );
  host_fd = -1;
}

//
//  main
//
int main( int argc, char *argv[] )  {
    
  int                 i, ret, res = 0;
  unsigned char       snd = HOST_PING_CHAR;
  unsigned char       rcv = 0;
  struct timespec     start, cur;
  unsigned long long  elapsed_us;
  
  // Initialize serial port
  if ( host_init_port( HOST_MODEMDEVICE ) )  {
    fprintf( stderr, "Error initializing serial port.\n" );
    exit( -1 );
  }
  
  // Testing roundtrip serial link duration
  for ( i = 0; i < HOST_NB_PING; i++ )  {
    
    // Send ping char
    res = write(   host_fd, &snd, 1 );
    if ( res < 0 )  { 
      perror( "write ping char" ); 
      exit( -2 ); 
    }
    fsync( host_fd );
    
    // Wait for response
    
    res = 0;
    // Get current time
    clock_gettime( CLOCK_MONOTONIC, &start );
    
    do  {
      ret = read(   host_fd, &rcv, 1 );
      if ( ret > 0 )  {
        res += ret;
      }
      
      // Read error
      if ( ret < 0 )
        break;
        
      // Compute time elapsed
      clock_gettime( CLOCK_MONOTONIC, &cur );
      elapsed_us = ( cur.tv_sec * 1e6 + cur.tv_nsec / 1e3 ) - ( start.tv_sec * 1e6 + start.tv_nsec / 1e3 );
      
      // Timeout
      if ( elapsed_us / 100000 > HOST_READ_TIMEOUT )
        break;
      
    } while ( res < 1 );
    
    // Check response
    if ( res != 1 )
      fprintf( stderr, "No response.\n" );
    if ( rcv !=  HOST_PING_CHAR )
      fprintf( stderr, "Invalid response.\n" );
    fprintf( stderr, "Delay: %llu us\n", elapsed_us );
  }
  
  // Restoring serial port initial configuration
  host_release_port( );
  
  return 0;
}
