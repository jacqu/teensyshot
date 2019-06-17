/******************************************************************************
 *	PING SERIAL : test serial link responsiveness
 * 	JG, Apr 2019
 * 	To compile : gcc -Wall -o ping_serial ping_serial.c
 *****************************************************************************/

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

#define UART_BAUDRATE 			B115200					// Serial baudrate
#define UART_MODEMDEVICE 		"/dev/ttyACM0"			// Serial port
#define UART_READ_TIMEOUT		5						// Tenth of second

#define UART_NB_PING			1000000					// Nb of pings
#define UART_PING_CHAR			'U'						// Ping character

int								uart_fd = -1;			// Serial port file descriptor
struct termios 					uart_oldtio;			// Backup of old configuration

/******************************************************************************
 *	uart_init_port : initialize serial port
 *****************************************************************************/
int uart_init_port( char *portname )	{
	struct termios 			newtio;
	struct serial_struct 	serial;

	/* Open device */
	uart_fd = open( portname, O_RDWR | O_NOCTTY ); 
	if ( uart_fd < 0 )  { 
		perror( portname ); 
		return -1; 
	}
	
	/* Set the low_latency flag */
	if ( ioctl( uart_fd, TIOCGSERIAL, &serial ) == -1 )	{
		perror( "TIOCGSERIAL" );
	} 
	serial.flags |= ASYNC_LOW_LATENCY;
	if ( ioctl( uart_fd, TIOCSSERIAL, &serial ) == -1 )	{
		perror( "TIOCSSERIAL" );
	}

	/* Save current port settings */
	tcgetattr( uart_fd, &uart_oldtio ); 

	/* Define new settings */
	bzero( &newtio, sizeof(newtio) );
	cfmakeraw( &newtio );
	newtio.c_cflag = UART_BAUDRATE | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;

	/* Set input mode (non-canonical, no echo,...) */
	newtio.c_lflag = 0;

	/* Inter-character timer  */
	newtio.c_cc[VTIME] = UART_READ_TIMEOUT;   
	newtio.c_cc[VMIN] = 0;

	/* Apply the settings */
	tcflush( uart_fd, TCIFLUSH );
	tcsetattr( uart_fd, TCSANOW, &newtio );

	return 0;
}

/******************************************************************************
 *	uart_release_port : release serial port
 *****************************************************************************/
void uart_release_port( void )	{

	/* Restore initial settings */
	tcsetattr( uart_fd, TCSANOW, &uart_oldtio );
	close( uart_fd );
	uart_fd = -1;
}

int main( int argc, char *argv[] )	{
		
	int 				i, ret, res = 0;
	unsigned char		snd = UART_PING_CHAR;
	unsigned char		rcv = 0;
	struct timespec 	start, cur;
	unsigned long long	elapsed_us;
	
	// Initialize serial port
	if ( uart_init_port( UART_MODEMDEVICE ) )	{
		fprintf( stderr, "Error initializing serial port.\n" );
		exit( -1 );
	}
	
	// Testing roundtrip serial link duration
	for ( i = 0; i < UART_NB_PING; i++ )	{
		
		// Send ping char
		res = write( uart_fd, &snd, 1 );
		if ( res < 0 )  { 
			perror( "write ping char" ); 
			exit( -2 ); 
		}
		fsync( uart_fd );
		
		// Wait for response
		
		res = 0;
		// Get current time
		clock_gettime( CLOCK_MONOTONIC, &start );
		
		do	{
			ret = read( uart_fd, &rcv, 1 );
			if ( ret > 0 )	{
				res += ret;
			}
			
			/* Read error */
			if ( ret < 0 )
				break;
				
			/* Compute time elapsed */
			clock_gettime( CLOCK_MONOTONIC, &cur );
			elapsed_us = ( cur.tv_sec * 1e6 + cur.tv_nsec / 1e3 ) - ( start.tv_sec * 1e6 + start.tv_nsec / 1e3 );
			
			/* Timeout */
			if ( elapsed_us / 100000 > UART_READ_TIMEOUT )
				break;
			
		} while ( res < 1 );
		
		// Check response
		if ( res != 1 )
			fprintf( stderr, "No response.\n" );
		if ( rcv !=  UART_PING_CHAR )
			fprintf( stderr, "Invalid response.\n" );
		fprintf( stderr, "Delay: %llu us\n", elapsed_us );
	}
	
	// Restoring serial port initial configuration
	uart_release_port( );
	
	return 0;
}
