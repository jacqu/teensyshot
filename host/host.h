/*
 *  Definitions for host.c
 */
 
#ifndef __HOST_H
#define __HOST_H

#include "../ESCPID/ESCPID.h"

// Defines
#define HOST_ERROR_FD         -1        // Non existant file descriptor
#define HOST_ERROR_DEV        -2        // Non existant serial device
#define HOST_ERROR_MAX_DEV    -3        // Maximum devices limit 
#define HOST_ERROR_WRITE_SER  -4        // Write error on serial
#define HOST_ERROR_BAD_PK_SZ  -5        // Bad incoming packet size error
#define HOST_ERROR_MAGIC      -6        // Bad magic number received

// Prototypes
char *Host_name_from_serial(  uint32_t );
int   Host_get_fd(            uint32_t );
int   Host_init_port(         uint32_t );
void  Host_release_port(      uint32_t );
int   Host_comm_update(       uint32_t, 
                              int16_t*, 
                              uint16_t*, 
                              uint16_t*, 
                              uint16_t*, 
                              uint16_t*,
                              ESCPIDcomm_struct_t** );

#endif