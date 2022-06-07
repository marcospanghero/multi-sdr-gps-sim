//
// Created by marco on 2021-01-15.
//

#ifndef DEMO_PPS_SERIAL_DRIVER_H
#define DEMO_PPS_SERIAL_DRIVER_H

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */


#define BLEN    4096


typedef struct {
    char *port_name;
    int port_descriptor;
    int port_status;
    int is_open;
    struct termios options;

    unsigned char rbuf[BLEN];
    unsigned char *rp;
    int bufcnt;

}serial_port_t;

/*
 * Initialize the serial port and the port status structure.
 */
int serial_port_init_port(char *port_name, serial_port_t *serial_port_config);

/*
 * Initialize the serial port and the port status structure. Call only once.
 */
int serial_port_open_port(serial_port_t *serial_port_config);


int set_interface_attribs (int fd, int speed, int parity);
int set_blocking (int fd, int should_block);
unsigned char getbyte(serial_port_t *serial_port_config);



/*
 * Set serial port in RAW io mode, avoids the serial port do substitute special character and ignores them - treat special chars as bytes
 */
int serial_port_set_raw_8bit(serial_port_t *serial_port_config);


/*
 * Close serial port
 */
int serial_port_close(serial_port_t *serial_port_config);


#endif //DEMO_PPS_SERIAL_DRIVER_H
