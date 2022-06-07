//
// Created by marco on 2021-01-15.
//

#include "serial-driver.h"
#include <linux/serial.h>    //Serial struct
#include <sys/ioctl.h>


int serial_port_init_port(char *port_name, serial_port_t *serial_port_config){

    serial_port_config->port_name = port_name;
    serial_port_config->port_descriptor = 0;
    serial_port_config->port_status = 0;
    serial_port_config->is_open = 0;

    serial_port_config->rp = &(serial_port_config->rbuf[BLEN]);
    return 0;
}

unsigned char getbyte(serial_port_t *serial_port_config){

    if ((serial_port_config->rp - serial_port_config->rbuf) >= serial_port_config->bufcnt) {
        /* buffer needs refill */
        serial_port_config->bufcnt = read(serial_port_config->port_descriptor, serial_port_config->rbuf, BLEN);
        if (serial_port_config->bufcnt <= 0) {
            /* report error, then abort */
        }
        serial_port_config->rp = serial_port_config->rbuf;
    }
    return *serial_port_config->rp++;
}

int set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        //printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetspeed(&tty, (speed_t)speed);

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */


    tty.c_iflag &= ~(IXON | IXOFF | IXANY);   /* no SW flowcontrol */

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);



    tty.c_oflag &= ~OPOST;

    tty.c_cc[VEOL] = 0;
    tty.c_cc[VEOL2] = 0;
    tty.c_cc[VEOF] = 0x04;

    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 1;

    struct serial_struct kernel_serial_settings;
    ioctl(fd, TIOCGSERIAL, &kernel_serial_settings);
    kernel_serial_settings.flags |= ASYNC_LOW_LATENCY;
    ioctl(fd, TIOCSSERIAL, &kernel_serial_settings);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        //printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

int set_blocking (int fd, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        //fprintf(stderr,"error %d from tggetattr\n", errno);
        return -1;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
        return -1;
        //fprintf(stderr,"error %d setting term attributes\n", errno);
    return 0;
}


int serial_port_open_port(serial_port_t *serial_port_config){

    serial_port_config->port_descriptor = open(serial_port_config->port_name, O_RDWR | O_NOCTTY | O_NDELAY);

    if (serial_port_config->port_descriptor < 0) {
        //fprintf(stderr, "Could not open port %s \n",serial_port_config->port_name);
        return 1;
    } else {
        set_interface_attribs (serial_port_config->port_descriptor, B9600, 0);
        if (tcgetattr (serial_port_config->port_descriptor, &(serial_port_config->options)) != 0)
        {
            //fprintf(stderr,"error %d from tggetattr\n", errno);
            return 1;
        }

    }
    //fprintf(stdout, "Successfully opened port %s\n", serial_port_config->port_name);
    serial_port_config->is_open = 1;
    return 0;

}

int serial_port_set_raw_8bit(serial_port_t *serial_port_config) {

    if (serial_port_config->is_open == 1) {
        *(&(serial_port_config->options).c_cflag)  &= ~(ICANON | ECHO | ECHOE | ISIG);
        *(&(serial_port_config->options).c_cflag)  &= ~OPOST;
        *(&(serial_port_config->options).c_cflag)  &= ~(IXON | IXOFF | IXANY | IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
        *(&(serial_port_config->options).c_cflag)  &= ~CSIZE; /* Mask the character size bits */
        *(&(serial_port_config->options).c_cflag)  |= CREAD | CLOCAL;
        *(&(serial_port_config->options).c_cflag)  |= CS8;    /* Select 8 data bits */
        tcsetattr(serial_port_config->port_descriptor, TCSANOW, &(serial_port_config->options));

    } else {
        //fprintf(stderr, "Port is not open\n");
        return 1;
    }
    //fprintf(stdout, "Configured raw options for port %s\n", serial_port_config->port_name);
    return 0;
}

int serial_port_close(serial_port_t *serial_port_config){
    if (serial_port_config->is_open == 1) {
        close(serial_port_config->port_descriptor);
    } else {
        //fprintf(stderr, "Port is not open\n");
        return 1;
    }
    //fprintf(stdout, "Succesfully closed port %s\n", serial_port_config->port_name);
    return 0;
}