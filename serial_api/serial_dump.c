#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#include <stdint.h>

#include <unistd.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include "serial-driver.h"
#include "driver_ubx.h"

#include "gpsd.h"

#include <signal.h>

serial_port_t serial_port;
char *port_name = "/dev/ttyACM0";

uint8_t ret = 0;
bool run = true;


void sig_handler(int sig)
{
    if (sig == SIGINT)
    {
        printf("GPS parsing stopped by SIGINT\n");
        run = false;
    }
}

void print_hex(unsigned char *s, int len)
{
    int x = 0;
    while (x <= len)
    {
        printf("%02x ", (unsigned char)*s++);
        x++;
    }
    printf("\n");
}

int main()
{
    ret = serial_port_init_port(port_name, &serial_port);
    ret = serial_port_open_port(&serial_port);

    size_t avb = 0;

    size_t multi_fragment_offset = 0;
    size_t packet_len = 0;
    size_t expected_len = 0;

    timespec_t rx_time = (timespec_t){0,0};

    struct gps_device_t device;
    gps_mask_t mask = 0;

    memset(&device, 0, sizeof(struct gps_device_t));


    // necessary in case we start reading in the middle of a GPGSV sequence
    gpsd_zero_satellites(&device.gpsdata);

    unsigned char msg_buf[1024];
    unsigned char rx_buf[1024];
    memset(msg_buf, 0, 1024);
    memset(rx_buf, 0, 1024);

    // initialize things for the packet parser

    do
    {
        ioctl(serial_port.port_descriptor, FIONREAD, &avb);
        if (avb > 0)
        {
            ret = read(serial_port.port_descriptor, &rx_buf, avb);
            // printf("RX Fragment %d %02x%02x EXP Len: %d\n", ret, rx_buf[0], rx_buf[1], expected_len);
            // print_hex(rx_buf, ret);
            if (rx_buf[0] == 0xb5 && rx_buf[1] == 0x62)
            {
                clock_gettime(CLOCK_MONOTONIC, &rx_time);
                expected_len = rx_buf[4] | rx_buf[5] << 8;
                multi_fragment_offset = 0;
                packet_len = 0;
            }
            else
            {
                multi_fragment_offset += ret;
                // printf("Multifragment packet: %d\n", multi_fragment_offset);
            }
            packet_len += ret;
            memcpy(&msg_buf[multi_fragment_offset], &rx_buf, ret);
            if (packet_len == expected_len + 8)
            {
                printf("[%ld.%ld] [%d] MSG FULL Len: %ld %x%x \n", rx_time.tv_sec, rx_time.tv_nsec, device.gpsdata.subframe.subframe_num, packet_len, msg_buf[0], msg_buf[1]);
                mask = ubx_parse(&device, msg_buf, packet_len);
                printf("%s\n", gps_maskdump(mask));
                if (device.gpsdata.subframe.subframe_num == 5)
                {
                    printf("------------ SUPERFRAME START ------------ \n ---------- [%ld.%ld] ------------\n", rx_time.tv_sec, rx_time.tv_nsec);
                    device.gpsdata.subframe.subframe_num = 0;
                }
                // printf("\t");
                // print_hex(msg_buf, packet_len);
                multi_fragment_offset = 0;
                packet_len = 0;
            }
        }
    } while (run);
}