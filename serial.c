#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <pthread.h>

#include <unistd.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include "serial_api/serial-driver.h"
#include "serial_api/driver_ubx.h"
#include "serial_api/gpsd.h"

#include <signal.h>
#include "sdr.h"
#include "gui.h"
#include "gps-sim.h"
#include "serial.h"

static const int gui_y_offset = 4;
static const int gui_x_offset = 2;

serial_port_t serial_port;
char *port_name = "/dev/ttyACM0";

uint8_t ret = 0;

void *gps_serial_thread_ep(void *arg)
{

    bool run = true;

    simulator_t *simulator = (simulator_t *)(arg);

    ret = serial_port_init_port(port_name, &serial_port);
    ret = serial_port_open_port(&serial_port);

    size_t avb = 0;

    size_t multi_fragment_offset = 0;
    size_t packet_len = 0;
    size_t expected_len = 0;

    timespec_t rx_time = (timespec_t){0, 0};
    timespec_t actual_time = (timespec_t){0, 0};

    struct gps_device_t device;
    gps_mask_t mask = 0;

    // PPS structures
    int num;
    int avail_mode;
    int i = 0;
    int ret;

    memset(&device, 0, sizeof(struct gps_device_t));

    // USE NAV_SOL as end of cycle
    device.driver.ubx.end_msgid = UBX_NAV_SOL;

    gps_clear_att(&device.gpsdata.attitude);
    gps_clear_dop(&device.gpsdata.dop);
    gps_clear_fix(&device.gpsdata.fix);
    device.gpsdata.dev.cycle = (timespec_t){1, 0};
    device.gpsdata.dev.mincycle = (timespec_t){1, 0};
    device.gpsdata.dev.parity = ' ';      // will be E, N, or O
    device.servicetype = SERVICE_UNKNOWN; // gpsd_open() sets this
    device.shm_clock_unit = -1;
    device.shm_pps_unit = -1;
    device.sourcetype = SOURCE_UNKNOWN;

    gpsd_zero_satellites(&device.gpsdata);

    unsigned char msg_buf[1024];
    unsigned char rx_buf[1024];
    memset(msg_buf, 0, 1024);
    memset(rx_buf, 0, 1024);

    struct test_t test;
    memset(&test, 0, sizeof(test));
    bool sent_once = false;
    // initialize things for the packet parser

    // setup the 1pps source
    struct timespec timeout = {2, 0};

    do
    {
        if (simulator->gps_serial_thread_running == false)
        {
            simulator->gps_serial_thread_running = true;
            pthread_cond_signal(&(simulator->gps_serial_init_done));
            gui_gps_status_wprintw(GREEN, "Started loop\n");
        }
        //fetch_source(0, &handle, &avail_mode, &info);
        ioctl(serial_port.port_descriptor, FIONREAD, &avb);
        if (avb > 0)
        {
            ret = read(serial_port.port_descriptor, &rx_buf, avb);
            if (rx_buf[0] == 0xb5 && rx_buf[1] == 0x62)
            {
                clock_gettime(CLOCK_REALTIME, &rx_time);
                expected_len = rx_buf[4] | rx_buf[5] << 8;
                multi_fragment_offset = 0;
                packet_len = 0;
            }
            else
            {
                //gui_status_wprintw(DEFAULT,"Multiframgent \n");
                multi_fragment_offset += ret;
            }
            packet_len += ret;
            memcpy(&msg_buf[multi_fragment_offset], &rx_buf, ret);
            if (packet_len == expected_len + 8)
            {
                //gui_status_wprintw(DEFAULT,"[%ld.%ld] [%d] MSG FULL Len: %ld %x%x \n", rx_time.tv_sec, rx_time.tv_nsec, device.gpsdata.subframe.subframe_num, packet_len, msg_buf[0], msg_buf[1]);
                mask = ubx_parse(&device, msg_buf, packet_len);
                if (SUBFRAME_SET == (mask & SUBFRAME_SET) && device.gpsdata.subframe.subframe_num == 4 && !sent_once)
                {
                    // We got a subframe and it is number 4! We fire the presync away
                    // We need a reliable cycle start! at the moment we send a ton of signals
                    kill(simulator->main_thread, SIGUSR1);
                    clock_gettime(CLOCK_REALTIME, &actual_time);
                    gui_status_wprintw(GREEN, "Sent SIGUSER1: because of superframe  - %d:%d\n", rx_time.tv_sec, rx_time.tv_nsec);
                    sent_once = true;
                }
                //gui_status_wprintw(GREEN, "%s\n", gps_maskdump(mask));

                multi_fragment_offset = 0;
                packet_len = 0;
                simulator->fixdata = device.gpsdata.fix;
                simulator->mask |= mask;
                gui_mvwprintw(LS_FIX, 18, 40, "%s\n", gps_maskdump(simulator->mask));
            }
        }
    } while (simulator->gps_serial_thread_exit == false);

end_gps_thread:
    gui_status_wprintw(RED, "Exit Serial thread\n");
    serial_port_close(serial_port.port_descriptor);
    simulator->gps_serial_thread_exit = true;
    pthread_cond_signal(&(simulator->gps_serial_init_done));
    pthread_exit(NULL);
}