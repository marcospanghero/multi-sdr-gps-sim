/**
 * multi-sdr-gps-sim generates a IQ data stream on-the-fly to simulate a
 * GPS L1 baseband signal using a SDR platform like HackRF or ADLAM-Pluto.
 *
 * This file is part of the Github project at
 * https://github.com/mictronics/multi-sdr-gps-sim.git
 *
 * Copyright © 2021 Mictronics
 * Distributed under the MIT License.
 *
 */

#ifndef GPS_SIM_H
#define GPS_SIM_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <pthread.h>
#include <stdatomic.h>
#include "gps-core.h"

#include "serial_api/gpsd.h"
#include "serial_api/gps.h"

#include "gps-core.h"

#include "gui.h"

#define PRINT_MSG(f_, ...) gui_gps_status_wprintw(DEFAULT, ( f_), ##__VA_ARGS__)


#define NOTUSED(V) ((void) V)

// Sampling data format
#define SC08 sizeof(signed char)
#define SC16 sizeof(signed short)

/* SDR device types */
typedef enum {
    SDR_NONE = 0, SDR_IQFILE, SDR_HACKRF, SDR_PLUTOSDR, SDR_LIME
} sdr_type_t;

/* Target information. */
typedef struct {
    double bearing;
    double distance;
    double lat;
    double lon;
    double height;
    double velocity;
    double speed;
    double vertical_speed;
    double drift_rate;
    int drift_sign;
    bool valid;
} target_t;

/* Simulator location. */
typedef struct {
    double lat; // Latitude
    double lon; // Longitude
    double height; // Height/Elevation
    gpstime_t start;
} location_t;

struct fixsource_t
/* describe a data source */
{
    char *spec;         /* pointer to actual storage */
    char *server;
    char *port;
    char *device;
};

/* All the GPS simulators variables. */
typedef struct {
    atomic_bool main_exit;
    atomic_bool gps_thread_exit;
    atomic_bool gps_thread_running;
    atomic_bool gps_serial_thread_exit;
    atomic_bool gps_serial_thread_running;

    pid_t main_thread;
    pid_t serial_thread;
    pid_t gps_core_thread;

    ephem_t eph[EPHEM_ARRAY_SIZE][MAX_SAT];
    ionoutc_t ionoutc;
    bool show_verbose;
    bool ionosphere_enable;
    bool interactive_mode;
    bool use_ftp;
    bool enable_tx_amp;
    bool use_rinex3;
    bool time_overwrite;
    bool almanac_enable;
    bool synchronizer;
    bool pre_synchronizer;
    timespec_t compensation;
    bool start_set;
    int duration;
    int tx_gain;
    int ppb;
    int sample_size;
    sdr_type_t sdr_type;
    char *nav_file_name;
    char *motion_file_name;
    char *sdr_name;
    char *pluto_uri;
    char *pluto_hostname;
    char *station_id;
    pthread_mutex_t gps_lock;
    pthread_t gps_thread;
    pthread_cond_t gps_init_done; // Condition signals GPS thread is running
    pthread_mutex_t gps_serial_lock;
    pthread_t gps_serial_thread;
    pthread_cond_t gps_serial_init_done; // Condition signals GPS thread is running
    location_t location; // Simulator geo location
    target_t target; // Target information
    datetime_t start; // Simulation start time
    struct gps_fix_t fixdata;
    gps_mask_t mask;
    bool external_data_ready;
    bool external;

} simulator_t;

void set_thread_name(const char *name);
int thread_to_core(int core_id);

#endif /* GPS_SIM_H */

