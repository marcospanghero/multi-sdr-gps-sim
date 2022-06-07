#ifndef CURL_STATIONS_H
#define CURL_STATIONS_H

#include <curl/curl.h>
#include <zlib.h>
#include "almanac.h"
#include "gui.h"
#include "gps-sim.h"
#include "gps-core.h"




#define COOKIE_STRING "ProxyAuth=VxS0fdwPQxuVoQHlI+x3RxGRsEQDgSWPUgHZ/ISNxwhv1zr+1vThRBOR5YI7nPYKtG6ukVqMOQbqWZWWTUPgYY1kaPbyARkP9lznhQVGT6Yy7wwiYGL4ejLdEYntHZFo"

static char rinex_date[21];

struct ftp_file
{
    const char *filename;
    FILE *stream;
};


int get_station_rinex_2(simulator_t *simulator);
size_t fwrite_rinex(void *buffer, size_t size, size_t nmemb, void *stream);
int readRinex2(ephem_t eph[][MAX_SAT], ionoutc_t *ionoutc, const char *fname);
int readRinex3(ephem_t eph[][MAX_SAT], ionoutc_t *ionoutc, const char *fname);
int replaceExpDesignator(char *str, int len);

#endif