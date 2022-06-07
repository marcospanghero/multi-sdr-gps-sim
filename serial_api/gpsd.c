#include "gpsd_config.h"   // must be before all includes

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "gpsd.h"

size_t strlcat(char *dst, const char *src, size_t siz)
{
    char *d = dst;
    const char *s = src;
    size_t n = siz;
    size_t dlen;

    /* Find the end of dst and adjust bytes left but don't go past end */
    while (n-- != 0 && *d != '\0')
        d++;
    dlen = (size_t) (d - dst);
    n = siz - dlen;

    if (n == 0)
        return (dlen + strlen(s));
    while (*s != '\0') {
        if (n != 1) {
            *d++ = *s;
            n--;
        }
        s++;
    }
    *d = '\0';

    return (dlen + (s - src));  /* count does not include NUL */
}

/*
 * Copy src to string dst of size siz.  At most siz-1 characters
 * will be copied.  Always NUL terminates (unless siz == 0).
 * Returns strlen(src); if retval >= siz, truncation occurred.
 */
size_t strlcpy(char *dst, const char *src, size_t siz)
{
    size_t len = strlen(src);
    if (siz != 0) {
        if (len >= siz) {
            memcpy(dst, src, siz - 1);
            dst[siz - 1] = '\0';
        } else
            memcpy(dst, src, len + 1);
    }
    return len;
}

// End of strlcat()/strlcpy() section

size_t strnlen(const char *s, size_t maxlen)
{
    size_t len = 0;
    while (len < maxlen && *s++) ++len;
    return len;
}


// gpsd_gpstime_resolv() convert GPS week/tow to UTC as a timespec
timespec_t gpsd_gpstime_resolv(struct gps_device_t *session,
                               unsigned week, timespec_t tow)
{
    timespec_t t;

    /*
     * This code detects and compensates for week counter rollovers that
     * happen while gpsd is running. It will not save you if there was a
     * rollover that confused the receiver before gpsd booted up.  It *will*
     * work even when Block IIF satellites increase the week counter width
     * to 13 bits.
     */
    if ((int)week < (session->context->gps_week & 0x3ff)) {
        ++session->context->rollovers;
    }

    /*
     * This guard copes with both conventional GPS weeks and the "extended"
     * 15-or-16-bit version with no wraparound that appears in Zodiac
     * chips and is supposed to appear in the Geodetic Navigation
     * Information (0x29) packet of SiRF chips.  Some SiRF firmware versions
     * (notably 231) actually ship the wrapped 10-bit week, despite what
     * the protocol reference claims.
     */
    if (1024 > week) {
        week += session->context->rollovers * 1024;
    }

    /* This used to sanity check week number, GPS epoch, against leap
     * seconds.  Did not work well with regressions because the leap_sconds
     * could be from the receiver, or from BUILD_LEAPSECONDS.
     * Maybe if the regressions files provided BUILD_LEAPSECONDS this
     * could be tried again.
     */

    // gcc needs the (time_t)week to not overflow. clang got it right.
    // if time_t is 32-bits, then still 2038 issues
    t.tv_sec = GPS_EPOCH + ((time_t)week * SECS_PER_WEEK) + tow.tv_sec;
    t.tv_sec -= session->context->leap_seconds;
    t.tv_nsec = tow.tv_nsec;

#if 4 < SIZEOF_TIME_T
    // 2038 rollover hack for unsigned 32-bit time, assuming today is < 2038
    if (0 > t.tv_sec) {
        // recompute for previous EPOCH
        week -= 1024;
        t.tv_sec = GPS_EPOCH + ((time_t)week * SECS_PER_WEEK) + tow.tv_sec;
        t.tv_sec -= session->context->leap_seconds;
        GPSD_LOG(LOG_WARN, &session->context->errout,
                 "2038 rollover. Adjusting to %lld. week %u leap %d\n",
                 (long long)t.tv_sec, week,
                 session->context->leap_seconds);
    }
#endif   // SIZEOF_TIME_T

    session->context->gps_week = week;
    session->context->gps_tow = tow;
    session->context->valid |= GPS_TIME_VALID;

    return t;
}

timespec_t gpsd_galtime_resolv(struct gps_device_t *session,
                               unsigned week, timespec_t tow)
{
    timespec_t t;
    /*
    *We liberally copy the implementations structure from the gpstime resolver. We just need to add a new macro that tells this function where is the beginning of time. For now we assume that Galileo will not overflow at a 10-bit week length, but play nice. We need to add week number to the context
    */

    t.tv_sec = GAL_EPOCH + ((time_t)week * SECS_PER_WEEK) + tow.tv_sec;
    t.tv_sec -= session->context->gal_leap_seconds;
    t.tv_nsec = tow.tv_nsec;

    return t;

    // NB: THIS CODE WILL BREAK IN 2038!!!!

}

 
static unsigned char parity_array[] = {
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0
};

unsigned int isgps_parity(isgps30bits_t th)
{
#define P_30_MASK       0x40000000u
 
#define PARITY_25       0xbb1f3480u
#define PARITY_26       0x5d8f9a40u
#define PARITY_27       0xaec7cd00u
#define PARITY_28       0x5763e680u
#define PARITY_29       0x6bb1f340u
#define PARITY_30       0x8b7a89c0u
    isgps30bits_t t;
    unsigned int p;
 
    /*
     * if (th & P_30_MASK)
     * th ^= W_DATA_MASK;
     */
 
    t = th & PARITY_25;
    p = parity_array[t & 0xff] ^ parity_array[(t >> 8) & 0xff] ^
        parity_array[(t >> 16) & 0xff] ^ parity_array[(t >> 24) & 0xff];
    t = th & PARITY_26;
    p = (p << 1) | (parity_array[t & 0xff] ^ parity_array[(t >> 8) & 0xff] ^
                    parity_array[(t >> 16) & 0xff] ^ parity_array[(t >> 24) &
                                                                  0xff]);
    t = th & PARITY_27;
    p = (p << 1) | (parity_array[t & 0xff] ^ parity_array[(t >> 8) & 0xff] ^
                    parity_array[(t >> 16) & 0xff] ^ parity_array[(t >> 24) &
                                                                  0xff]);
    t = th & PARITY_28;
    p = (p << 1) | (parity_array[t & 0xff] ^ parity_array[(t >> 8) & 0xff] ^
                    parity_array[(t >> 16) & 0xff] ^ parity_array[(t >> 24) &
                                                                  0xff]);
    t = th & PARITY_29;
    p = (p << 1) | (parity_array[t & 0xff] ^ parity_array[(t >> 8) & 0xff] ^
                    parity_array[(t >> 16) & 0xff] ^ parity_array[(t >> 24) &
                                                                  0xff]);
    t = th & PARITY_30;
    p = (p << 1) | (parity_array[t & 0xff] ^ parity_array[(t >> 8) & 0xff] ^
                    parity_array[(t >> 16) & 0xff] ^ parity_array[(t >> 24) &
                                                                  0xff]);
 
#ifdef __UNUSED__
    GPSD_LOG(ISGPS_ERRLEVEL_BASE + 2, errout, "ISGPS parity %u\n", p);
#endif /* __UNUSED__ */
    return (p);
}
 


const char *gps_maskdump(gps_mask_t set)
{
    static char buf[242];
    const struct {
        gps_mask_t      mask;
        const char      *name;
    } *sp, names[] = {
        {ONLINE_SET,	"ONLINE"},
        {TIME_SET,	"TIME"},
        {TIMERR_SET,	"TIMERR"},
        {LATLON_SET,	"LATLON"},
        {ALTITUDE_SET,	"ALTITUDE"},
        {SPEED_SET,	"SPEED"},
        {TRACK_SET,	"TRACK"},
        {CLIMB_SET,	"CLIMB"},
        {STATUS_SET,	"STATUS"},
        {MODE_SET,	"MODE"},
        {DOP_SET,	"DOP"},
        {HERR_SET,	"HERR"},
        {VERR_SET,	"VERR"},
        {ATTITUDE_SET,	"ATTITUDE"},
        {SATELLITE_SET,	"SATELLITE"},
        {SPEEDERR_SET,	"SPEEDERR"},
        {TRACKERR_SET,	"TRACKERR"},
        {CLIMBERR_SET,	"CLIMBERR"},
        {DEVICE_SET,	"DEVICE"},
        {DEVICELIST_SET,	"DEVICELIST"},
        {DEVICEID_SET,	"DEVICEID"},
        {RTCM2_SET,	"RTCM2"},
        {RTCM3_SET,	"RTCM3"},
        {AIS_SET,	"AIS"},
        {PACKET_SET,	"PACKET"},
        {SUBFRAME_SET,	"SUBFRAME"},
        {GST_SET,	"GST"},
        {VERSION_SET,	"VERSION"},
        {POLICY_SET,	"POLICY"},
        {LOGMESSAGE_SET,	"LOGMESSAGE"},
        {ERROR_SET,	"ERROR"},
        {TOFF_SET,	"TOFF"},
        {PPS_SET,	"PPS"},
        {NAVDATA_SET,	"NAVDATA"},
        {OSCILLATOR_SET,	"OSCILLATOR"},
        {ECEF_SET,	"ECEF"},
        {VECEF_SET,	"VECEF"},
        {MAGNETIC_TRACK_SET,	"MAGNETIC_TRACK"},
        {RAW_SET,	"RAW"},
        {NED_SET,	"NED"},
        {VNED_SET,	"VNED"},
        {LOG_SET,	"LOG"},
        {IMU_SET,	"IMU"},
        {CLOCK_SET,	"CLOCK"},
        {GALTIME_SET,	"GALTIME"},
        {RAW_IS,	"RAW"},
        {USED_IS,	"USED"},
        {DRIVER_IS,	"DRIVER"},
        {CLEAR_IS,	"CLEAR"},
        {REPORT_IS,	"REPORT"},
        {NODATA_IS,	"NODATA"},
        {NTPTIME_IS,	"NTPTIME"},
        {PERR_IS,	"PERR"},
        {PASSTHROUGH_IS,	"PASSTHROUGH"},
        {EOF_IS,	"EOF"},
        {GOODTIME_IS,	"GOODTIME"},
        {GPSTIME_SET,	"GPSTIME"},
    };

    memset(buf, '\0', sizeof(buf));
    buf[0] = '{';
    for (sp = names; sp < names + sizeof(names)/sizeof(names[0]); sp++)
        if ((set & sp->mask)!=0) {
            (void)strlcat(buf, sp->name, sizeof(buf));
            (void)strlcat(buf, "|", sizeof(buf));
        }
    if (buf[1] != '\0')
        buf[strlen(buf)-1] = '\0';
    (void)strlcat(buf, "}", sizeof(buf));
    return buf;
}



