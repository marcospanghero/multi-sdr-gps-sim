/*
 * UBX driver.  For u-blox binary, also includes Antaris4 binary
 * Reference manuals are at
 * http://www.u-blox.com/en/download/documents-a-resources/u-blox-6-gps-modules-resources.html
 *
 * updated for u-blox 8
 * http://www.ublox.com/images/downloads/Product_Docs/u-bloxM8_ReceiverDescriptionProtocolSpec_%28UBX-13003221%29_Public.pdf
 *
 * Week counters are not limited to 10 bits. It's unknown what
 * the firmware is doing to disambiguate them, if anything; it might just
 * be adding a fixed offset based on a hidden epoch value, in which case
 * unhappy things will occur on the next rollover.
 *
 * For the Antaris 4, the default leap-second offset (before getting one from
 * the sats, one presumes) is 0sec; for the u-blox 6 it's 15sec.
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 *
 */

#include "gpsd_config.h" /* must be before all includes */

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "driver_ubx.h"
#include "gpsd.h"
#include "gps.h"

#include "bits.h" // For UINT2INT()
#include "timespec.h"

#include "../gps-sim.h"
//#define PRINT_MSG(f_, ...) printf((f_), ##__VA_ARGS__)


/*
 * Some high-precision messages provide data where the main part is a
 * signed 32-bit integer (same as the standard-precision versions),
 * and there's an 8-bit signed field providing an addend scaled to
 * 1/100th of the main value.  This macro provides a fetch for such
 * values, scaled to match the extension (i.e., 100X the main-value scale).
 * Since the fields are nonconsective, the offsets are provided separately.
 * The result is a signed 64-bit integer.
 *
 * The second macro incorporates scaling the result by a specified double.
 */
#define getles32x100s8(buf, off, offx) \
    ((int64_t)(getles32((buf), (off)) * 100LL + getsb((buf), (offx))))
#define getles32x100s8d(buf, off, offx, scale) \
    (getles32x100s8((buf), (off), (offx)) * (double)(scale))

/*
 * A ubx packet looks like this:
 * leader: 0xb5 0x62
 * message class: 1 byte
 * message type: 1 byte
 * length of payload: 2 bytes
 * payload: variable length
 * checksum: 2 bytes
 *
 * see also the FV25 and UBX documents on reference.html
 */
#define UBX_PREFIX_LEN 6
#define UBX_CLASS_OFFSET 2
#define UBX_TYPE_OFFSET 3

/* because we hates magic numbers forever */
#define USART1_ID 1
#define USART2_ID 2
#define USB_ID 3
#define UBX_PROTOCOL_MASK 0x01
#define NMEA_PROTOCOL_MASK 0x02
#define RTCM_PROTOCOL_MASK 0x04
#define RTCM3_PROTOCOL_MASK 0x20 // protVer 20+
#define UBX_CFG_LEN 20
#define outProtoMask 14

typedef struct
{
    const char *fw_string;
    const float protver;
} fw_protver_map_entry_t;

/* based on u-blox document no. GPS.G7-SW-12001-B1 (15 June 2018) */
/* capture decimal parts of protVer info even when session->protver currently
 * is integer (which _might_ change in the future, so avoid having to revisit
 * the info at that time).
 * This list is substantially incomplete and over specific. */
const fw_protver_map_entry_t fw_protver_map[] = {
    {"2.10", 8.10},  // antaris 4, version 8 is a guess
    {"2.11", 8.11},  // antaris 4, version 8 is a guess
    {"3.04", 9.00},  // antaris 4, version 9 is a guess
    {"4.00", 10.00}, // antaris 4, and u-blox 5
    {"4.01", 10.01}, // antaris 4, and u-blox 5
    {"5.00", 11.00}, // u-blox 5 and antaris 4
    {"6.00", 12.00}, // u-blox 5 and 6
    {"6.02", 12.02}, // u-blox 5 and 6
    {"7.01", 13.01}, // u-blox 7
    {"7.03", 13.03}, // u-blox 7
    {"1.00", 14.00}, // u-blox 6 w/ GLONASS, and 7
    // protVer >14 should carry explicit protVer in MON-VER extension
    {NULL, 0.0},
};

/*
 * Model  Fw          Protver
 * M10    SPG 5.00    34.00
 */

/* Convert a ubx PRN to an NMEA 4.0 (extended) PRN and ubx gnssid, svid
 *
 * return 0 on fail
 */
short ubx_to_prn(int ubx_PRN, unsigned char *gnssId,
                 unsigned char *svId)
{
    *gnssId = 0;
    *svId = 0;

    // IRNSS??
    if (1 > ubx_PRN)
    {
        // skip 0 PRN
        return 0;
    }
    else if (32 >= ubx_PRN)
    {
        // GPS 1..32 -> 1..32
        *gnssId = 0;
        *svId = ubx_PRN;
    }
    else if (64 >= ubx_PRN)
    {
        // BeiDou, 159..163,33..64 -> 1..5,6..37
        *gnssId = 3;
        *svId = ubx_PRN - 27;
    }
    else if (96 >= ubx_PRN)
    {
        // GLONASS 65..96 -> 1..32
        *gnssId = 6;
        *svId = ubx_PRN - 64;
    }
    else if (120 > ubx_PRN)
    {
        // Huh?
        return 0;
    }
    else if (158 >= ubx_PRN)
    {
        // SBAS 120..158 -> 120..158
        *gnssId = 1;
        *svId = ubx_PRN;
    }
    else if (163 >= ubx_PRN)
    {
        // BeiDou, 159..163 -> 1..5
        *gnssId = 3;
        *svId = ubx_PRN - 158;
    }
    else if (173 > ubx_PRN)
    {
        // Huh?
        return 0;
    }
    else if (182 >= ubx_PRN)
    {
        // IMES 173..182 -> 1..5, in u-blox 8, bot u-blox 9
        *gnssId = 4;
        *svId = ubx_PRN - 172;
    }
    else if (193 > ubx_PRN)
    {
        // Huh?
        return 0;
    }
    else if (199 >= ubx_PRN)
    {
        // QZSS 193..197 -> 1..5
        // ZED-F9T also see 198 and 199
        *gnssId = 5;
        *svId = ubx_PRN - 192;
    }
    else if (211 > ubx_PRN)
    {
        // Huh?
        return 0;
    }
    else if (246 >= ubx_PRN)
    {
        // Galileo 211..246 -> 1..36
        *gnssId = 2;
        *svId = ubx_PRN - 210;
    }
    else
    {
        // greater than 246, GLONASS (255), unused, or other unknown
        return 0;
    }
    return ubx2_to_prn(*gnssId, *svId);
}

// UBX-CFG-RATE
// Deprecated in u-blox 10
void ubx_msg_cfg_rate(struct gps_device_t *session, unsigned char *buf,
                      size_t data_len)
{
    return;
}

/* UBX-ESF-ALG
 *
 * UBX-ESF-ALG, and UBX-ESF-INS are synchronous to the GNSS epoch.
 * They need to be combined and reported together with the rest of
 * the epoch.
 */
gps_mask_t
ubx_msg_esf_alg(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{

    return 0;
}

/* UBX-ESF-INS
 *
 * protVer 19 and up.  ADR and UDR only
 *
 * UBX-ESF-ALG, and UBX-ESF-INS are synchronous to the GNSS epoch.
 * They need to be combined and reported together with the rest of
 * the epoch.
 */
gps_mask_t
ubx_msg_esf_ins(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    return 0;
}

/* UBX-ESF-MEAS
 *
 * protVer 15 and up.  ADR only
 * protVer 19 and up.  ADR and UDR only
 *
 * asynchronous to the GNSS epoch, and at a higher rate.
 * Needs to be reported immediately.
 *
 */
gps_mask_t
ubx_msg_esf_meas(struct gps_device_t *session, unsigned char *buf,
                 size_t data_len)
{
    return 0;
}

/* UBX-ESF-RAW
 *
 * protVer 15 and up.  ADR only
 * protVer 19 and up.  ADR and UDR only
 *
 * asynchronous to the GNSS epoch, and a a higher rate.
 * Needs to be reported immediately.
 *
 */
gps_mask_t
ubx_msg_esf_raw(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    return 0;
}

// UBX-ESF-STATUS
gps_mask_t
ubx_msg_esf_status(struct gps_device_t *session, unsigned char *buf,
                   size_t data_len)
{
    return 0;
}

/**
 * HNR Attitude solution
 * UBX-HNR-ATT Class x28, ID 1
 *
 * Not before u-blox 8, protVer 19.2 and up.
 * only on ADR, and UDR
 */
gps_mask_t
ubx_msg_hnr_att(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    return 0;
}

/**
 * HNR Vehicle dynamics information
 * UBX-HNR-INS Class x28, ID 2
 *
 * Not before u-blox 8, protVer 19.1 and up.
 * only on ADR, and UDR
 */
gps_mask_t
ubx_msg_hnr_ins(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    return 0;
}

/**
 * High rate output of PVT solution
 * UBX-HNR-PVT Class x28, ID 2
 *
 * Not before u-blox 8, protVer 19 and up.
 * only on ADR, and UDR
 */
gps_mask_t
ubx_msg_hnr_pvt(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    return 0;
}

/**
 * Receiver/Software Version
 * UBX-MON-VER
 *
 * sadly more info than fits in session->swtype for now.
 * so squish the data hard.
 */
gps_mask_t ubx_msg_mon_ver(struct gps_device_t *session,
                           unsigned char *buf,
                           size_t data_len)
{
    // output SW and HW Version at LOG_INF
    return 0;
}

/* UBX-MON-TXBUF
 * Present in u-blox 5+ through at least protVer 23.01
 * Supported but deprecated in M9P protVer 27.11
 * Supported but deprecated in M9N protVer 32.00 */
gps_mask_t
ubx_msg_mon_txbuf(struct gps_device_t *session, unsigned char *buf,
                  size_t data_len)
{
    return 0;
}

/* UBX-MON-RXBUF
 * Present in u-blox 5+ through at least protVer 23.01
 * Supported but deprecated in M9P protVer 27.11
 * Supported but deprecated in M9N protVer 32.00 */
gps_mask_t
ubx_msg_mon_rxbuf(struct gps_device_t *session, unsigned char *buf,
                  size_t data_len)
{
    return 0;
}

/**
 * UBX-LOG-BATCH entry only part of UBX protocol
 * Used for GPS standalone operation (internal batch retrieval)
 */
gps_mask_t
ubx_msg_log_batch(struct gps_device_t *session, unsigned char *buf,
                  size_t data_len)
{
    return 0;
}

/**
 * UBX-LOG-INFO info of log status
 * u-blox 7,8,9.  protVer 14 to 29
 * WIP: Initial decode, log only.
 *
 */
gps_mask_t
ubx_msg_log_info(struct gps_device_t *session, unsigned char *buf,
                 size_t data_len)
{
    return 0;
}

/*
 * UBX-LOG-RETRIEVEPOS (Indexed PVT entry)
 * Used for GPS standalone operation and host saved logs
 * u-blox 7,8,9.  protVer 14 to 29
 */
gps_mask_t
ubx_msg_log_retrievepos(struct gps_device_t *session, unsigned char *buf,
                        size_t data_len)
{
    return 0;
}

/*
 * UBX-LOG-RETRIEVEPOSEXTRA (Indexed Odometry entry)
 * Used for GPS standalone operation and host saved logs
 * u-blox 7,8,9.  protVer 14 to 29
 */
gps_mask_t
ubx_msg_log_retrieveposextra(struct gps_device_t *session,
                             unsigned char *buf, size_t data_len)
{
    return 0;
}

/*
 * UBX-NAV-HPPOSECEF - High Precision Position Solution in ECEF
 *
 * Present in u-blox 8 and above, protVwer 20.00 and up.
 * Only with High Precision firmware.
 */
gps_mask_t
ubx_msg_nav_hpposecef(struct gps_device_t *session, unsigned char *buf,
                      size_t data_len)
{
    return 0;
}

/**
 * High Precision Geodetic Position Solution
 * UBX-NAV-HPPOSLLH, Class 1, ID x14
 *
 * No mode, so limited usefulness.
 *
 * Present in u-blox 8 and above, protVwer 20.00 and up.
 * Only with High Precision firmware.
 */
gps_mask_t
ubx_msg_nav_hpposllh(struct gps_device_t *session, unsigned char *buf,
                     size_t data_len)
{
    return 0;
}

/*
 * Navigation Position ECEF message
 *
 * This message does not bother to tell us if it is valid.
 */
gps_mask_t
ubx_msg_nav_posecef(struct gps_device_t *session, unsigned char *buf,
                    size_t data_len)
{
    return 0;
}

/**
 * Navigation Position Velocity Time solution message
 * UBX-NAV-PVT Class 1, ID 7
 *
 * Not in u-blox 5 or 6, present in u-blox 7
 * u-blox 6 w/ GLONASS, protver 14 have NAV-PVT
 */
gps_mask_t
ubx_msg_nav_pvt(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    uint8_t valid;
    uint8_t flags;
    uint8_t fixType;
    struct tm unpacked_date;
    int *status = &session->gpsdata.fix.status;
    int *mode = &session->gpsdata.fix.mode;
    gps_mask_t mask = 0;
    char ts_buf[TIMESPEC_LEN];

    /* u-blox 6 and 7 are 84 bytes, u-blox 8 and 9 are 92 bytes  */
    if (84 > data_len)
    {
        return 0;
    }

    session->iTOW = getleu32(buf, 0);
    valid = (unsigned int)getub(buf, 11);
    fixType = (unsigned int)getub(buf, 20);
    flags = (unsigned int)getub(buf, 21);

    session->gpsdata.fix.mode = fixType;

    if ((valid & UBX_NAV_PVT_VALID_DATE_TIME) == UBX_NAV_PVT_VALID_DATE_TIME)
    {
        unpacked_date.tm_year = (uint16_t)getleu16(buf, 4) - 1900;
        unpacked_date.tm_mon = (uint8_t)getub(buf, 6) - 1;
        unpacked_date.tm_mday = (uint8_t)getub(buf, 7);
        unpacked_date.tm_hour = (uint8_t)getub(buf, 8);
        unpacked_date.tm_min = (uint8_t)getub(buf, 9);
        unpacked_date.tm_sec = (uint8_t)getub(buf, 10);
        unpacked_date.tm_isdst = 0;
        unpacked_date.tm_wday = 0;
        unpacked_date.tm_yday = 0;
        session->gpsdata.fix.time.tv_sec = mkgmtime(&unpacked_date);
        /* field 16, nano, can be negative! So normalize */
        session->gpsdata.fix.time.tv_nsec = getles32(buf, 16);
        TS_NORM(&session->gpsdata.fix.time);
        mask |= TIME_SET | NTPTIME_IS | GOODTIME_IS;
    }

    session->gpsdata.fix.longitude = 1e-7 * getles32(buf, 24);
    session->gpsdata.fix.latitude = 1e-7 * getles32(buf, 28);
    /* altitude WGS84 */
    session->gpsdata.fix.altHAE = 1e-3 * getles32(buf, 32);
    /* altitude MSL */
    session->gpsdata.fix.altMSL = 1e-3 * getles32(buf, 36);
    /* Let gpsd_error_model() deal with geoid_sep */

    session->gpsdata.fix.speed = 1e-3 * (int32_t)getles32(buf, 60);
    /* u-blox calls this Heading of motion (2-D) */
    session->gpsdata.fix.track = 1e-5 * (int32_t)getles32(buf, 64);
    mask |= LATLON_SET | ALTITUDE_SET | SPEED_SET | TRACK_SET;

    /* u-blox does not document the basis for the following "accuracy"
     * estimates.  Maybe CEP(50), one sigma, two sigma, CEP(99), etc. */

    /* Horizontal Accuracy estimate, in mm */
    session->gpsdata.fix.eph = (double)(getles32(buf, 40) / 1000.0);
    /* Vertical Accuracy estimate, in mm */
    session->gpsdata.fix.epv = (double)(getles32(buf, 44) / 1000.0);
    /* Speed Accuracy estimate, in mm/s */
    session->gpsdata.fix.eps = (double)(getles32(buf, 68) / 1000.0);
    /* let gpsd_error_model() do the rest */

    mask |= HERR_SET | SPEEDERR_SET | VERR_SET;
    // if cycle ender worked, could get rid of this REPORT_IS.
    // mask |= REPORT_IS;
    if (92 <= data_len)
    {
        // u-blox 8 and 9 extended
        double magDec = NAN;
        double magAcc = NAN;
#ifdef __UNUSED
        if (flags & UBX_NAV_PVT_FLAG_HDG_OK)
        {
            /* u-blox calls this Heading of vehicle (2-D)
             * why is it different than earlier track? */
            session->gpsdata.fix.track = (double)(getles32(buf, 84) * 1e-5);
        }
#endif // __UNUSED
        if (valid & UBX_NAV_PVT_VALID_MAG)
        {
            magDec = (double)(getles16(buf, 88) * 1e-2);
            magAcc = (double)(getleu16(buf, 90) * 1e-2);
        }
    }
    return mask;
}

/**
 * High Precision Relative Positioning Information in NED frame
 * UBX-NAV-RELPOSNED, Class 1, ID x3c
 * HP GNSS only, protver 20+
 */
gps_mask_t
ubx_msg_nav_relposned(struct gps_device_t *session, unsigned char *buf,
                      size_t data_len)
{
    return 0;
}

/**
 * Navigation solution message: UBX-NAV-SOL
 *
 * UBX-NAV-SOL, present in Antaris, up to 23,01
 * deprecated in u-blox 6, gone in u-blox 9.
 * Use UBX-NAV-PVT instead
 *
 * UBX-NAV-SOL has ECEF and VECEF, so no need for UBX-NAV-POSECEF and
 * UBX-NAV-VELECEF
 */
gps_mask_t ubx_msg_nav_sol(struct gps_device_t *session,
                           unsigned char *buf, size_t data_len)
{
    unsigned flags, pdop;
    unsigned char navmode;
    gps_mask_t mask;
    char ts_buf[TIMESPEC_LEN];

    if (52 > data_len)
    {
        return 0;
    }

    session->iTOW = getleu32(buf, 0);
    flags = (unsigned int)getub(buf, 11);
    mask = 0;
#define DATE_VALID (UBX_SOL_VALID_WEEK | UBX_SOL_VALID_TIME)
    if ((flags & DATE_VALID) == DATE_VALID)
    {
        unsigned short week;
        timespec_t ts_tow;

        MSTOTS(&ts_tow, session->iTOW);
        ts_tow.tv_nsec += (long)getles32(buf, 4);
        TS_NORM(&ts_tow);
        week = (unsigned short)getles16(buf, 8);
        //session->gpsdata.fix.time = gpsd_gpstime_resolv(session, week, ts_tow);
        mask |= TIME_SET | NTPTIME_IS | GOODTIME_IS;
    }
#undef DATE_VALID

    session->gpsdata.fix.ecef.x = getles32(buf, 12) / 100.0;
    session->gpsdata.fix.ecef.y = getles32(buf, 16) / 100.0;
    session->gpsdata.fix.ecef.z = getles32(buf, 20) / 100.0;
    session->gpsdata.fix.ecef.pAcc = getleu32(buf, 24) / 100.0;
    session->gpsdata.fix.ecef.vx = getles32(buf, 28) / 100.0;
    session->gpsdata.fix.ecef.vy = getles32(buf, 32) / 100.0;
    session->gpsdata.fix.ecef.vz = getles32(buf, 36) / 100.0;
    session->gpsdata.fix.ecef.vAcc = getleu32(buf, 40) / 100.0;
    mask |= ECEF_SET | VECEF_SET;

    session->gpsdata.fix.eps = (double)(getles32(buf, 40) / 100.0);
    mask |= SPEEDERR_SET;

    pdop = getleu16(buf, 44);
    if (9999 > pdop)
    {
        session->gpsdata.dop.pdop = (double)(pdop / 100.0);
        mask |= DOP_SET;
    }
    session->gpsdata.satellites_used = (int)getub(buf, 47);

    navmode = (unsigned char)getub(buf, 10);
    switch (navmode)
    {
    case UBX_MODE_TMONLY:
        // Surveyed-in, better not have moved
        session->gpsdata.fix.mode = MODE_3D;
        session->gpsdata.fix.status = STATUS_TIME;
        break;
    case UBX_MODE_3D:
        session->gpsdata.fix.mode = MODE_3D;
        session->gpsdata.fix.status = STATUS_GPS;
        break;
    case UBX_MODE_2D:
        session->gpsdata.fix.mode = MODE_2D;
        session->gpsdata.fix.status = STATUS_GPS;
        break;
    case UBX_MODE_DR: // consider this too as 2D
        session->gpsdata.fix.mode = MODE_2D;
        session->gpsdata.fix.status = STATUS_DR;
        break;
    case UBX_MODE_GPSDR: // DR-aided GPS is valid 3D
        session->gpsdata.fix.mode = MODE_3D;
        session->gpsdata.fix.status = STATUS_GNSSDR;
        break;
    default:
        session->gpsdata.fix.mode = MODE_NO_FIX;
        session->gpsdata.fix.status = STATUS_UNK;
        break;
    }

    if (0 != (flags & UBX_SOL_FLAG_DGPS))
        session->gpsdata.fix.status = STATUS_DGPS;

    mask |= MODE_SET | STATUS_SET;
    // older u-blox, cycle ender may be iffy
    // so err o nthe side of over-reporting TPV
    mask |= REPORT_IS;
    return mask;
}

/**
 * Receiver navigation status
 * UBX-NAV-STATUS Class 1, ID 3
 *
 * Present in Antaris to 9-series
 */
gps_mask_t
ubx_msg_nav_status(struct gps_device_t *session, unsigned char *buf,
                   size_t data_len)
{
    return 0;
}

/**
 * Navigation time to leap second: UBX-NAV-TIMELS
 *
 * Sets leap_notify if leap second is < 23 hours away.
 * Not in u-blox 5
 */
gps_mask_t
ubx_msg_nav_timels(struct gps_device_t *session, unsigned char *buf,
                   size_t data_len)
{
    return 0;
}

/**
 * Geodetic position solution message
 * UBX-NAV-POSLLH, Class 1, ID 2
 *
 * This message does not bother to tell us if it is valid.
 * No mode, so limited usefulness
 */
gps_mask_t
ubx_msg_nav_posllh(struct gps_device_t *session, unsigned char *buf,
                   size_t data_len)
{
    gps_mask_t mask = 0;

    if (28 > data_len)
    {
        return 0;
    }

    session->iTOW = getleu32(buf, 0);
    session->gpsdata.fix.longitude = 1e-7 * getles32(buf, 4);
    session->gpsdata.fix.latitude = 1e-7 * getles32(buf, 8);
    /* altitude WGS84 */
    session->gpsdata.fix.altHAE = 1e-3 * getles32(buf, 12);
    /* altitude MSL */
    session->gpsdata.fix.altMSL = 1e-3 * getles32(buf, 16);
    /* Let gpsd_error_model() deal with geoid_sep */

    /* Horizontal accuracy estimate in mm, unknown type */
    session->gpsdata.fix.eph = getleu32(buf, 20) * 1e-3;
    /* Vertical accuracy estimate in mm, unknown type */
    session->gpsdata.fix.epv = getleu32(buf, 24) * 1e-3;

    mask = ONLINE_SET | HERR_SET | VERR_SET | LATLON_SET | ALTITUDE_SET;
    return mask;
}

/**
 * Clock Solution
 *
 * Present in u-blox 7
 */
gps_mask_t
ubx_msg_nav_clock(struct gps_device_t *session, unsigned char *buf,
                  size_t data_len)
{
    gps_mask_t mask = CLOCK_SET;

    if (20 > data_len)
    {
        return 0;
    }

    session->iTOW = getleu32(buf, 0);
    session->gpsdata.fix.clock_bias = getles32(buf, 4);
    session->gpsdata.fix.clock_drift = getles32(buf, 8);
    session->gpsdata.fix.tAcc_estimate = getleu32(buf, 12);
    session->gpsdata.fix.fAcc_estimate = getleu32(buf, 16);
    return mask;
}

/**
 * DGPS Data Used for NAV
 *
 * May be good cycle ender
 *
 * Present in u-blox 7
 */
gps_mask_t ubx_msg_nav_dgps(struct gps_device_t *session,
                            unsigned char *buf, size_t data_len)
{
    return 0;
}

/**
 * Dilution of precision message
 */
gps_mask_t
ubx_msg_nav_dop(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    unsigned u;
    gps_mask_t mask = 0;

    if (18 > data_len)
    {
        return 0;
    }

    session->iTOW = getleu32(buf, 0);
    /*
     * We make a deliberate choice not to clear DOPs from the
     * last skyview here, but rather to treat this as a supplement
     * to our calculations from the visibility matrix, trusting
     * the firmware algorithms over ours.
     */
    u = getleu16(buf, 4);
    if (9999 > u)
    {
        session->gpsdata.dop.gdop = (double)(u / 100.0);
        mask |= DOP_SET;
    }
    u = getleu16(buf, 6);
    if (9999 > u)
    {
        session->gpsdata.dop.pdop = (double)(u / 100.0);
        mask |= DOP_SET;
    }
    u = getleu16(buf, 8);
    if (9999 > u)
    {
        session->gpsdata.dop.tdop = (double)(u / 100.0);
        mask |= DOP_SET;
    }
    u = getleu16(buf, 10);
    if (9999 > u)
    {
        session->gpsdata.dop.vdop = (double)(u / 100.0);
        mask |= DOP_SET;
    }
    u = getleu16(buf, 12);
    if (9999 > u)
    {
        session->gpsdata.dop.hdop = (double)(u / 100.0);
        mask |= DOP_SET;
    }
    return mask;
}

/**
 * Position error ellipse parameters
 * protVer 19.1 and up
 * Not in u-blox 5, 6 or 7
 * Present in some u-blox 8, 9 and 10 (ADR, HPS)
 */
gps_mask_t
ubx_msg_nav_eell(struct gps_device_t *session, unsigned char *buf,
                 size_t data_len)
{
    return 0;
}

/**
 * End of Epoch
 * Not in u-blox 5, 6 or 7
 * Present in u-blox 8 and 9
 */
gps_mask_t
ubx_msg_nav_eoe(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    return 0;
}

/**
 * GPS Leap Seconds - UBX-NAV-TIMEGPS
 */
gps_mask_t
ubx_msg_nav_timegps(struct gps_device_t *session, unsigned char *buf,
                    size_t data_len)
{
    uint8_t valid; /* Validity Flags */
    gps_mask_t mask = 0;
    char ts_buf[TIMESPEC_LEN];

    if (16 > data_len)
    {
        return 0;
    }

    session->iTOW = getleu32(buf, 0);
    valid = getub(buf, 11);
    // Valid leap seconds ?
    if ((valid & UBX_TIMEGPS_VALID_LEAP_SECOND) ==
        UBX_TIMEGPS_VALID_LEAP_SECOND)
    {
        // session->context->leap_seconds = (int)getub(buf, 10);
        // session->context->valid |= LEAP_SECOND_VALID;
    }
    // Valid GPS time of week and week number
#define VALID_TIME (UBX_TIMEGPS_VALID_TIME | UBX_TIMEGPS_VALID_WEEK)
    if ((valid & VALID_TIME) == VALID_TIME)
    {
#undef VALID_TIME
        uint16_t week;
        double tAcc; /* Time Accuracy Estimate in ns */
        timespec_t ts_tow;

        week = getles16(buf, 8);
        MSTOTS(&ts_tow, session->iTOW);
        ts_tow.tv_nsec += (long)getles32(buf, 4);
        TS_NORM(&ts_tow);
        session->gpsdata.fix.gps_time_itow = ts_tow.tv_sec;
        session->gpsdata.fix.gps_time_ftow = ts_tow.tv_nsec;
        session->gpsdata.fix.gps_time_weekn = week;
        // session->gpsdata.fix.time = gpsd_gpstime_resolv(session, week, ts_tow);

        tAcc = (double)getleu32(buf, 12); // tAcc in ns
        session->gpsdata.fix.ept = tAcc * 1e-9;

        mask |= (GPSTIME_SET | TIME_SET | NTPTIME_IS);
    }

    return mask;
}

/**
 * UBX-NAV-TIMEGAL
 */

gps_mask_t
ubx_msg_nav_timegal(struct gps_device_t *session, unsigned char *buf,
                    size_t data_len)
{
    return 0;
}

/**
 * UBX-NAV-TIMEUTC
 */
gps_mask_t
ubx_msg_nav_timeutc(struct gps_device_t *session, unsigned char *buf,
                    size_t data_len)
{
    uint8_t valid; // Validity Flags
    gps_mask_t mask = 0;

    if (20 > data_len)
    {
        return 0;
    }

    session->iTOW = getleu32(buf, 0);
    valid = getub(buf, 19);
    if (4 == (4 & valid))
    {
        // UTC is valid
        struct tm date = {0};
        // mask |= (TIME_SET | NTPTIME_IS);
        uint32_t tAcc = getleu32(buf, 4); // tAcc in ns
        // nano can be negative, so this is not normalized UTC.
        int32_t nano = getles32(buf, 8);         // fract sec in ns
        date.tm_year = getleu16(buf, 12) - 1900; // year, 1999..2099
        date.tm_mon = getub(buf, 14) - 1;        // month 1..12
        date.tm_mday = getub(buf, 15);           // day 1..31
        date.tm_hour = getub(buf, 16);           // hour 0..23
        date.tm_min = getub(buf, 17);            // min 0..59
        date.tm_sec = getub(buf, 18);            // sec 0..60
        session->gpsdata.fix.time.tv_sec = mkgmtime(&date);
        session->gpsdata.fix.time.tv_nsec = nano;
        // nano, can be negative! So normalize
        TS_NORM(&session->gpsdata.fix.time);
        // other timestamped messages lack nano, so time will jump around...
        mask |= TIME_SET | NTPTIME_IS | GOODTIME_IS;
    }
    return mask;
}

/**
 * GPS Satellite Info -- new style UBX-NAV-SAT
 * Not in u-blox 5
 * Present in u-blox 8,  protocol version 15+
 */
gps_mask_t
ubx_msg_nav_sat(struct gps_device_t *session, unsigned char *buf,
                size_t data_len)
{
    unsigned int i, nchan, nsv, st, ver;
    timespec_t ts_tow;

    if (8 > data_len)
    {
        return 0;
    }

    session->iTOW = getleu32(buf, 0);
    MSTOTS(&ts_tow, session->iTOW);

    ver = (unsigned int)getub(buf, 4);
    if (1 != ver)
    {
        return 0;
    }
    nchan = (unsigned int)getub(buf, 5);
    if (nchan > MAXCHANNELS)
    {
        return 0;
    }
    /* two "unused" bytes at buf[6:7] */

    gpsd_zero_satellites(&session->gpsdata);
    nsv = 0;
    for (i = st = 0; i < nchan; i++)
    {
        unsigned int off = 8 + 12 * i;
        short nmea_PRN = 0;
        unsigned char gnssId = getub(buf, off + 0);
        short svId = (short)getub(buf, off + 1);
        unsigned char cno = getub(buf, off + 2);
        /* health data in flags. */
        uint32_t flags = getleu32(buf, off + 8);
        bool used = (bool)(flags & 0x08);
        int tmp;
        /* Notice NO sigid! */

        nmea_PRN = ubx2_to_prn(gnssId, svId);
        session->gpsdata.skyview[st].gnssid = gnssId;
        session->gpsdata.skyview[st].svid = svId;
        session->gpsdata.skyview[st].PRN = nmea_PRN;

        session->gpsdata.skyview[st].ss = (double)cno;
        tmp = getsb(buf, off + 3);
        if (90 >= abs(tmp))
        {
            session->gpsdata.skyview[st].elevation = (double)tmp;
        }
        tmp = getles16(buf, off + 4);
        if (359 > tmp && 0 <= tmp)
        {
            session->gpsdata.skyview[st].azimuth = (double)tmp;
        }
        session->gpsdata.skyview[st].used = used;
        /* by some coincidence, our health flags matches u-blox's */
        session->gpsdata.skyview[st].health = (flags >> 4) & 3;
        /* sbas_in_use is not same as used */
        if (used)
        {
            nsv++;
            session->gpsdata.skyview[st].used = true;
        }
        st++;
    }

    session->gpsdata.satellites_visible = (int)st;
    session->gpsdata.satellites_used = (int)nsv;
    return SATELLITE_SET | USED_IS;
}

/**
 * GPS Satellite Info -- deprecated - UBX-NAV-SVINFO
 * Not in u-blox 9 or 10, use UBX-NAV-SAT instead
 */
gps_mask_t
ubx_msg_nav_svinfo(struct gps_device_t *session, unsigned char *buf,
                   size_t data_len)
{
    unsigned int i, nchan, nsv, st;
    timespec_t ts_tow;

    if (8 > data_len)
    {

        return 0;
    }

    session->iTOW = getleu32(buf, 0);
    MSTOTS(&ts_tow, session->iTOW);

    nchan = (unsigned int)getub(buf, 4);
    if (nchan > MAXCHANNELS)
    {
        return 0;
    }
    gpsd_zero_satellites(&session->gpsdata);
    nsv = 0;
    for (i = st = 0; i < nchan; i++)
    {
        unsigned int off = 8 + 12 * i;
        short nmea_PRN;
        short ubx_PRN = (short)getub(buf, off + 1);
        unsigned char snr = getub(buf, off + 4);
        bool used = (bool)(getub(buf, off + 2) & 0x01);
        unsigned char flags = getub(buf, off + 12) & 3;
        int tmp;

        nmea_PRN = ubx_to_prn(ubx_PRN,
                              &session->gpsdata.skyview[st].gnssid,
                              &session->gpsdata.skyview[st].svid);

        if (1 > nmea_PRN)
        {
            // skip bad PRN
            continue;
        }
        session->gpsdata.skyview[st].PRN = nmea_PRN;

        session->gpsdata.skyview[st].ss = (double)snr;
        tmp = getsb(buf, off + 5);
        if (90 >= abs(tmp))
        {
            session->gpsdata.skyview[st].elevation = (double)tmp;
        }
        tmp = (double)getles16(buf, off + 6);
        if (359 > tmp && 0 <= tmp)
        {
            session->gpsdata.skyview[st].azimuth = (double)tmp;
        }
        session->gpsdata.skyview[st].used = used;
        if (0x10 & flags)
        {
            session->gpsdata.skyview[st].health = SAT_HEALTH_BAD;
        }
        else
        {
            session->gpsdata.skyview[st].health = SAT_HEALTH_OK;
        }

        /* sbas_in_use is not same as used */
        if (used)
        {
            /* not really 'used', just integrity data from there */
            nsv++;
            session->gpsdata.skyview[st].used = true;
        }
        st++;
    }

    session->gpsdata.satellites_visible = (int)st;
    session->gpsdata.satellites_used = (int)nsv;

    return SATELLITE_SET | USED_IS;
}

/*
 * Velocity Position ECEF message, UBX-NAV-VELECEF
 */
gps_mask_t
ubx_msg_nav_velecef(struct gps_device_t *session, unsigned char *buf,
                    size_t data_len)
{
    gps_mask_t mask = VECEF_SET;

    if (20 > data_len)
    {

        return 0;
    }

    session->iTOW = getleu32(buf, 0);
    session->gpsdata.fix.ecef.vx = getles32(buf, 4) / 100.0;
    session->gpsdata.fix.ecef.vy = getles32(buf, 8) / 100.0;
    session->gpsdata.fix.ecef.vz = getles32(buf, 12) / 100.0;
    session->gpsdata.fix.ecef.vAcc = getleu32(buf, 16) / 100.0;

    return mask;
}

/*
 * Velocity NED message, UBX-NAV-VELNED
 * protocol versions 15+
 */
gps_mask_t
ubx_msg_nav_velned(struct gps_device_t *session, unsigned char *buf,
                   size_t data_len)
{
    gps_mask_t mask = VNED_SET;

    if (36 > data_len)
    {

        return 0;
    }

    session->iTOW = getleu32(buf, 0);
    session->gpsdata.fix.NED.velN = getles32(buf, 4) / 100.0;
    session->gpsdata.fix.NED.velE = getles32(buf, 8) / 100.0;
    session->gpsdata.fix.NED.velD = getles32(buf, 12) / 100.0;
    return mask;
}

/*
 * SBAS Info UBX-NAV-SBAS
 * in u-blox 4_
 * in NEO-M9N
 * Not in some u-blox 9
 * Decode looks good, but data only goes to log.
 */
gps_mask_t
ubx_msg_nav_sbas(struct gps_device_t *session, unsigned char *buf,
                 size_t data_len)
{
    return 0;
}

/*
 * Multi-GNSS Raw measurement Data -- UBX-RXM-RAWX
 * Not in u-blox 5, 6 or 7
 * u-blox 9, message version 0 (but no version byte!)
 * u-blox 9, message version 1
 */
gps_mask_t ubx_msg_rxm_rawx(struct gps_device_t *session,
                            const unsigned char *buf,
                            size_t data_len)
{
    double rcvTow;
    uint16_t week;
    int8_t leapS;
    uint8_t numMeas;
    uint8_t recStat;
    uint8_t version;
    int i;
    const char *obs_code;
    timespec_t ts_tow;

    if (16 > data_len)
    {
        return 0;
    }

    /* Note: this is "approximately" GPS TOW, this is not iTOW */
    rcvTow = getled64((const char *)buf, 0); /* time of week in seconds */
    week = getleu16(buf, 8);
    leapS = getsb(buf, 10);
    numMeas = getub(buf, 11);

    //Save the number of measurements we have
    session->gpsdata.raw.avb_meas = numMeas;
    recStat = getub(buf, 12);
    /* byte 13 is version on u-blox 9, reserved on u-blox 8
     * how is that supposed to work?? */
    version = getub(buf, 13);

    if (recStat & 1)
    {
        /* Valid leap seconds */
        // session->context->leap_seconds = leapS;
        // session->context->valid |= LEAP_SECOND_VALID;
    }
    /* convert GPS weeks and "approximately" GPS TOW to UTC */
    DTOTS(&ts_tow, rcvTow);
    // Do not set gpsdata.fix.time.  set gpsdata.raw.mtime
    //session->gpsdata.raw.mtime = gpsd_gpstime_resolv(session, week, ts_tow);

    /* zero the measurement data */
    /* so we can tell which meas never got set */
    memset(session->gpsdata.raw.meas, 0, sizeof(session->gpsdata.raw.meas));

    if (numMeas > MAXCHANNELS)
    {
        return 0;
    }
    for (i = 0; i < numMeas; i++)
    {
        int off = 32 * i;
        /* pseudorange in meters */
        double prMes = getled64((const char *)buf, off + 16);
        /* carrier phase in cycles */
        double cpMes = getled64((const char *)buf, off + 24);
        /* doppler in Hz, positive towards sat */
        double doMes = getlef32((const char *)buf, off + 32);
        uint8_t gnssId = getub(buf, off + 36);
        uint8_t svId = getub(buf, off + 37);
        // reserved in u-blox 8, sigId in u-blox 9 (version 1)
        uint8_t sigId = getub(buf, off + 38);
        /* GLONASS frequency slot */
        uint8_t freqId = getub(buf, off + 39);
        /* carrier phase locktime in ms, max 64500ms */
        uint16_t locktime = getleu16(buf, off + 40);
        /* carrier-to-noise density ratio dB-Hz */
        uint8_t cno = getub(buf, off + 42);
        uint8_t prStdev = getub(buf, off + 43) & 0x0f;
        uint8_t cpStdev = getub(buf, off + 44) & 0x0f;
        uint8_t doStdev = getub(buf, off + 45) & 0x0f;
        /* tracking stat
         * bit 0 - prMes valid
         * bit 1 - cpMes valid
         * bit 2 - halfCycle valid
         * bit 3 - halfCycle subtracted from phase
         */
        uint8_t trkStat = getub(buf, off + 46);

        session->gpsdata.raw.meas[i].gnssid = gnssId;
        session->gpsdata.raw.meas[i].sigid = sigId;

        /* some of these are GUESSES as the u-blox codes do not
         * match RINEX codes */
        switch (gnssId)
        {
        case 0: /* GPS */
            switch (sigId)
            {
            default:
                /* let PPP figure it out */
            case 0: /* L1C/A */
                obs_code = "L1C";
                break;
            case 3: /* L2 CL */
                obs_code = "L2C";
                break;
            case 4: /* L2 CM */
                obs_code = "L2X";
                break;
            }
            break;
        case 1: /* SBAS */
            /* sigId added on protVer 27, and SBAS gone in protVer 27
             * so must be L1C/A */
            svId -= 100; /* adjust for RINEX 3 svid */

            obs_code = "L1C"; /* u-blox calls this L1C/A */
            /* SBAS can do L5I, but the code? */
            switch (sigId)
            {
            default:
                /* let PPP figure it out */
                break;
            case 0: /* L1C/A */
                obs_code = "L1C";
                break;
            }
            break;
        case 2: /* GALILEO */
            switch (sigId)
            {
            default:
                /* let PPP figure it out */
            case 0:               /*  */
                obs_code = "L1C"; /* u-blox calls this E1OS or E1C */
                break;
            case 1:               /*  */
                obs_code = "L1B"; /* u-blox calls this E1B */
                break;
            case 5:               /*  */
                obs_code = "L7I"; /* u-blox calls this E5bl */
                break;
            case 6:               /*  */
                obs_code = "L7Q"; /* u-blox calls this E5bQ */
                break;
            }
            break;
        case 3: /* BeiDou */
            switch (sigId)
            {
            default:
                /* let PPP figure it out */
            case 0:               /*  */
                obs_code = "L2Q"; /* u-blox calls this B1I D1 */
                break;
            case 1:               /*  */
                obs_code = "L2I"; /* u-blox calls this B1I D2 */
                break;
            case 2:               /*  */
                obs_code = "L7Q"; /* u-blox calls this B2I D1 */
                break;
            case 3:               /*  */
                obs_code = "L7I"; /* u-blox calls this B2I D2 */
                break;
            }
            break;
        default:           /* huh? */
        case 4:            /* IMES.  really? */
            obs_code = ""; /* u-blox calls this L1 */
            break;
        case 5: /* QZSS */
            switch (sigId)
            {
            default:
                /* let PPP figure it out */
            case 0:               /*  */
                obs_code = "L1C"; /* u-blox calls this L1C/A */
                break;
            case 4:               /*  */
                obs_code = "L2S"; /* u-blox calls this L2CM */
                break;
            case 5:               /*  */
                obs_code = "L2L"; /* u-blox calls this L2CL*/
                break;
            }
            break;
        case 6: /* GLONASS */
            switch (sigId)
            {
            default:
                /* let PPP figure it out */
            case 0:               /*  */
                obs_code = "L1C"; /* u-blox calls this L1OF */
                break;
            case 2:               /*  */
                obs_code = "L2C"; /* u-blox calls this L2OF */
                break;
            }
            break;
        }
        (void)strlcpy(session->gpsdata.raw.meas[i].obs_code, obs_code,
                      sizeof(session->gpsdata.raw.meas[i].obs_code));

        session->gpsdata.raw.meas[i].svid = svId;
        session->gpsdata.raw.meas[i].freqid = freqId;
        session->gpsdata.raw.meas[i].snr = cno;
        session->gpsdata.raw.meas[i].satstat = trkStat;
        if (trkStat & 1)
        {
            /* prMes valid */
            session->gpsdata.raw.meas[i].pseudorange = prMes;
        }
        else
        {
            session->gpsdata.raw.meas[i].pseudorange = NAN;
        }
        if ((trkStat & 2) && (5 >= cpStdev))
        {
            /* cpMes valid, RTKLIB uses 5 < cpStdev */
            session->gpsdata.raw.meas[i].carrierphase = cpMes;
        }
        else
        {
            session->gpsdata.raw.meas[i].carrierphase = NAN;
        }
        session->gpsdata.raw.meas[i].doppler = doMes;
        session->gpsdata.raw.meas[i].codephase = NAN;
        session->gpsdata.raw.meas[i].deltarange = NAN;
        session->gpsdata.raw.meas[i].locktime = locktime;
        if (0 == locktime)
        {
            /* possible slip */
            session->gpsdata.raw.meas[i].lli = 2;
        }
    }

    return RAW_IS;
}

/*
 * Raw Subframes - UBX-RXM-SFRB
 * In u-blox 7, only in raw firmware option
 * Not in u-blox 8 or 9
 */
gps_mask_t ubx_msg_rxm_sfrb(struct gps_device_t *session,
                            unsigned char *buf, size_t data_len)
{
    unsigned int i, chan, svid;
    uint32_t words[10];

    if (42 > data_len)
    {
        PRINT_MSG("UBX-RXM-SFRB message, runt payload len %zd", data_len);
        return 0;
    }

    chan = (unsigned int)getub(buf, 0);
    svid = (unsigned int)getub(buf, 1);
   PRINT_MSG("UBX-RXM-SFRB: %u %u\n", chan, svid);

    /* UBX does all the parity checking, but still bad data gets through */
    for (i = 0; i < 10; i++)
    {
        // bits 24 to 31 undefined, remove them.
        words[i] = (uint32_t)getleu32(buf, 4 * i + 2) & 0x00ffffff;
    }

    // probably GPS, could be SBAS
    return gpsd_interpret_subframe(session, GNSSID_GPS, svid, words);
}

/*
 * Raw Subframes - UBX-RXM-SFRBX
 * in u-blox 8, protver 17 and up, time sync firmware only
 * in u-blox F9P abd HPG only
 * not present  before u-blox8
 */
gps_mask_t ubx_msg_rxm_sfrbx(struct gps_device_t *session,
                             unsigned char *buf, size_t data_len)
{
    unsigned i;
    uint8_t gnssId, svId, freqId, numWords, chn, version;
    uint32_t words[17];
    char *chn_s;

    if (8 > data_len)
    {

        return 0;
    }

    numWords = getub(buf, 4);
    if (data_len != (size_t)(8 + (4 * numWords)) ||
        16 < numWords)
    {
        return 0;
    }

    gnssId = getub(buf, 0);
    svId = getub(buf, 1);
    freqId = getub(buf, 2);
    version = getub(buf, 6);
    chn = getub(buf, 5);
    if (1 < version)
    {
        // receiver channel in version 2 and up.
        // valid range 0 to 13?
        chn_s = "chn";
    }
    else
    {
        chn_s = "reserved";
    }

    if (0 == version)
    {
        // unknown ersion
        return 0;
    }

    memset(words, 0, sizeof(words));
    for (i = 0; i < numWords; i++)
    {
        // grab the words, don't mangle them
        words[i] = (uint32_t)getleu32(buf, 4 * i + 8);
    }

    // do we need freqId or chn?
    return gpsd_interpret_subframe_raw(session, gnssId, svId, words, numWords);
}

/**
 * SV Status Info
 *
 * May be good cycle ender
 *
 * Present in u-blox 7
 */
gps_mask_t
ubx_msg_rxm_svsi(struct gps_device_t *session, unsigned char *buf,
                 size_t data_len)
{
    return 0;
}

/* UBX-INF-* */
gps_mask_t
ubx_msg_inf(struct gps_device_t *session, unsigned char *buf, size_t data_len)
{
    return 0;
}

/**
 * Survey-in data - UBX-TIM-SVIN
 * Time Sync products only
 */
gps_mask_t
ubx_msg_tim_svin(struct gps_device_t *session, unsigned char *buf,
                 size_t data_len)
{
    return 0;
}

/**
 * Time Pulse Timedata - UBX-TIM-TP
 */
gps_mask_t
ubx_msg_tim_tp(struct gps_device_t *session, unsigned char *buf,
               size_t data_len)
{
    return 0;
}

gps_mask_t ubx_parse(struct gps_device_t *session, unsigned char *buf,
                     size_t len)
{
    size_t data_len;
    unsigned short msgid;
    gps_mask_t mask = 0;

    // the packet at least contains a head long enough for an empty message
    if (UBX_PREFIX_LEN > len)
    {
        return 0;
    }

    // session->cycle_end_reliable = true;
    session->iTOW = -1; // set by decoder

    // extract message id and length
    msgid = (buf[2] << 8) | buf[3];
    data_len = (size_t)getles16(buf, 4);

    switch (msgid)
    {
    case UBX_ACK_ACK:
        if (2 <= data_len)
        {
        }
        break;
    case UBX_ACK_NAK:
        if (2 <= data_len)
        {
        }
        break;

    case UBX_CFG_NAV5:
        // deprecated in u-blox 10
        PRINT_MSG("UBX-CFG-NAV5\n");
        break;
    case UBX_CFG_NAVX5:
        // deprecated in u-blox 10
        PRINT_MSG("UBX-CFG-NAVX5\n");
        break;
    case UBX_CFG_PRT:
        // deprecated in u-blox 10
        PRINT_MSG("UBX-CFG-PRT\n");
        break;
    case UBX_CFG_RATE:
        // deprecated in u-blox 10
        PRINT_MSG("UBX-CFG-RATE\n");
        ubx_msg_cfg_rate(session, &buf[UBX_PREFIX_LEN], data_len);
        break;

    case UBX_ESF_ALG:
        mask = ubx_msg_esf_alg(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_ESF_INS:
        mask = ubx_msg_esf_ins(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_ESF_MEAS:
        mask = ubx_msg_esf_meas(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_ESF_RAW:
        mask = ubx_msg_esf_raw(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_ESF_STATUS:
        mask = ubx_msg_esf_status(session, &buf[UBX_PREFIX_LEN], data_len);
        break;

    case UBX_HNR_ATT:
        mask = ubx_msg_hnr_att(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_HNR_INS:
        mask = ubx_msg_hnr_ins(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_HNR_PVT:
        mask = ubx_msg_hnr_pvt(session, &buf[UBX_PREFIX_LEN], data_len);
        break;

    case UBX_INF_DEBUG:
        FALLTHROUGH
    case UBX_INF_ERROR:
        FALLTHROUGH
    case UBX_INF_NOTICE:
        FALLTHROUGH
    case UBX_INF_TEST:
        FALLTHROUGH
    case UBX_INF_USER:
        FALLTHROUGH
    case UBX_INF_WARNING:
        mask = ubx_msg_inf(session, buf, data_len);
        break;

    case UBX_LOG_BATCH:
        PRINT_MSG("UBX-LOG-BATCH\n");
        mask = ubx_msg_log_batch(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_LOG_INFO:
        PRINT_MSG("UBX-LOG-INFO\n");
        mask = ubx_msg_log_info(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_LOG_RETRIEVEPOS:
        PRINT_MSG("UBX-LOG-RETRIEVEPOS\n");
        mask = ubx_msg_log_retrievepos(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_LOG_RETRIEVEPOSEXTRA:
        PRINT_MSG("UBX-LOG-RETRIEVEPOSEXTRA\n");
        mask = ubx_msg_log_retrieveposextra(session, &buf[UBX_PREFIX_LEN],
                                            data_len);
        break;
    case UBX_LOG_RETRIEVESTRING:
        break;

    case UBX_MON_BATCH:
        PRINT_MSG("UBX-MON-BATCH\n");
        break;
    case UBX_MON_EXCEPT:
        PRINT_MSG("UBX-MON-EXCEPT\n");
        break;
    case UBX_MON_GNSS:
        PRINT_MSG("UBX-MON-GNSS\n");
        break;
    case UBX_MON_HW:
        PRINT_MSG("UBX-MON-HW\n");
        break;
    case UBX_MON_HW2:
        PRINT_MSG("UBX-MON-HW2\n");
        break;
    case UBX_MON_HW3:
        PRINT_MSG("UBX-MON-HW3\n");
        break;
    case UBX_MON_IO:
        PRINT_MSG("UBX-MON-IO\n");
        break;
    case UBX_MON_IPC:
        PRINT_MSG("UBX-MON-IPC\n");
        break;
    case UBX_MON_MSGPP:
        PRINT_MSG("UBX-MON-MSGPP\n");
        break;
    case UBX_MON_PATCH:
        PRINT_MSG("UBX-MON-PATCH\n");
        break;
    case UBX_MON_RF:
        PRINT_MSG("UBX-MON-RF\n");
        break;
    case UBX_MON_RXBUF:
        PRINT_MSG("UBX-MON-RXBUF\n");
        ubx_msg_mon_rxbuf(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_MON_RXR:
        PRINT_MSG("UBX-MON-RXR\n");
        break;
    case UBX_MON_SCHED:
        PRINT_MSG("UBX-MON-SCHED\n");
        break;
    case UBX_MON_SMGR:
        PRINT_MSG("UBX-MON-SMGR\n");
        break;
    case UBX_MON_SPAN:
        PRINT_MSG("UBX-MON-SPAN\n");
        break;
    case UBX_MON_TXBUF:
        PRINT_MSG("UBX-MON-TXBUF\n");
        ubx_msg_mon_txbuf(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_MON_USB:
        PRINT_MSG("UBX-MON-USB\n");
        break;
    case UBX_MON_VER:
        PRINT_MSG("UBX-MON-VER\n");
        mask = ubx_msg_mon_ver(session, &buf[UBX_PREFIX_LEN], data_len);
        break;

    case UBX_NAV_AOPSTATUS:
        PRINT_MSG("UBX-NAV-AOPSTATUS\n");
        break;
    case UBX_NAV_ATT:
        PRINT_MSG("UBX-NAV-ATT\n");
        break;
    case UBX_NAV_CLOCK:
        PRINT_MSG("UBX-NAV-CLOCK\n");
        // mask = ubx_msg_nav_clock(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_DGPS:
        PRINT_MSG("UBX-NAV-DGPS\n");

        mask = ubx_msg_nav_dgps(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_DOP:
        PRINT_MSG("UBX-NAV_DOP\n");
        // DOP seems to be the last NAV sent in a cycle, unless NAV-EOE
        mask = ubx_msg_nav_dop(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_EELL:
    PRINT_MSG("UBX-NAV_EELL\n");
        mask = ubx_msg_nav_eell(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_EKFSTATUS:
        PRINT_MSG("UBX-NAV-EKFSTATUS\n");
        break;
    case UBX_NAV_EOE:
        PRINT_MSG("UBX-NAV_EOE\n");

        mask = ubx_msg_nav_eoe(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_GEOFENCE:
        PRINT_MSG("UBX-NAV-GEOFENCE\n");
        break;
    case UBX_NAV_HPPOSECEF:
        PRINT_MSG("UBX-NAV-HPPOSECEF\n");
        mask = ubx_msg_nav_hpposecef(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_HPPOSLLH:
        PRINT_MSG("UBX-NAV-HPPOSLLH\n");
        mask = ubx_msg_nav_hpposllh(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_ODO:
        PRINT_MSG("UBX-NAV-ODO\n");
        break;
    case UBX_NAV_ORB:
        PRINT_MSG("UBX-NAV-ORB\n");
        break;
    case UBX_NAV_POSECEF:
    PRINT_MSG("UBX-RXM-POSECEF\n");
        mask = ubx_msg_nav_posecef(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_POSLLH:
        PRINT_MSG("UBX-NAV-POSLLH\n");
        mask = ubx_msg_nav_posllh(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_POSUTM:
        PRINT_MSG("UBX-NAV-POSUTM\n");
        break;
    case UBX_NAV_PVT:
        PRINT_MSG("UBX-NAV-PVT\n");
        mask = ubx_msg_nav_pvt(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_RELPOSNED:
        PRINT_MSG("UBX-NAV-RELPOSNED\n");
        mask = ubx_msg_nav_relposned(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_RESETODO:
        PRINT_MSG("UBX-NAV-RESETODO\n");
        break;
    case UBX_NAV_SAT:
        PRINT_MSG("UBX-NAV-SAT\n");
        mask = ubx_msg_nav_sat(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_SBAS:
        PRINT_MSG("UBX-NAV-SBAS\n");
        // mask = ubx_msg_nav_sbas(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_SIG:
        PRINT_MSG("UBX-NAV-SIG\n");
        break;
    case UBX_NAV_SOL:
        /* UBX-NAV-SOL deprecated in u-blox 6, gone in u-blox 9 and 10.
         * Use UBX-NAV-PVT instead */
        PRINT_MSG("UBX-NAV-SOL\n");
        mask = ubx_msg_nav_sol(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_STATUS:
    PRINT_MSG("UBX-RXM-STATUS\n");
        mask = ubx_msg_nav_status(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_SVIN:
    PRINT_MSG("UBX-RXM-SVIN\n");
        mask = ubx_msg_tim_svin(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_SVINFO:
        // UBX-NAV-SVINFO deprecated, use UBX-NAV-SAT instead
        PRINT_MSG("UBX-NAV-SVINFO\n");
        mask = ubx_msg_nav_svinfo(session, &buf[UBX_PREFIX_LEN], data_len);

        break;
    case UBX_NAV_TIMEBDS:
        PRINT_MSG("UBX-NAV-TIMEBDS\n");
        break;
    case UBX_NAV_TIMEGAL:
        mask = ubx_msg_nav_timegal(session, &buf[UBX_PREFIX_LEN], data_len);
        PRINT_MSG("UBX-NAV-TIMEGAL\n");
        break;
    case UBX_NAV_TIMEGLO:
        PRINT_MSG("UBX-NAV-TIMEGLO\n");
        break;
    case UBX_NAV_TIMEGPS:
    PRINT_MSG("UBX-NAV-TIMEGPS\n");
        mask = ubx_msg_nav_timegps(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_TIMELS:
        mask = ubx_msg_nav_timels(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_TIMEQZSS:
        PRINT_MSG("UBX-NAV-TIMEQZSS\n");
        break;
    case UBX_NAV_TIMEUTC:
        mask = ubx_msg_nav_timeutc(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_VELECEF:
        PRINT_MSG("UBX-NAV-VELECEF\n");
        mask = ubx_msg_nav_velecef(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_NAV_VELNED:
        PRINT_MSG("UBX-NAV-VELNED\n");
        mask = ubx_msg_nav_velned(session, &buf[UBX_PREFIX_LEN], data_len);
        break;

    case UBX_MGA_ACK:
        PRINT_MSG("UBX-MGA-ACK\n");
        break;
    case UBX_MGA_DBD:
        PRINT_MSG("UBX-MGA-DBD\n");
        break;

    case UBX_RXM_ALM:
        PRINT_MSG("UBX-RXM-ALM\n");
        break;
    case UBX_RXM_EPH:
        PRINT_MSG("UBX-RXM-EPH\n");
        break;
    case UBX_RXM_IMES:
        PRINT_MSG("UBX-RXM-IMES\n");
        break;
    case UBX_RXM_MEASX:
        PRINT_MSG("UBX-RXM-MEASX\n");
        break;
    case UBX_RXM_PMREQ:
        PRINT_MSG("UBX-RXM-PMREQ\n");
        break;
    case UBX_RXM_POSREQ:
        PRINT_MSG("UBX-RXM-POSREQ\n");
        break;
    case UBX_RXM_RAW:
        PRINT_MSG("UBX-RXM-RAW\n");
        break;
    case UBX_RXM_RAWX:
        PRINT_MSG("UBX-RXM-RAWX\n");
        mask = ubx_msg_rxm_rawx(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_RXM_RLM:
        PRINT_MSG("UBX-RXM-RLM\n");
        break;
    case UBX_RXM_RTCM:
        PRINT_MSG("UBX-RXM-RTCM\n");
        break;
    case UBX_RXM_SFRB:
    PRINT_MSG("UBX-RXM-SFRB\n");
        mask = ubx_msg_rxm_sfrb(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_RXM_SFRBX:
    PRINT_MSG("UBX-RXM-SFRBX\n");
        mask = ubx_msg_rxm_sfrbx(session, &buf[UBX_PREFIX_LEN], data_len);
        PRINT_MSG("UBX-RXM-SIGN\n");
        break;
    case UBX_RXM_SVSI:
    PRINT_MSG("UBX-RXM-SVSI\n");
        // Gone in u-blox 10, use UBX-NAV-ORB instead
        mask = ubx_msg_rxm_svsi(session, &buf[UBX_PREFIX_LEN], data_len);
        break;

    // undocumented
    // case UBX_SEC_SESSID:
    //      PRINT_MSG("UBX-SEC-SESSID\n");
    //     break;
    case UBX_SEC_SIGN:
        PRINT_MSG("UBX_SEC_SIGN\n");
        break;
    case UBX_SEC_UNIQID:
        PRINT_MSG("UBX_SEC_UNIQID\n");
        break;

    case UBX_TIM_DOSC:
        PRINT_MSG("UBX-TIM-DOSC\n");
        break;
    case UBX_TIM_FCHG:
        PRINT_MSG("UBX-TIM-FCHG\n");
        break;
    case UBX_TIM_HOC:
        PRINT_MSG("UBX-TIM-HOC\n");
        break;
    case UBX_TIM_SMEAS:
        PRINT_MSG("UBX-TIM-SMEAS\n");
        break;
    case UBX_TIM_SVIN:
        PRINT_MSG("UBX-TIM-SVIN\n");
        break;
    case UBX_TIM_TM:
        PRINT_MSG("UBX-TIM-TM\n");
        break;
    case UBX_TIM_TM2:
        PRINT_MSG("UBX-TIM-TM2\n");
        break;
    case UBX_TIM_TP:
        mask = ubx_msg_tim_tp(session, &buf[UBX_PREFIX_LEN], data_len);
        break;
    case UBX_TIM_TOS:
        PRINT_MSG("UBX-TIM-TOS\n");
        break;
    case UBX_TIM_VCOCAL:
        PRINT_MSG("UBX-TIM-VCOCAL\n");
        break;
    case UBX_TIM_VRFY:
        PRINT_MSG("UBX-TIM-VRFY\n");
        break;

    default:
        PRINT_MSG("No clue what happened - unkn packet\n");
    }

    // // iTOW drives the cycle start/end detection
    // // iTOW is in ms, can go forward or backward
    // if (-1 < session->iTOW ) {
    //     int64_t iTOW_diff;

    //     // this sentence has a (maybe good) time
    //     // end of cycle ?
    //     if (session->driver.ubx.end_msgid == msgid) {
    //         // got known cycle ender.  Assume end of cycle, report it
    //         mask |= REPORT_IS;
    //     }

    //     // start of cycle?  Start can equal end if only one message per epoch
    //     // u-blox iTOW can have ms jitter in the same epoch!
    //     iTOW_diff = session->driver.ubx.last_iTOW - session->iTOW ;
    //     if (10 < llabs(iTOW_diff)) {
    //         // time changed more than 10 ms (100 Hz), cycle start

    //         if (session->driver.ubx.end_msgid !=
    //             session->driver.ubx.last_msgid) {
    //             // new cycle ender
    //         }
    //         session->driver.ubx.last_iTOW = session->iTOW ;
    //         mask |= CLEAR_IS;;
    //     }

    //     session->driver.ubx.last_msgid = msgid;
    //     // FIXME: last_time never used...
    //     session->driver.ubx.last_time = session->gpsdata.fix.time;
    // }

    return mask | ONLINE_SET;
}
