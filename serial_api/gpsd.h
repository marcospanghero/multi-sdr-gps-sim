/* gpsd.h -- fundamental types and structures for the gpsd library
 *
 * Nothing in this file should be used by any client.  So safe to change
 * anything here without (directly) affecting the API or ABI.
 *
 * This file is Copyright 2017 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */

#ifndef _GPSD_H_
#define _GPSD_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <termios.h>
#ifdef HAVE_WINSOCK2_H
#include <winsock2.h>   /* for fd_set */
#else                   /* !HAVE_WINSOCK2_H */
#include <sys/select.h> /* for fd_set */
#endif                  /* !HAVE_WINSOCK2_H */
#include <time.h>       /* for time_t */

#include "gps.h"
#include "timespec.h"

    /*
     * Constants for the VERSION response
     * 3.1: Base JSON version
     * 3.2: Added POLL command and response
     * 3.3: AIS app_id split into DAC and FID
     * 3.4: Timestamps change from seconds since Unix epoch to ISO8601.
     * 3.5: POLL subobject name changes: fixes -> tpv, skyview -> sky.
     *      DEVICE::activated becomes ISO8601 rather than real.
     * 3.6  VERSION, WATCH, and DEVICES from slave gpsds get "remote" attribute.
     * 3.7  PPS message added to repertoire. SDDBT water depth reported as
     *      negative altitude with Mode 3 set.
     * 3.8  AIS course member becomes float in scaled mode (bug fix).
     * 3.9  split24 flag added. Controlled-vocabulary fields are now always
     *      dumped in both numeric and string form, with the string being the
     *      value of a synthesized additional attribute with "_text" appended.
     *      (Thus, the 'scaled' flag no longer affects display of these fields.)
     *      PPS drift message ships nsec rather than msec.
     * 3.10 The obsolete tag field has been dropped from JSON.
     * 3.11 A precision field, log2 of the time source jitter, has been added
     *      to the PPS report.  See ntpshm.h for more details.
     * 3.12 OSC message added to repertoire.
     * 3.13 gnssid:svid added to SAT
     *      time added to ATT
     * 3.19 Added RAW message class.
     *      Add cfg_stage and cfg_step, for initialization
     *      Add oldfix2 for better oldfix
     *      Make subtype longer
     *      Add ubx.protver, ubx.last_msgid and more to gps_device_t.ubx
     *      MAX_PACKET_LENGTH 516 -> 9216
     *      Add stuff to gps_device_t.nmea for NMEA 4.1
     * 3.20
     *      Remove TIMEHINT_ENABLE.  It only worked when enabled.
     *      Remove NTP_ENABLE and NTPSHM_ENABLE.  It only worked when enabled.
     *      Change gps_type_t.min_cycle from double to timespec_t
     *      Change gps_device_t.last_time from double to timespec_t
     *      Change gps_lexer_t.start_time from timestamp_t to timespec_t
     *      Change gps_context_t.gps_tow from double to timespec_t
     *      Change gps_device_t.sor from timestamp_t to timespec_t
     *      Change gps_device_t.this_frac_time, last_frac_time to timespec_t
     *      Change nmea.subseconds from double to timespec_t
     *      Remove gpsd_gpstime_resolve()
     *      Changed order of gpsd_log() arguments.  Add GPSD_LOG().
     *      Remove gps_device_t.back_to_nmea.
     *      Add fixed_port_speed, fixed_port_framing to gps_context_t.
     *      change tsip.superpkt from bool to int.
     *      Add tsip  .machine_id, .hardware_code, .last_tow, last_chan_seen
     *      Split gps_device_t.subtype into subtype and subtype1
     * 3.21
     *      GPSD_PROTO_*_VERSION moved to gpsd_config.h
     *      Add gps_context_t.passive
     *      Add gps_context_t.batteryRTC
     * 3.21.1
     *      Add gps_device_t.ubx.last_protver
     *      Add gps_device_t last_word_gal and last_svid3_gal
     *      Add timespec ts_startCurrentBaud to gps_device_t
     *      ntrip_conn_* to NTRIP_CONN_*
     * 3.23.2~rc1
     *      add ntrip_parse_url(), ntrip_close()
     *      add host, stream_time, to ntrip_stream_t
     *      add shm_clock_unit and shm_pps_unit to device_t
     *      add VALID_UNIT()
     *      remove shm_clock and shm_pps from device_t
     *      add pkt_time to lexer_t
     *      add netlib_connectsock1()
     *      add nmea.gsx_more to gps_device_t
     *      add TSIPv1 stuff
     */

#define JSON_DATE_MAX 24 /* ISO8601 timestamp with 2 decimal places */

// be sure to change BUILD_LEAPSECONDS as needed.
#define BUILD_CENTURY 2000
#define BUILD_LEAPSECONDS 18

#ifndef DEFAULT_GPSD_SOCKET
#define DEFAULT_GPSD_SOCKET RUNDIR "/gpsd.sock"
#endif

/* Some internal capabilities depend on which drivers we're compiling. */
#if !defined(AIVDM_ENABLE) && defined(NMEA2000_ENABLE)
#define AIVDM_ENABLE
#endif

#ifdef EARTHMATE_ENABLE
#define ZODIAC_ENABLE
#endif

#if defined(EVERMORE_ENABLE) ||   \
    defined(GARMIN_ENABLE) ||     \
    defined(GEOSTAR_ENABLE) ||    \
    defined(GREIS_ENABLE) ||      \
    defined(ITRAX_ENABLE) ||      \
    defined(NAVCOM_ENABLE) ||     \
    defined(NMEA2000_ENABLE) ||   \
    defined(ONCORE_ENABLE) ||     \
    defined(SIRF_ENABLE) ||       \
    defined(SUPERSTAR2_ENABLE) || \
    defined(TSIP_ENABLE) ||       \
    defined(UBLOX_ENABLE) ||      \
    defined(ZODIAC_ENABLE)
#define BINARY_ENABLE
#endif

#if defined(TRIPMATE_ENABLE) || defined(BINARY_ENABLE)
#define NON_NMEA0183_ENABLE
#endif
#ifdef ISYNC_ENABLE
#define STASH_ENABLE
#endif

// First, declarations for the packet layer...

/*
 * NMEA 3.01, Section 5.3 says the max sentence length shall be
 * 82 chars, including the leading $ and terminating \r\n.
 *
 * Some receivers (TN-200, GSW 2.3.2) emit oversized sentences.
 * The Trimble BX-960 receiver emits a 91-character GGA message.
 * The Skytraq S2525F8 which emits a 100-character PSTI message.
 * The current hog champion is the Skytraq PX1172RH which emits
 * a 103-character PSTI message.
 */
#define NMEA_MAX 110                    // max length of NMEA sentence
#define NMEA_MAX_FLD 100                // max fields in an NMEA sentence
#define NMEA_BIG_BUF (2 * NMEA_MAX + 1) // longer than longest NMEA sentence

    // a few bits of ISGPS magic
    enum isgpsstat_t
    {
        ISGPS_NO_SYNC,
        ISGPS_SYNC,
        ISGPS_SKIP,
        ISGPS_MESSAGE,
    };

#define RTCM_MAX (RTCM2_WORDS_MAX * sizeof(isgps30bits_t))
/*
 * RTCM3 is more variable length than RTCM 2.  In the absence of
 * reading a specification, follow BKG's lead in assuming 2048.
 *
 * In theory, the limit is
 *   1 octet preamble
 *   2 octets payload length (first 6 bits reserved) --> max payload length 1023 octets
 *   0-1023 octects payload
 *   3 octets CRC
 *   1029 octets maximum
 */
#define RTCM3_MAX 2048

/*
 * The packet buffers need to be as long than the longest packet we
 * expect to see in any protocol, because we have to be able to hold
 * an entire packet for checksumming...
 * First we thought it had to be big enough for a SiRF Measured Tracker
 * Data packet (188 bytes). Then it had to be big enough for a UBX SVINFO
 * packet (206 bytes). Now it turns out that a couple of ITALK messages are
 * over 512 bytes. I know we like verbose output, but this is ridiculous.
 * Whoopie! The u-blox 8 UBX-RXM-RAWX packet is 8214 byte long!
 */
#define MAX_PACKET_LENGTH 9216 /* 4 + 16 + (256 * 32) + 2 + fudge */

/*
 * UTC of second 0 of week 0 of the first rollover period of GPS time.
 * Used to compute UTC from GPS time. Also, the threshold value
 * under which system clock times are considered unreliable. Often,
 * embedded systems come up thinking it's early 1970 and the system
 * clock will report small positive values until the clock is set.  By
 * choosing this as the cutoff, we'll never reject historical GPS logs
 * that are actually valid.
 */
#define GPS_EPOCH ((time_t)315964800) /* 6 Jan 1980 00:00:00 UTC */
#define GAL_EPOCH ((time_t)935272800) /* 22 Aug 1999 00:00:00 UTC */

/* time constant */
#define SECS_PER_DAY ((time_t)(60 * 60 * 24)) /* seconds per day */
#define SECS_PER_WEEK (7 * SECS_PER_DAY)      /* seconds per week */
#define GPS_ROLLOVER (1024 * SECS_PER_WEEK)   /* rollover period */

    struct gpsd_errout_t
    {
        int debug;                    /* lexer debug level */
        void (*report)(const char *); /* reporting hook for lexer errors */
        char *label;
    };

    struct gps_lexer_t
    {
        // packet-getter internals
        int type;
#define BAD_PACKET -1
#define COMMENT_PACKET 0
#define NMEA_PACKET 1
#define AIVDM_PACKET 2
#define GARMINTXT_PACKET 3
#define MAX_TEXTUAL_TYPE 3 // increment this as necessary
#define SIRF_PACKET 4
#define ZODIAC_PACKET 5
#define TSIP_PACKET 6
#define EVERMORE_PACKET 7
#define ITALK_PACKET 8
#define GARMIN_PACKET 9
#define NAVCOM_PACKET 10
#define UBX_PACKET 11
#define SUPERSTAR2_PACKET 12
#define ONCORE_PACKET 13
#define GEOSTAR_PACKET 14
#define NMEA2000_PACKET 15
#define GREIS_PACKET 16
#define MAX_GPSPACKET_TYPE 16 // increment this as necessary
#define RTCM2_PACKET 17
#define RTCM3_PACKET 18
#define JSON_PACKET 19
#define PACKET_TYPES 20 // increment this as necessary
#define SKY_PACKET 21
#define TEXTUAL_PACKET_TYPE(n) ((((n) >= NMEA_PACKET) && ((n) <= MAX_TEXTUAL_TYPE)) || (n) == JSON_PACKET)
#define GPS_PACKET_TYPE(n) (((n) >= NMEA_PACKET) && ((n) <= MAX_GPSPACKET_TYPE))
#define LOSSLESS_PACKET_TYPE(n) (((n) >= RTCM2_PACKET) && ((n) <= RTCM3_PACKET))
#define PACKET_TYPEMASK(n) (1 << (n))
#define GPS_TYPEMASK (((2 << (MAX_GPSPACKET_TYPE + 1)) - 1) & ~PACKET_TYPEMASK(COMMENT_PACKET))
        unsigned int state;
        size_t length;
        unsigned char inbuffer[MAX_PACKET_LENGTH * 2 + 1];
        size_t inbuflen;
        unsigned char *inbufptr;
        // outbuffer needs to be able to hold 4 GPGSV records at once
        unsigned char outbuffer[MAX_PACKET_LENGTH * 2 + 1];
        size_t outbuflen;
        unsigned long char_counter;  // count characters processed
        unsigned long retry_counter; // count sniff retries
        unsigned counter;            // packets since last driver switch
        struct gpsd_errout_t errout; // how to report errors
        timespec_t start_time;       // time of first input, sort of
        timespec_t pkt_time;         // time of last packet parsed
        unsigned long start_char;    // char counter at first input
        /*
         * ISGPS200 decoding context.
         *
         * This is not conditionalized on RTCM104_ENABLE because we need to
         * be able to build gpsdecode even when RTCM support is not
         * configured in the daemon.
         */
        struct
        {
            bool locked;
            int curr_offset;
            isgps30bits_t curr_word;
            unsigned int bufindex;
            /*
             * Only these should be referenced from elsewhere, and only when
             * RTCM_MESSAGE has just been returned.
             */
            isgps30bits_t buf[RTCM2_WORDS_MAX]; // packet data
            size_t buflen;                      // packet length in bytes
        } isgps;
        unsigned int json_depth;
        unsigned int json_after;
#ifdef STASH_ENABLE
        unsigned char stashbuffer[MAX_PACKET_LENGTH];
        size_t stashbuflen;
#endif // STASH_ENABLE
    };

    extern void lexer_init(struct gps_lexer_t *);
    extern void packet_reset(struct gps_lexer_t *);
    extern void packet_pushback(struct gps_lexer_t *);
    extern void packet_parse(struct gps_lexer_t *);
    extern ssize_t packet_get(int, struct gps_lexer_t *);
    extern int packet_sniff(struct gps_lexer_t *);
#define packet_buffered_input(lexer) ((lexer)->inbuffer + (lexer)->inbuflen - (lexer)->inbufptr)

// Next, declarations for the core library...

// factors for converting among confidence interval units
#define CEP50_SIGMA 1.18
#define DRMS_SIGMA 1.414
#define CEP95_SIGMA 2.45

// this is where we choose the confidence level to use in reports
#define GPSD_CONFIDENCE CEP95_SIGMA
//#define NTPSHMSEGS      (MAX_DEVICES * 2)       // number of NTP SHM segments
#define NTP_MIN_FIXES 3 // # fixes to wait for before shipping NTP time

#define AIVDM_CHANNELS 2 // A, B

    struct gps_device_t;

    struct gps_context_t
    {
        int valid;                   // member validity flags
#define LEAP_SECOND_VALID 0x01       // we have or don't need correction
#define GPS_TIME_VALID 0x02          // GPS week/tow is valid
#define CENTURY_VALID 0x04           // have received ZDA or 4-digit yearupdate
        struct gpsd_errout_t errout; // debug verbosity level and hook
        bool readonly;               // if true, never write to device
        bool passive;                // if true, never autoconfigure device
        // if true, remove fix gate to time, for some RTC backed receivers.
        // DANGEROUS
        bool batteryRTC;
        speed_t fixed_port_speed;   // Fixed port speed, if non-zero
        char fixed_port_framing[4]; // Fixed port framing, if non-blank
        /* DGPS status */
        int fixcnt; // count of good fixes seen
        /* timekeeping */
        time_t start_time;       // local time of daemon startup
        int leap_seconds;        // Unix secs to UTC (GPS-UTC offset)
        unsigned short gps_week; // GPS week, usually 10 bits
        timespec_t gps_tow;      // GPS time of week
        int century;             // for NMEA-only devices without ZDA
        int rollovers;           // rollovers since start of run
        int leap_notify;         // notification state from subframe
#define LEAP_NOWARNING 0x0       // normal, no leap second warning
#define LEAP_ADDSECOND 0x1       // last minute of day has 60 seconds
#define LEAP_DELSECOND 0x2       // last minute of day has 59 seconds
#define LEAP_NOTINSYNC 0x3       // overload, clock is free running
        /* we need the volatile here to tell the C compiler not to
         * 'optimize' as 'dead code' the writes to SHM */
        // volatile struct shmTime *shmTime[NTPSHMSEGS];
        // bool shmTimeInuse[NTPSHMSEGS];
        void (*pps_hook)(struct gps_device_t *, int, int, struct timedelta_t *);
#ifdef SHM_EXPORT_ENABLE
        /* we don't want the compiler to treat writes to shmexport as dead code,
         * and we don't want them reordered either */
        volatile void *shmexport;
        int shmid; // ID of SHM  (for later IPC_RMID)
#endif
        ssize_t (*serial_write)(struct gps_device_t *,
                                const char *buf, const size_t len);

        /*
         * We add here some of the Galileo stuff as well. Now, this does not make too much sense as the context is called gps_context, but I guess a more accurate way of calling it would be gnss_context as most receivers are multiconstellation nowadays
         */
        unsigned short gal_week; // GAL week, usually 10 bits
        timespec_t gal_tow;      // GAL time of week
        int gal_leap_seconds;    // Unix sec to UTC (GAL-UTC offset)
    };

    // state for resolving interleaved Type 24 packets
    struct ais_type24a_t
    {
        unsigned int mmsi;
        char shipname[AIS_SHIPNAME_MAXLEN + 1];
    };
#define MAX_TYPE24_INTERLEAVE 8 /* max number of queued type 24s */
    struct ais_type24_queue_t
    {
        struct ais_type24a_t ships[MAX_TYPE24_INTERLEAVE];
        int index;
    };

    /* state for resolving AIVDM decodes */
    struct aivdm_context_t
    {
        /* hold context for decoding AIDVM packet sequences */
        int decoded_frags; // for tracking AIDVM parts in a multipart sequence
        unsigned char bits[2048];
        size_t bitlen; /* how many valid bits */
        struct ais_type24_queue_t type24_queue;
    };

#define MODE_NMEA 0
#define MODE_BINARY 1

    typedef enum
    {
        ANY,
        GPS,
        RTCM2,
        RTCM3,
        AIS
    } gnss_type;
    typedef enum
    {
        event_wakeup,
        event_triggermatch,
        event_identified,
        event_configure,
        event_driver_switch,
        event_deactivate,
        event_reactivate,
    } event_t;

#define INTERNAL_SET(n) ((gps_mask_t)(1llu << (SET_HIGH_BIT + (n))))
#define RAW_IS INTERNAL_SET(1)         /* raw pseudoranges available */
#define USED_IS INTERNAL_SET(2)        /* sat-used count available */
#define DRIVER_IS INTERNAL_SET(3)      /* driver type identified */
#define CLEAR_IS INTERNAL_SET(4)       /* starts a reporting cycle */
#define REPORT_IS INTERNAL_SET(5)      /* ends a reporting cycle */
#define NODATA_IS INTERNAL_SET(6)      /* no data read from fd */
#define NTPTIME_IS INTERNAL_SET(7)     // precision time is available
#define PERR_IS INTERNAL_SET(8)        /* PDOP set */
#define PASSTHROUGH_IS INTERNAL_SET(9) /* passthrough mode */
#define EOF_IS INTERNAL_SET(10)        /* synthetic EOF */
#define GOODTIME_IS INTERNAL_SET(11)   /* time good even if no pos fix */
#define DATA_IS ~(ONLINE_SET | PACKET_SET | CLEAR_IS | REPORT_IS)

    typedef unsigned int driver_mask_t;
#define DRIVER_NOFLAGS 0x00000000u
#define DRIVER_STICKY 0x00000001u

/*
 * True if a device type is non-null and has control methods.
 */
#define CONTROLLABLE(dp) (((dp) != NULL) && \
                          ((dp)->speed_switcher != NULL || (dp)->mode_switcher != NULL || (dp)->rate_switcher != NULL))

/*
 * True if a driver selection of it should be sticky.
 */
#define STICKY(dp) ((dp) != NULL && ((dp)->flags & DRIVER_STICKY) != 0)

    struct gps_type_t
    {
        /* GPS method table, describes how to talk to a particular GPS type */
        char *type_name;
        int packet_type;
        driver_mask_t flags; /* reserved for expansion */
        char *trigger;
        int channels;
        bool (*probe_detect)(struct gps_device_t *session);
        ssize_t (*get_packet)(struct gps_device_t *session);
        gps_mask_t (*parse_packet)(struct gps_device_t *session);
        ssize_t (*rtcm_writer)(struct gps_device_t *session,
                               const char *rtcmbuf, size_t rtcmbytes);
        void (*init_query)(struct gps_device_t *session);
        void (*event_hook)(struct gps_device_t *session, event_t event);
        bool (*speed_switcher)(struct gps_device_t *session,
                               speed_t speed, char parity, int stopbits);
        void (*mode_switcher)(struct gps_device_t *session, int mode);
        bool (*rate_switcher)(struct gps_device_t *session, double rate);
        timespec_t min_cycle;
        ssize_t (*control_send)(struct gps_device_t *session,
                                char *buf, size_t buflen);
        double (*time_offset)(struct gps_device_t *session);
    };

    /*
     * Each input source has an associated type.  This is currently used in two
     * ways:
     *
     * (1) To determine if we require that gpsd be the only process opening a
     * device.  We make an exception for PTYs because the master side has to be
     * opened by test code.
     *
     * (2) To determine whether it's safe to send wakeup strings.  These are
     * required on some unusual RS-232 devices (such as the TNT compass and
     * Thales/Ashtech GPSes) but should not be shipped to unidentified USB
     * or Bluetooth devices as we don't even know in advance those are GPSes;
     * they might not cope well.
     *
     * Where it says "case detected but not used" it means that we can identify
     * a source type but no behavior is yet contingent on it.  A "discoverable"
     * device is one for which there is discoverable metadata such as a
     * vendor/product ID.
     *
     * We should never see a block device; that would indicate a serious error
     * in command-line usage or the hotplug system.
     *
     * Order matters, SOURCE_BLOCKDEV >= is used to check for read-only
     */
    typedef enum
    {
        SOURCE_UNKNOWN,
        SOURCE_BLOCKDEV,  // block devices can't be GPS sources
        SOURCE_RS232,     // potential GPS source, not discoverable
        SOURCE_USB,       // potential GPS source, discoverable
        SOURCE_BLUETOOTH, // potential GPS source, discoverable
        SOURCE_CAN,       // potential GPS source, fixed CAN format
        SOURCE_PTY,       // PTY: we don't require exclusive access
        SOURCE_TCP,       // TCP/IP stream: case detected but not used
        SOURCE_UDP,       // UDP stream: case detected but not used
        SOURCE_GPSD,      // Remote gpsd instance over TCP/IP
        SOURCE_PPS,       // PPS-only device, such as /dev/ppsN
        SOURCE_PIPE,      // Unix FIFO; don't use blocking I/O
        SOURCE_ACM,       // potential GPS source, discoverable, no speed
    } sourcetype_t;

    /*
     * Each input source also has an associated service type.
     */
    typedef enum
    {
        SERVICE_UNKNOWN = 0,
        SERVICE_SENSOR, // local, or network, sensor
        SERVICE_DGPSIP, // dgpsip://
        SERVICE_NTRIP,  // ntrip://
    } servicetype_t;

    /*
     * Private state information about an NTRIP stream.
     */
    struct ntrip_stream_t
    {
        timespec_t stream_time; // time stream was last opened or closed
        char mountpoint[101];   // stream name
        char credentials[128];  // username:password
        char authStr[128];      // HTTP Authorization: line
        char url[256];          // full url: http://user@pass:host:port/mp
        char host[256];         // hostname or IP
        char port[32];          // in my /etc/services 16 was the longest
        bool set;               // found and set
        enum ntrip_fmt
        {
            FMT_UNKNOWN = 0,
            FMT_CMRP, // CMR+, dunno what it is.  ORGN uses it
            FMT_RTCM2,
            FMT_RTCM2_0,
            FMT_RTCM2_1,
            FMT_RTCM2_2,
            FMT_RTCM2_3,
            FMT_RTCM3_0,
            FMT_RTCM3_1,
            FMT_RTCM3_2,
            FMT_RTCM3_3,
        } format;
        int carrier;
        double latitude;
        double longitude;
        int nmea;
        enum
        {
            CMP_ENC_NONE,
            CMP_ENC_UNKNOWN
        } compr_encryp;
        enum
        {
            AUTH_NONE,
            AUTH_BASIC,
            AUTH_DIGEST,
            AUTH_UNKNOWN
        } authentication;
        int fee;
        int bitrate;
    };

    struct gps_device_t
    {
        int64_t iTOW;

        /* session object, encapsulates all global state */
        struct gps_data_t gpsdata;
        const struct gps_type_t *device_type;
        unsigned int driver_index;       /* numeric index of current driver */
        unsigned int drivers_identified; /* bitmask; what drivers have we seen? */
        unsigned int cfg_stage;          /* configuration stage counter */
        unsigned int cfg_step;           /* configuration step counter */
        const struct gps_type_t *last_controller;
        struct gps_context_t *context;
        sourcetype_t sourcetype;
        servicetype_t servicetype;
        int mode;
        struct termios ttyset, ttyset_old;
        unsigned int baudindex;
        int saved_baud;
        struct gps_lexer_t lexer;
        int badcount;
        int subframe_count;
        /* firmware version or subtype ID, 96 too small for ZED-F9 */
        char subtype[128];
        char subtype1[128];
        time_t opentime; // FIXME: change to timespec_t
        time_t releasetime;
        bool zerokill;
        time_t reawake;
        timespec_t sor; // time start of this reporting cycle
        // time start of current autobaud hunt.
        // maybe should be in struct device_t, but should not be client visible.
        timespec_t ts_startCurrentBaud;
        unsigned long chars; // characters in the cycle
        bool ship_to_ntpd;
        int shm_clock_unit;
        int shm_pps_unit;
        int chronyfd; /* for talking to chrony */
        /*
         * msgbuf needs to hold the hex decode of inbuffer
         * so msgbuf must be 2x the size of inbuffer
         */
        char msgbuf[MAX_PACKET_LENGTH * 4 + 1]; /* command message buffer for sends */
        size_t msgbuflen;
        int observed;             /* which packet type`s have we seen? */
        bool cycle_end_reliable;  /* does driver signal REPORT_MASK */
        int fixcnt;               /* count of fixes from this device */
        int last_word_gal;        // last subframe word from Galileo
        int last_svid3_gal;       // last SVID3 from Galileo
        struct gps_fix_t newdata; /* where drivers put their data */
        struct gps_fix_t lastfix; /* not quite yet ready for oldfix */
        struct gps_fix_t oldfix;  /* previous fix for error modeling */

        /*
         * The rest of this structure is driver-specific private storage.
         * Only put a driver's scratch storage in here if it is never
         * implemented on the same device that supports any mode already
         * in this union; otherwise bad things might happen after a device
         * mode switch.
         */
        struct
        {
            struct
            {
                /* FIXME: last_time set but never used? */
                timespec_t last_time; /* time of last_msgid */
                /* iTOW, and last_iTOW, in ms, used for cycle end detect. */
                int64_t iTOW;
                int64_t last_iTOW;
                unsigned int end_msgid;  /* cycle ender class/ID */
                unsigned int last_msgid; /* last class/ID */
                unsigned char port_id;
                unsigned char sbas_in_use;
                unsigned char protver;      // u-blox protocol version
                unsigned char last_protver; // last protocol version
            } ubx;
            struct
            {
                /* ISGPS200 decoding */
                bool locked;
                int curr_offset;
                isgps30bits_t curr_word;
                isgps30bits_t buf[RTCM2_WORDS_MAX];
                unsigned int bufindex;
            } isgps;
        } driver;
    };

/*
 * These are used where a file descriptor of 0 or greater indicates open device.
 */
#define UNALLOCATED_FD -1  /* this slot is available for reallocation */
#define PLACEHOLDING_FD -2 /* this slot *not* available for reallocation */

/* logging levels */
#define LOG_ERROR -1 /* errors, display always */
#define LOG_SHOUT 0  /* not an error but we should always see it */
#define LOG_WARN 1   /* not errors but may indicate a problem */
#define LOG_CLIENT 2 /* log JSON reports to clients */
#define LOG_INF 3    /* key informative messages */
#define LOG_PROG 4   /* progress messages */
#define LOG_IO 5     /* IO to and from devices */
#define LOG_DATA 6   /* log data management messages */
#define LOG_SPIN 7   /* logging for catching spin bugs */
#define LOG_RAW 8    /* raw low-level I/O */
#define LOG_RAW1 9   // rawer
#define LOG_RAW2 10  // rawest

#define ISGPS_ERRLEVEL_BASE LOG_RAW

#define IS_HIGHEST_BIT(v, m) (v & ~((m << 1) - 1)) == 0

    /* driver helper functions */
    extern void isgps_init(struct gps_lexer_t *);
    enum isgpsstat_t isgps_decode(struct gps_lexer_t *,
                                  bool (*preamble_match)(isgps30bits_t *),
                                  bool (*length_check)(struct gps_lexer_t *),
                                  size_t,
                                  unsigned int);
    extern unsigned int isgps_parity(isgps30bits_t);
    extern void isgps_output_magnavox(const isgps30bits_t *, unsigned int, FILE *);

    extern enum isgpsstat_t rtcm2_decode(struct gps_lexer_t *, unsigned int);
    extern void json_rtcm2_dump(const struct rtcm2_t *,
                                const char *, char[], size_t);
    extern void rtcm2_unpack(struct gps_device_t *, struct rtcm2_t *, char *);
    extern void json_rtcm3_dump(const struct rtcm3_t *,
                                const char *, char[], size_t);
    extern void rtcm3_unpack(const struct gps_context_t *,
                             struct rtcm3_t *, char *);

    /* here are the available GPS drivers */
    extern const struct gps_type_t **gpsd_drivers;

    /* gpsd library internal prototypes */
    extern gps_mask_t generic_parse_input(struct gps_device_t *);
    extern ssize_t generic_get(struct gps_device_t *);

    extern gps_mask_t nmea_parse(char *, struct gps_device_t *);
    extern ssize_t nmea_write(struct gps_device_t *, char *, size_t);
    extern ssize_t nmea_send(struct gps_device_t *, const char *, ...);
    extern void nmea_add_checksum(char *);

    extern gps_mask_t sirf_parse(struct gps_device_t *, unsigned char *, size_t);
    extern gps_mask_t evermore_parse(struct gps_device_t *, unsigned char *,
                                     size_t);
    extern gps_mask_t navcom_parse(struct gps_device_t *, unsigned char *, size_t);
    extern gps_mask_t garmin_ser_parse(struct gps_device_t *);
    extern gps_mask_t garmintxt_parse(struct gps_device_t *);
    extern gps_mask_t aivdm_parse(struct gps_device_t *);

    extern bool netgnss_uri_check(char *);
    extern int netgnss_uri_open(struct gps_device_t *, char *);
    extern void netgnss_report(struct gps_context_t *,
                               struct gps_device_t *,
                               struct gps_device_t *);
    extern void netgnss_autoconnect(struct gps_context_t *, double, double);

    extern socket_t dgpsip_open(struct gps_device_t *, const char *);
    extern void dgpsip_report(struct gps_context_t *,
                              struct gps_device_t *,
                              struct gps_device_t *);
    extern void dgpsip_autoconnect(struct gps_context_t *,
                                   double, double, const char *);
    extern int ntrip_open(struct gps_device_t *, char *);
    extern void ntrip_report(struct gps_context_t *,
                             struct gps_device_t *,
                             struct gps_device_t *);

    extern bool gpsd_set_raw(struct gps_device_t *);
    extern int gpsd_serial_isatty(const struct gps_device_t *);
    extern int gpsd_serial_open(struct gps_device_t *);
    extern void gpsd_tty_init(struct gps_device_t *);
    extern ssize_t gpsd_serial_write(struct gps_device_t *,
                                     const char *, const size_t);
    extern bool gpsd_next_hunt_setting(struct gps_device_t *);
    extern int gpsd_switch_driver(struct gps_device_t *, char *);
    extern void gpsd_set_speed(struct gps_device_t *, speed_t, char, unsigned int);
    extern int gpsd_get_speed(const struct gps_device_t *);
    extern int gpsd_get_speed_old(const struct gps_device_t *);
    extern int gpsd_get_stopbits(const struct gps_device_t *);
    extern char gpsd_get_parity(const struct gps_device_t *);
    extern void gpsd_assert_sync(struct gps_device_t *);
    extern void gpsd_close(struct gps_device_t *);

    extern ssize_t gpsd_write(struct gps_device_t *, const char *, const size_t);

    extern void gpsd_time_init(struct gps_context_t *, time_t);
    extern void gpsd_set_century(struct gps_device_t *);
    extern timespec_t gpsd_gpstime_resolv(struct gps_device_t *, unsigned,
                                          timespec_t);
    extern timespec_t gpsd_galtime_resolv(struct gps_device_t *, unsigned,
                                          timespec_t);
    extern timespec_t gpsd_utc_resolve(struct gps_device_t *);
    extern void gpsd_century_update(struct gps_device_t *, int);

    extern gps_mask_t gpsd_interpret_subframe(struct gps_device_t *,
                                              unsigned int,
                                              unsigned int,
                                              uint32_t[]);
    extern gps_mask_t gpsd_interpret_subframe_raw(struct gps_device_t *,
                                                  unsigned int,
                                                  unsigned int,
                                                  uint32_t[],
                                                  unsigned int);
    extern const char *gpsd_hexdump(char *, size_t, char *, size_t);
    extern const char *gpsd_packetdump(char *, size_t, char *, size_t);
    extern const char *gpsd_prettydump(struct gps_device_t *);
#ifdef __cplusplus
    extern "C"
    {
#endif
        extern int gpsd_hexpack(const char *, char *, size_t);
#ifdef __cplusplus
    }
#endif
    extern ssize_t hex_escapes(char *, const char *);
    extern void gpsd_position_fix_dump(struct gps_device_t *,
                                       char[], size_t);
    extern void gpsd_clear_data(struct gps_device_t *);
    // FIXME: use in libgps, so should not be in gpsd.h!
    // deprecated Oct 2021, use netlib_connectsock1() instead
    extern socket_t netlib_connectsock(int, const char *, const char *,
                                       const char *);
    extern socket_t netlib_connectsock1(int, const char *, const char *,
                                        const char *, int,
                                        char *, size_t);
    // end FIXME
    extern socket_t netlib_localsocket(const char *, int);
    extern const char *netlib_errstr(const int);
    extern char *netlib_sock2ip(socket_t);

    extern void nmea_tpv_dump(struct gps_device_t *, char[], size_t);
    extern void nmea_sky_dump(struct gps_device_t *, char[], size_t);
    extern void nmea_subframe_dump(struct gps_device_t *, char[], size_t);
    extern void nmea_ais_dump(struct gps_device_t *, char[], size_t);
    extern unsigned int ais_binary_encode(struct ais_t *ais, unsigned char *bits,
                                          int flag);

    extern void ntrip_close(struct gps_device_t *);
    extern int ntrip_parse_url(const struct gpsd_errout_t *,
                               struct ntrip_stream_t *, const char *);
    extern void ntp_latch(struct gps_device_t *device, struct timedelta_t *td);
    extern void ntpshm_context_init(struct gps_context_t *);
    extern void ntpshm_session_init(struct gps_device_t *);
    extern void ntpshm_put(struct gps_device_t *, int unit, int precision,
                           struct timedelta_t *);
    extern void ntpshm_link_deactivate(struct gps_device_t *);
    extern void ntpshm_link_activate(struct gps_device_t *);

    extern void errout_reset(struct gpsd_errout_t *errout);

    extern void gpsd_acquire_reporting_lock(void);
    extern void gpsd_release_reporting_lock(void);

    extern gps_mask_t ecef_to_wgs84fix(struct gps_fix_t *,
                                       double, double, double,
                                       double, double, double);
    extern void clear_dop(struct dop_t *);

/* shmexport.c */
#define GPSD_SHM_KEY 0x47505344 /* "GPSD" */
    struct shmexport_t
    {
        int bookend1;
        struct gps_data_t gpsdata;
        int bookend2;
    };
    extern bool shm_acquire(struct gps_context_t *);
    extern void shm_release(struct gps_context_t *);
    extern void shm_update(struct gps_context_t *, struct gps_data_t *);

/* dbusexport.c */
#if defined(DBUS_EXPORT_ENABLE)
    int initialize_dbus_connection(void);
    void send_dbus_fix(struct gps_device_t *channel);
#endif /* defined(DBUS_EXPORT_ENABLE) */

    /* a BSD transplant */
    int b64_ntop(unsigned char const *src, size_t srclength, char *target,
                 size_t targsize);

    /* application interface */
    extern void gps_context_init(struct gps_context_t *context,
                                 const char *label);
    extern void gpsd_init(struct gps_device_t *,
                          struct gps_context_t *,
                          const char *);
    extern void gpsd_clear(struct gps_device_t *);
    extern int parse_uri_dest(char *s, char **host, char **service, char **device);
    extern int gpsd_open(struct gps_device_t *);
#define O_CONTINUE 0
#define O_PROBEONLY 1
#define O_OPTIMIZE 2
    extern int gpsd_activate(struct gps_device_t *, const int);
    extern void gpsd_deactivate(struct gps_device_t *);

#define AWAIT_TIMEOUT 2
#define AWAIT_GOT_INPUT 1
#define AWAIT_NOT_READY 0
#define AWAIT_FAILED -1
    extern int gpsd_await_data(fd_set *,
                               fd_set *,
                               int,
                               fd_set *,
                               struct gpsd_errout_t *,
                               timespec_t);
    extern gps_mask_t gpsd_poll(struct gps_device_t *);
#define DEVICE_EOF -3
#define DEVICE_ERROR -2
#define DEVICE_UNREADY -1
#define DEVICE_READY 1
#define DEVICE_UNCHANGED 0
    extern int gpsd_multipoll(const bool,
                              struct gps_device_t *,
                              void (*)(struct gps_device_t *, gps_mask_t),
                              float reawake_time);
    extern void gpsd_wrap(struct gps_device_t *);
    extern bool gpsd_add_device(const char *device_name, bool flag_nowait);
    const char *gps_maskdump(gps_mask_t set);

    /* exceptional driver methods */
    extern bool ubx_write(struct gps_device_t *, unsigned int, unsigned int,
                          const unsigned char *, size_t);
    extern bool ais_binary_decode(const struct gpsd_errout_t *errout,
                                  struct ais_t *ais,
                                  const unsigned char *, size_t,
                                  struct ais_type24_queue_t *);

    void gpsd_labeled_report(const int, const int,
                             const char *, const char *, va_list);

    char *visibilize(char *outbuf, size_t outlen, const char *inbuf, size_t inlen);
    // do not call gpsd_log() directly, use GPSD_LOG() to save a lot of cpu time

    // STUFF WE ADD WHEN WE CUT PIECES OUT
    size_t strlcpy(char *dst, const char *src, size_t siz);
    size_t strlcat(char *dst, const char *src, size_t siz);
    size_t strnlen(const char *s, size_t maxlen);

    /*
     * GPSD_LOG() is the new one debug logger to rule them all.
     *
     * The calling convention is not attractive:
     *     GPSD_LOG(debuglevel, (fmt, ...));
     *     GPSD_LOG(2, ("this will appear on stdout if debug >= %d\n", 2));
     *
     * This saves significant pushing, popping, hexification, etc. when
     * the debug level does not require it.
     */

#define NITEMS(x) ((int)(sizeof(x) / sizeof(x[0]) + COMPILE_CHECK_IS_ARRAY(x)))

/*
 * C99 requires NAN to be defined if the implementation supports quiet
 * NANs.  At one point, it seems Solaris did not define NAN; it is not
 * clear if this is still true.
 */
#ifndef NAN
#define NAN (0.0f / 0.0f)
#endif

#if !defined(HAVE_CFMAKERAW)
    /*
     * POSIX does not specify cfmakeraw, but it is pretty common.  We
     * provide an implementation in serial.c for systems that lack it.
     */
    void cfmakeraw(struct termios *);
#endif /* !defined(HAVE_CFMAKERAW) */

#define DEVICEHOOKPATH "/" SYSCONFDIR "/gpsd/device-hook"

#ifdef __cplusplus
}
#endif

#endif /* _GPSD_H_ */
// Local variables:
// mode: c
// end:
// vim: set expandtab shiftwidth=4
