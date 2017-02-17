/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
    Library of functions to manage a GNSS module (typically GPS) for accurate
    timestamping of packets and synchronisation of gateways.
    A limited set of module brands/models are supported.

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Michael Coracin
*/


#ifndef _LORAGW_GPS_H
#define _LORAGW_GPS_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#define _GNU_SOURCE
#include <stdint.h>     /* C99 types */
#include <time.h>       /* time library */
#include <termios.h>    /* speed_t */
#include <unistd.h>     /* ssize_t */

#include "config.h"     /* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */

/**
@struct coord_s
@brief Time solution required for timestamp to absolute time conversion
*/
struct tref {
    time_t          systime;    /*!> system time when solution was calculated */
    uint32_t        count_us;   /*!> reference concentrator internal timestamp */
    struct timespec utc;        /*!> reference UTC time (from GPS/NMEA) */
    struct timespec gps;        /*!> reference GPS time (since 01.Jan.1980) */
    double          xtal_err;   /*!> raw clock error (eg. <1 'slow' XTAL) */
};

/**
@struct coord_s
@brief Geodesic coordinates
*/
struct coord_s {
    double  lat;    /*!> latitude [-90,90] (North +, South -) */
    double  lon;    /*!> longitude [-180,180] (East +, West -)*/
    short   alt;    /*!> altitude in meters (WGS 84 geoid ref.) */
};

/**
@enum gps_msg
@brief Type of GPS (and other GNSS) sentences
*/
enum gps_msg {
    UNKNOWN,         /*!> neutral value */
    IGNORED,         /*!> frame was not parsed by the system */
    INVALID,         /*!> system try to parse frame but failed */
    INCOMPLETE,      /*!> frame parsed was missing bytes */
    /* NMEA messages of interest */
    NMEA_RMC,        /*!> Recommended Minimum data (time + date) */
    NMEA_GGA,        /*!> Global positioning system fix data (pos + alt) */
    NMEA_GNS,        /*!> GNSS fix data (pos + alt, sat number) */
    NMEA_ZDA,        /*!> Time and Date */
    /* NMEA message useful for time reference quality assessment */
    NMEA_GBS,        /*!> GNSS Satellite Fault Detection */
    NMEA_GST,        /*!> GNSS Pseudo Range Error Statistics */
    NMEA_GSA,        /*!> GNSS DOP and Active Satellites (sat number) */
    NMEA_GSV,        /*!> GNSS Satellites in View (sat SNR) */
    /* Misc. NMEA messages */
    NMEA_GLL,        /*!> Latitude and longitude, with time fix and status */
    NMEA_TXT,        /*!> Text Transmission */
    NMEA_VTG,        /*!> Course over ground and Ground speed */
    /* uBlox proprietary NMEA messages of interest */
    UBX_NAV_TIMEGPS, /*!> GPS Time Solution */
    UBX_NAV_TIMEUTC  /*!> UTC Time Solution */
};

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

#define LGW_GPS_SUCCESS 0
#define LGW_GPS_ERROR   -1

#define LGW_GPS_MIN_MSG_SIZE      (8)
#define LGW_GPS_UBX_SYNC_CHAR     (0xB5)
#define LGW_GPS_NMEA_SYNC_CHAR    (0x24)

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

/**
@brief Configure a GPS module

@param tty_path path to the TTY connected to the GPS
@param gps_familly parameter (eg. ubx6 for uBlox gen.6)
@param target_brate target baudrate for communication (0 keeps default target baudrate)
@param fd_ptr pointer to a variable to receive file descriptor on GPS tty
@return success if the function was able to connect and configure a GPS module
*/
int lgw_gps_enable(char* tty_path, char* gps_familly, speed_t target_brate, int* fd_ptr);

/**
@brief Restore GPS serial configuration and close serial device

@param fd file descriptor on GPS tty
@return success if the function was able to complete
*/
int lgw_gps_disable(int fd);

/**
@brief Parse messages coming from the GPS system (or other GNSS)

@param serial_buff pointer to the string to be parsed
@param buff_size maximum string lengths for NMEA parsing (incl. null char)
@return type of frame parsed

The RAW NMEA sentences are parsed to a global set of variables shared with the
lgw_gps_get function.
If the lgw_parse_nmea and lgw_gps_get are used in different threads, a mutex
lock must be acquired before calling either function.
*/
enum gps_msg lgw_parse_nmea(const char* serial_buff, int buff_size);

/**
@brief Parse Ublox proprietary messages coming from the GPS system

@param serial_buff pointer to the string to be parsed
@param buff_size maximum string lengths for UBX parsing (incl. null char)
@param msg_size number of bytes parsed as UBX message if found
@return type of frame parsed

The RAW UBX sentences are parsed to a global set of variables shared with the
lgw_gps_get function.
If the lgw_parse_ubx and lgw_gps_get are used in different threads, a mutex
lock must be acquired before calling either function.
*/
enum gps_msg lgw_parse_ubx(const char* serial_buff, size_t buff_size, size_t *msg_size);

/**
@brief Get the GPS solution (space & time) for the concentrator

@param utc pointer to store UTC time, with ns precision (NULL to ignore)
@param gps_time pointer to store GPS time, with ns precision (NULL to ignore)
@param loc pointer to store coordinates (NULL to ignore)
@param err pointer to store coordinates standard deviation (NULL to ignore)
@return success if the chosen elements could be returned

This function read the global variables generated by the NMEA/UBX parsing
functions lgw_parse_nmea/lgw_parse_ubx. It returns time and location data in a
format that is exploitable by other functions in that library sub-module.
If the lgw_parse_nmea/lgw_parse_ubx and lgw_gps_get are used in different
threads, a mutex lock must be acquired before calling either function.
*/
int lgw_gps_get(struct timespec *utc, struct timespec *gps_time, struct coord_s *loc, struct coord_s *err);

/**
@brief Get time and position information from the serial GPS last message received
@param utc UTC time, with ns precision (leap seconds are ignored)
@param gps_time timestamp of last time pulse from the GPS module converted to the UNIX epoch
       (leap seconds are ignored)
@param loc location information
@param err location error estimate if supported
@return success if timestamp was read and time reference could be refreshed

Set systime to 0 in ref to trigger initial synchronization.
*/
int lgw_gps_sync(struct tref *ref, uint32_t count_us, struct timespec utc, struct timespec gps_time);

/**
@brief Convert concentrator timestamp counter value to UTC time

@param ref time reference structure required for time conversion
@param count_us internal timestamp counter of the LoRa concentrator
@param utc pointer to store UTC time, with ns precision (leap seconds ignored)
@return success if the function was able to convert timestamp to UTC

This function is typically used when a packet is received to transform the
internal counter-based timestamp in an absolute timestamp with an accuracy in
the order of a couple microseconds (ns resolution).
*/
int lgw_cnt2utc(struct tref ref, uint32_t count_us, struct timespec* utc);

/**
@brief Convert UTC time to concentrator timestamp counter value

@param ref time reference structure required for time conversion
@param utc UTC time, with ns precision (leap seconds are ignored)
@param count_us pointer to store internal timestamp counter of LoRa concentrator
@return success if the function was able to convert UTC to timestamp

This function is typically used when a packet must be sent at an accurate time
(eg. to send a piggy-back response after receiving a packet from a node) to
transform an absolute UTC time into a matching internal concentrator timestamp.
*/
int lgw_utc2cnt(struct tref ref,struct timespec utc, uint32_t* count_us);

/**
@brief Convert concentrator timestamp counter value to GPS time

@param ref time reference structure required for time conversion
@param count_us internal timestamp counter of the LoRa concentrator
@param gps_time pointer to store GPS time, with ns precision (leap seconds ignored)
@return success if the function was able to convert timestamp to GPS time

This function is typically used when a packet is received to transform the
internal counter-based timestamp in an absolute timestamp with an accuracy in
the order of a millisecond.
*/
int lgw_cnt2gps(struct tref ref, uint32_t count_us, struct timespec* gps_time);

/**
@brief Convert GPS time to concentrator timestamp counter value

@param ref time reference structure required for time conversion
@param gps_time GPS time, with ns precision (leap seconds are ignored)
@param count_us pointer to store internal timestamp counter of LoRa concentrator
@return success if the function was able to convert GPS time to timestamp

This function is typically used when a packet must be sent at an accurate time
(eg. to send a piggy-back response after receiving a packet from a node) to
transform an absolute GPS time into a matching internal concentrator timestamp.
*/
int lgw_gps2cnt(struct tref ref, struct timespec gps_time, uint32_t* count_us);

#endif

/* --- EOF ------------------------------------------------------------------ */
