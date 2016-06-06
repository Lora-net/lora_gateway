/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
    Minimum test program for the loragw_gps 'library'

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Sylvain Miermont
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

/* fix an issue between POSIX and C99 */
#if __STDC_VERSION__ >= 199901L
    #define _XOPEN_SOURCE 600
#else
    #define _XOPEN_SOURCE 500
#endif

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */
#include <stdio.h>      /* printf */
#include <string.h>     /* memset */
#include <signal.h>     /* sigaction */
#include <stdlib.h>     /* exit */
#include <unistd.h>     /* read */

#include "loragw_hal.h"
#include "loragw_gps.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

static void sig_handler(int sigio);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

static void sig_handler(int sigio) {
    if (sigio == SIGQUIT) {
        quit_sig = 1;;
    } else if ((sigio == SIGINT) || (sigio == SIGTERM)) {
        exit_sig = 1;
    }
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main()
{
    struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */

    int i;
    char tmp_str[80];

    /* serial variables */
    char serial_buff[128]; /* buffer to receive GPS data */
    ssize_t nb_char;
    int gps_tty_dev; /* file descriptor to the serial port of the GNSS module */

    /* NMEA variables */
    enum gps_msg latest_msg; /* keep track of latest NMEA message parsed */

    /* variables for PPM pulse GPS synchronization */
    uint32_t ppm_tstamp;
    struct timespec ppm_utc;
    struct tref ppm_ref;

    /* variables for timestamp <-> UTC conversions */
    uint32_t x, z;
    struct timespec y;

    /* configure signal handling */
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigact.sa_handler = sig_handler;
    sigaction(SIGQUIT, &sigact, NULL);
    sigaction(SIGINT, &sigact, NULL);
    sigaction(SIGTERM, &sigact, NULL);

    /* Intro message and library information */
    printf("Beginning of test for loragw_gps.c\n");
    printf("*** Library version information ***\n%s\n***\n", lgw_version_info());

    /* Open and configure GPS */
    i = lgw_gps_enable("/dev/ttyAMA0", NULL, 0, &gps_tty_dev);
    if (i != LGW_GPS_SUCCESS) {
        printf("ERROR: IMPOSSIBLE TO ENABLE GPS\n");
        exit(EXIT_FAILURE);
    }

    /* start concentrator */
    lgw_start();

    /* initialize some variables before loop */
    memset(serial_buff, 0, sizeof serial_buff);
    memset(&ppm_ref, 0, sizeof ppm_ref);

    /* loop until user action */
    while ((quit_sig != 1) && (exit_sig != 1)) {
        /* blocking canonical read on serial port */
        nb_char = read(gps_tty_dev, serial_buff, sizeof(serial_buff)-1);
        if (nb_char <= 0) {
            printf("Warning: read() returned value <= 0\n");
            continue;
        } else {
            serial_buff[nb_char] = 0;
        }

        /* parse the received NMEA */
        latest_msg = lgw_parse_nmea(serial_buff, sizeof(serial_buff));

        if (latest_msg == NMEA_RMC) {

            printf("\n~~ RMC NMEA sentence, triggering synchronization attempt ~~\n");

            /* get UTC time for synchronization */
            i = lgw_gps_get(&ppm_utc, NULL, NULL);
            if (i != LGW_GPS_SUCCESS) {
                printf("    No valid reference UTC time available, synchronization impossible.\n");
                continue;
            }
            /* get timestamp for synchronization */
            i = lgw_get_trigcnt(&ppm_tstamp);
            if (i != LGW_HAL_SUCCESS) {
                printf("    Failed to read timestamp, synchronization impossible.\n");
                continue;
            }
            /* try to update synchronize time reference with the new UTC & timestamp */
            i = lgw_gps_sync(&ppm_ref, ppm_tstamp, ppm_utc);
            if (i != LGW_GPS_SUCCESS) {
                printf("    Synchronization error.\n");
                continue;
            }
            /* display result */
            printf("    * Synchronization successful *\n");
            strftime(tmp_str, sizeof(tmp_str), "%F %T", gmtime(&(ppm_ref.utc.tv_sec)));
            printf("    UTC reference time: %s.%09ldZ\n", tmp_str, ppm_ref.utc.tv_nsec);
            printf("    Internal counter reference value: %u\n", ppm_ref.count_us);
            printf("    Clock error: %.9f\n", ppm_ref.xtal_err);

            x = ppm_tstamp + 500000;
            printf("    * Test of timestamp counter <-> UTC value conversion *\n");
            printf("    Test value: %u\n", x);
            lgw_cnt2utc(ppm_ref, x, &y);
            strftime(tmp_str, sizeof(tmp_str), "%F %T", gmtime(&(y.tv_sec)));
            printf("    Conversion to UTC: %s.%09ldZ\n", tmp_str, y.tv_nsec);
            lgw_utc2cnt(ppm_ref, y, &z);
            printf("    Converted back: %u\n", z);
        }
    }

    /* clean up before leaving */
    if (exit_sig == 1) {
        lgw_stop();
    }

    printf("\nEnd of test for loragw_gps.c\n");
    exit(EXIT_SUCCESS);
}

/* --- EOF ------------------------------------------------------------------ */
