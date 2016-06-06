/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
    Listen Before Talk basic test application

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Michael Coracin
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
#include <stdio.h>      /* printf fprintf sprintf fopen fputs */

#include <signal.h>     /* sigaction */
#include <unistd.h>     /* getopt access */
#include <stdlib.h>     /* rand */

#include "loragw_aux.h"
#include "loragw_reg.h"
#include "loragw_hal.h"
#include "loragw_radio.h"
#include "loragw_fpga.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a)   (sizeof(a) / sizeof((a)[0]))
#define MSG(args...)    fprintf(stderr, args) /* message that is destined to the user */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES (GLOBAL) ------------------------------------------- */

/* signal handling variables */
struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */
static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

static void sig_handler(int sigio);

void usage (void);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

static void sig_handler(int sigio) {
    if (sigio == SIGQUIT) {
        quit_sig = 1;;
    } else if ((sigio == SIGINT) || (sigio == SIGTERM)) {
        exit_sig = 1;
    }
}

/* describe command line options */
void usage(void) {
    printf( "Available options:\n");
    printf( " -h print this help\n");
    printf( " -f <float> start frequency in MHz\n");
    printf( " -t <uint>  number of read loops [1..32000]\n");
    printf( " -n <uint>  number of spi access to SX127x RSSI instant register [0..255]\n");
    printf( " -s <uint>  spi speed divider [0..255]\n");
    printf( " -p <uint>  pll lock time: delay in 8 µsec step between frequency programming and RX ready [0..255]\n");
    printf( " -r <uint>  target RSSI: signal strength target used to detect if the channel is clear or not [0..255]\n");
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv)
{
    int i;
    int xi = 0;

    /* in/out variables */
    double f1 = 0.0;
    uint32_t f_start = 0; /* in Hz */
    uint16_t loop_cnt = 0;
    uint16_t tempo = 100;
    uint16_t nb_point = 14;
    uint8_t spi_speed_div = 15;
    uint8_t rssi_target = 162;
    uint8_t pll_lock_time = 50;
    uint16_t lsb_start_freq_int;
    uint32_t timestamp;
    uint8_t rssi_value;
    int32_t val;
    int channel;

    /* parse command line options */
    while ((i = getopt (argc, argv, "h:f:t:n:s:p:r:o:")) != -1) {
        switch (i) {
            case 'h':
                usage();
                return EXIT_FAILURE;
                break;

            case 'f':
                i = sscanf(optarg, "%lf", &f1);
                if ((i != 1) || (f1 < 30.0) || (f1 > 3000.0)) {
                    MSG("ERROR: Invalid LBT start frequency\n");
                    usage();
                    return EXIT_FAILURE;
                } else {
                    f_start = (uint32_t)((f1*1e6) + 0.5);/* .5 Hz offset to get rounding instead of truncating */
                }
                break;
            case 't':
                i = sscanf(optarg, "%i", &xi);
                if ((i != 1) || (xi < 1) || (xi > 32000)) {
                    MSG("ERROR: tempo must be b/w 1 & 32000 \n");
                    usage();
                    return EXIT_FAILURE;
                } else {
                    tempo = xi;
                }
                break;
            case 'n':
                i = sscanf(optarg, "%i", &xi);
                if ((i != 1) || (xi < 0) || (xi > 255)) {
                    MSG("ERROR: nb_point must be b/w 0 & 255 \n");
                    usage();
                    return EXIT_FAILURE;
                } else {
                    nb_point = xi;
                }
                break;
            case 's':
                i = sscanf(optarg, "%i", &xi);
                if ((i != 1) || (xi < 0) || (xi > 255)) {
                    MSG("ERROR: spi_speed_div must be b/w 0 & 255 \n");
                    usage();
                    return EXIT_FAILURE;
                } else {
                    spi_speed_div = xi;
                }
                break;
            case 'r':
                i = sscanf(optarg, "%i", &xi);
                if ((i != 1) || (xi < 0) || (xi > 255)) {
                    MSG("ERROR: rssi_target must be b/w 0 & 255 \n");
                    usage();
                    return EXIT_FAILURE;
                } else {
                    rssi_target = xi;
                }
                break;
            case 'p':
                i = sscanf(optarg, "%i", &xi);
                if ((i != 1) || (xi < 0) || (xi > 255)) {
                    MSG("ERROR: pll_lock_time must be b/w 1 & 256 \n");
                    usage();
                    return EXIT_FAILURE;
                } else {
                    pll_lock_time = xi;
                }
                break;
            default:
                MSG("ERROR: argument parsing use -h option for help\n");
                usage();
                return EXIT_FAILURE;
        }
    }

    /* Sanity check */
    if (f_start == 0) {
        MSG("ERROR: LBT start frequency must be set\n");
        usage();
        return EXIT_FAILURE;
    }

    MSG("INFO: Starting LoRa Gateway v1.5 LBT test\n");

    /* configure signal handling */
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigact.sa_handler = sig_handler;
    sigaction(SIGQUIT, &sigact, NULL);
    sigaction(SIGINT, &sigact, NULL);
    sigaction(SIGTERM, &sigact, NULL);

    /* Connect to concentrator */
    i = lgw_connect();
    if (i != LGW_REG_SUCCESS) {
        MSG("ERROR: lgw_connect() did not return SUCCESS\n");
        return EXIT_FAILURE;
    }

    /* Check if FPGA supports LBT */
    lgw_fpga_reg_r(LGW_FPGA_FPGA_FEATURE, &val);
    if (TAKE_N_BITS_FROM((uint8_t)val, 2, 1) != true) {
        printf("ERROR: LBT is not supported (0x%x)\n", (uint8_t)val);
        return EXIT_FAILURE;
    }

    MSG("FREQ: %u\n", f_start);

    /* Configure SX127x and read few RSSI points */
    lgw_setup_sx127x(f_start, MOD_FSK);
    for (i = 0; i < 100; i++) {
        lgw_sx127x_reg_r(0x11, &rssi_value); /* 0x11: RegRssiValue */
        MSG("SX127x RSSI:-%u dBm\n", rssi_value>>1);
        wait_ms(10);
    }

    /* Configure LBT */
    lgw_fpga_reg_w(LGW_FPGA_SPI_MASTER_SPEED_DIVIDER, (int32_t)spi_speed_div);
    lgw_fpga_reg_w(LGW_FPGA_NB_READ_RSSI, (int32_t)nb_point);
    lgw_fpga_reg_w(LGW_FPGA_PLL_LOCK_TIME, (int32_t)pll_lock_time);
    lgw_fpga_reg_w(LGW_FPGA_RSSI_TARGET, (int32_t)rssi_target);
    lsb_start_freq_int = (((uint64_t)f_start<<19)/(uint64_t)32000000);
    lgw_fpga_reg_w(LGW_FPGA_LSB_START_FREQ, (int32_t)lsb_start_freq_int);
    lgw_fpga_reg_w(LGW_FPGA_LBT_TIMESTAMP_NB_CH, (6-1)); /* 6 channels */

    /* Enable LBT FSM */
    lgw_fpga_reg_w(LGW_FPGA_CTRL_FEATURE_START, 1);

    /* Read back LBT config */
    lgw_fpga_reg_r(LGW_FPGA_SPI_MASTER_SPEED_DIVIDER, &val);
    MSG("spi_speed_div = %d\n", val);
    if (val != spi_speed_div) {
        return EXIT_FAILURE;
    }
    lgw_fpga_reg_r(LGW_FPGA_NB_READ_RSSI, &val);
    MSG("nb_point = %d\n", val);
    if (val != nb_point) {
        return EXIT_FAILURE;
    }
    lgw_fpga_reg_r(LGW_FPGA_PLL_LOCK_TIME, &val);
    MSG("PLL_LOCK = %d\n", val);
    if (val != pll_lock_time) {
        return EXIT_FAILURE;
    }
    lgw_fpga_reg_r(LGW_FPGA_RSSI_TARGET, &val);
    MSG("RSSI_TARGET = %d\n", val);
    if (val != rssi_target) {
        return EXIT_FAILURE;
    }
    lgw_fpga_reg_r(LGW_FPGA_VERSION, &val);
    MSG("FPGA VERSION = %d\n", val);

    /* Start test */
    while (((quit_sig != 1) && (exit_sig != 1)) && (loop_cnt != tempo)) {
        /* Get current FPGA timestamp (1MHz) */
        lgw_fpga_reg_r(LGW_FPGA_TIMESTAMP, &val);
        timestamp = (uint32_t)val;
        MSG(" TIMESTAMP = %u\n", timestamp);

        for (channel = 0; channel <= 5; channel++) {
            /* Select LBT channel */
            lgw_fpga_reg_w(LGW_FPGA_LBT_TIMESTAMP_SELECT_CH, channel);

            /* Get last instant when the selected channel was free */
            lgw_fpga_reg_r(LGW_FPGA_LBT_TIMESTAMP_CH, &val);
            timestamp = (uint32_t)(val & 0x00FFFFFF) * 256; /* 24bits (1LSB = 256µs) */
            MSG(" TIMESTAMP_CH%u = %u\n", channel, timestamp);
        }

        loop_cnt += 1;
        wait_ms(400);
    }

    /* close SPI link */
    i = lgw_disconnect();
    if (i != LGW_REG_SUCCESS) {
        MSG("ERROR: lgw_disconnect() did not return SUCCESS\n");
        return EXIT_FAILURE;
    }

    MSG("INFO: Exiting LoRa Gateway v1.5 LBT test successfully\n");
    return EXIT_SUCCESS;
}

/* --- EOF ------------------------------------------------------------------ */

