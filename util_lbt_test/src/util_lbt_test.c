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
/* --- PRIVATE MACROS & CONSTANTS ------------------------------------------- */

#define ARRAY_SIZE(a)   (sizeof(a) / sizeof((a)[0]))
#define MSG(args...)    fprintf(stderr, args) /* message that is destined to the user */

#define DEFAULT_SX127X_RSSI_OFFSET -1

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
    printf("Available options:\n");
    printf(" -h print this help\n");
    printf(" -f <float> frequency in MHz of the first LBT channel\n");
    printf(" -o <int>   offset in dB to be applied to the SX127x RSSI [-128..127]\n");
    printf(" -r <int>   target RSSI: signal strength target used to detect if the channel is clear or not [-128..0]\n");
    printf(" -s <uint>  scan time in µs for all 8 LBT channels [128,5000]\n");
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv)
{
    int i;
    int xi = 0;

    /* in/out variables */
    double f1 = 0.0;
    uint32_t f_init = 0; /* in Hz */
    uint32_t f_start = 0; /* in Hz */
    uint16_t loop_cnt = 0;
    int8_t rssi_target_dBm = -80;
    uint16_t scan_time_us = 128;
    uint32_t timestamp;
    uint8_t rssi_value;
    int8_t rssi_offset = DEFAULT_SX127X_RSSI_OFFSET;
    int32_t val, val2;
    int channel;
    uint32_t freq_offset;

    /* parse command line options */
    while ((i = getopt (argc, argv, "h:f:s:r:o:")) != -1) {
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
            case 's':
                i = sscanf(optarg, "%i", &xi);
                if ((i != 1) || ((xi != 128) && (xi != 5000))) {
                    MSG("ERROR: scan_time_us must be 128 or 5000 \n");
                    usage();
                    return EXIT_FAILURE;
                } else {
                    scan_time_us = xi;
                }
                break;
            case 'r':
                i = sscanf(optarg, "%i", &xi);
                if ((i != 1) || ((xi < -128) && (xi > 0))) {
                    MSG("ERROR: rssi_target must be b/w -128 & 0 \n");
                    usage();
                    return EXIT_FAILURE;
                } else {
                    rssi_target_dBm = xi;
                }
                break;
            case 'o': /* -o <int>  SX127x RSSI offset [-128..127] */
                i = sscanf(optarg, "%i", &xi);
                if((i != 1) || (xi < -128) || (xi > 127)) {
                    MSG("ERROR: rssi_offset must be b/w -128 & 127\n");
                    usage();
                    return EXIT_FAILURE;
                } else {
                    rssi_offset = (int8_t)xi;
                }
                break;
            default:
                MSG("ERROR: argument parsing use -h option for help\n");
                usage();
                return EXIT_FAILURE;
        }
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
    i = lgw_connect(false, LGW_DEFAULT_NOTCH_FREQ);
    if (i != LGW_REG_SUCCESS) {
        MSG("ERROR: lgw_connect() did not return SUCCESS\n");
        return EXIT_FAILURE;
    }

    /* Check if FPGA supports LBT */
    lgw_fpga_reg_r(LGW_FPGA_FEATURE, &val);
    if (TAKE_N_BITS_FROM((uint8_t)val, 2, 1) != true) {
        MSG("ERROR: LBT is not supported (0x%x)\n", (uint8_t)val);
        return EXIT_FAILURE;
    }

    /* Get FPGA lowest frequency for LBT channels */
    lgw_fpga_reg_r(LGW_FPGA_LBT_INITIAL_FREQ, &val);
    switch (val) {
        case 0:
            f_init = 915000000;
            break;
        case 1:
            f_init = 863000000;
            break;
        default:
            MSG("ERROR: LBT start frequency %d is not supported\n", val);
            return EXIT_FAILURE;
    }

    /* Initialize 1st LBT channel freq if not given by user */
    if (f_start == 0) {
        f_start = f_init;
    } else if (f_start < f_init) {
        MSG("ERROR: LBT start frequency %u is not supported (f_init=%u)\n", f_start, f_init);
        return EXIT_FAILURE;
    }
    MSG("FREQ: %u\n", f_start);

    /* Configure SX127x and read few RSSI points */
    lgw_setup_sx127x(f_init, MOD_FSK, LGW_SX127X_RXBW_100K_HZ, rssi_offset); /* 200KHz LBT channels */
    for (i = 0; i < 100; i++) {
        lgw_sx127x_reg_r(0x11, &rssi_value); /* 0x11: RegRssiValue */
        MSG("SX127x RSSI:%i dBm\n", -(rssi_value/2));
        wait_ms(10);
    }

    /* Configure LBT */
    val = -2*(rssi_target_dBm);
    lgw_fpga_reg_w(LGW_FPGA_RSSI_TARGET, val);
    for (i = 0; i < LBT_CHANNEL_FREQ_NB; i++) {
        freq_offset = (f_start - f_init)/100E3 + i*2; /* 200KHz between each channel */
        lgw_fpga_reg_w(LGW_FPGA_LBT_CH0_FREQ_OFFSET+i, (int32_t)freq_offset);
        if (scan_time_us == 5000) { /* configured to 128 by default */
            lgw_fpga_reg_w(LGW_FPGA_LBT_SCAN_TIME_CH0+i, 1);
        }
    }

    lgw_fpga_reg_r(LGW_FPGA_RSSI_TARGET, &val);
    MSG("RSSI_TARGET = %d\n", val);
    if (val != (-2*rssi_target_dBm)) {
        MSG("ERROR: failed to read back RSSI target register value\n");
        return EXIT_FAILURE;
    }
    for (i = 0; i < LBT_CHANNEL_FREQ_NB; i++) {
        lgw_fpga_reg_r(LGW_FPGA_LBT_CH0_FREQ_OFFSET+i, &val);
        lgw_fpga_reg_r(LGW_FPGA_LBT_SCAN_TIME_CH0+i, &val2);
        MSG("CH_%i: freq=%u (offset=%i), scan_time=%u (%i)\n", i, (uint32_t)((val*100E3)+f_init), val, (val2==1)?5000:128, val2);
    }
    lgw_fpga_reg_r(LGW_FPGA_VERSION, &val);
    MSG("FPGA VERSION = %d\n", val);

    /* Enable LBT FSM */
    lgw_fpga_reg_w(LGW_FPGA_CTRL_FEATURE_START, 1);

    /* Start test */
    while ((quit_sig != 1) && (exit_sig != 1)) {
        MSG("~~~~\n");
        for (channel = 0; channel < LBT_CHANNEL_FREQ_NB; channel++) {
            /* Select LBT channel */
            lgw_fpga_reg_w(LGW_FPGA_LBT_TIMESTAMP_SELECT_CH, channel);

            /* Get last instant when the selected channel was free */
            lgw_fpga_reg_r(LGW_FPGA_LBT_TIMESTAMP_CH, &val);
            timestamp = (uint32_t)(val & 0x0000FFFF) * 256; /* 16bits (1LSB = 256µs) */
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

