/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
    SPI stress test

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
#include <stdio.h>      /* printf fprintf sprintf fopen fputs */

#include <signal.h>     /* sigaction */
#include <unistd.h>     /* getopt access */
#include <stdlib.h>     /* rand */

#include "loragw_reg.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a)    (sizeof(a) / sizeof((a)[0]))
#define MSG(args...)    fprintf(stderr, args) /* message that is destined to the user */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define VERS                    103
#define READS_WHEN_ERROR        16 /* number of times a read is repeated if there is a read error */
#define BUFF_SIZE               1024 /* maximum number of bytes that we can write in sx1301 RX data buffer */
#define DEFAULT_TX_NOTCH_FREQ   129E3

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
    MSG( "Available options:\n");
    MSG( " -h print this help\n");
    MSG( " -t <int> specify which test you want to run (1-4)\n");
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv)
{
    int i;
    int xi = 0;

    /* application option */
    int test_number = 1;
    int cycle_number = 0;
    int repeats_per_cycle = 1000;
    bool error = false;

    /* in/out variables */
    int32_t test_value;
    int32_t read_value;
    int32_t rb1, rb2, rb3; /* interstitial readbacks, to flush buffers if needed */

    /* data buffer */
    int32_t test_addr;
    uint8_t test_buff[BUFF_SIZE];
    uint8_t read_buff[BUFF_SIZE];

    /* parse command line options */
    while ((i = getopt (argc, argv, "ht:")) != -1) {
        switch (i) {
            case 'h':
                usage();
                return EXIT_FAILURE;
                break;

            case 't':
                i = sscanf(optarg, "%i", &xi);
                if ((i != 1) || (xi < 1) || (xi > 4)) {
                    MSG("ERROR: invalid test number\n");
                    return EXIT_FAILURE;
                } else {
                    test_number = xi;
                }
                break;

            default:
                MSG("ERROR: argument parsing use -h option for help\n");
                usage();
                return EXIT_FAILURE;
        }
    }
    MSG("INFO: Starting LoRa concentrator SPI stress-test number %i\n", test_number);

    /* configure signal handling */
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigact.sa_handler = sig_handler;
    sigaction(SIGQUIT, &sigact, NULL);
    sigaction(SIGINT, &sigact, NULL);
    sigaction(SIGTERM, &sigact, NULL);

    /* start SPI link */
    i = lgw_connect(false, DEFAULT_TX_NOTCH_FREQ);
    if (i != LGW_REG_SUCCESS) {
        MSG("ERROR: lgw_connect() did not return SUCCESS");
        return EXIT_FAILURE;
    }

    if (test_number == 1) {
        /* single 8b register R/W stress test */
        while ((quit_sig != 1) && (exit_sig != 1)) {
            printf("Cycle %i > ", cycle_number);
            for (i=0; i<repeats_per_cycle; ++i) {
                test_value = (rand() % 256);
                lgw_reg_w(LGW_IMPLICIT_PAYLOAD_LENGHT, test_value);
                lgw_reg_r(LGW_IMPLICIT_PAYLOAD_LENGHT, &read_value);
                if (read_value != test_value) {
                    error = true;
                    break;
                }
            }
            if (error) {
                printf("error during the %ith iteration: write 0x%02X, read 0x%02X\n", i+1, test_value, read_value);
                printf("Repeat read of target register:");
                for (i=0; i<READS_WHEN_ERROR; ++i) {
                    lgw_reg_r(LGW_IMPLICIT_PAYLOAD_LENGHT, &read_value);
                    printf(" 0x%02X", read_value);
                }
                printf("\n");
                return EXIT_FAILURE;
            } else {
                printf("did %i R/W on an 8 bits reg with no error\n", repeats_per_cycle);
                ++cycle_number;
            }
        }
    } else if (test_number == 2) {
        /* single 8b register R/W with interstitial VERSION check stress test */
        while ((quit_sig != 1) && (exit_sig != 1)) {
            printf("Cycle %i > ", cycle_number);
            for (i=0; i<repeats_per_cycle; ++i) {
                test_value = (rand() % 256);
                lgw_reg_r(LGW_VERSION, &rb1);
                lgw_reg_w(LGW_IMPLICIT_PAYLOAD_LENGHT, test_value);
                lgw_reg_r(LGW_VERSION, &rb2);
                lgw_reg_r(LGW_IMPLICIT_PAYLOAD_LENGHT, &read_value);
                lgw_reg_r(LGW_VERSION, &rb3);
                if ((rb1 != VERS) || (rb2 != VERS) || (rb3 != VERS) || (read_value != test_value)) {
                    error = true;
                    break;
                }
            }
            if (error) {
                printf("error during the %ith iteration: write %02X, read %02X, version (%i, %i, %i)\n", i+1, test_value, read_value, rb1, rb2, rb3);
                printf("Repeat read of target register:");
                for (i=0; i<READS_WHEN_ERROR; ++i) {
                    lgw_reg_r(LGW_IMPLICIT_PAYLOAD_LENGHT, &read_value);
                    printf(" 0x%02X", read_value);
                }
                printf("\n");
                return EXIT_FAILURE;
            } else {
                printf("did %i R/W on an 8 bits reg with no error\n", repeats_per_cycle);
                ++cycle_number;
            }
        }
    } else if (test_number == 3) {
        /* 32b register R/W stress test */
        while ((quit_sig != 1) && (exit_sig != 1)) {
            printf("Cycle %i > ", cycle_number);
            for (i=0; i<repeats_per_cycle; ++i) {
                test_value = (rand() & 0x0000FFFF);
                test_value += (int32_t)(rand() & 0x0000FFFF) << 16;
                lgw_reg_w(LGW_FSK_REF_PATTERN_LSB, test_value);
                lgw_reg_r(LGW_FSK_REF_PATTERN_LSB, &read_value);
                if (read_value != test_value) {
                    error = true;
                    break;
                }
            }
            if (error) {
                printf("error during the %ith iteration: write 0x%08X, read 0x%08X\n", i+1, test_value, read_value);
                printf("Repeat read of target register:");
                for (i=0; i<READS_WHEN_ERROR; ++i) {
                    lgw_reg_r(LGW_FSK_REF_PATTERN_LSB, &read_value);
                    printf(" 0x%08X", read_value);
                }
                printf("\n");
                return EXIT_FAILURE;
            } else {
                printf("did %i R/W on a 32 bits reg with no error\n", repeats_per_cycle);
                ++cycle_number;
            }
        }
    } else if (test_number == 4) {
        /* databuffer R/W stress test */
        while ((quit_sig != 1) && (exit_sig != 1)) {
            for (i=0; i<BUFF_SIZE; ++i) {
                test_buff[i] = rand() & 0xFF;
            }
            printf("Cycle %i > ", cycle_number);
            test_addr = rand() & 0xFFFF;
            lgw_reg_w(LGW_RX_DATA_BUF_ADDR, test_addr); /* write at random offset in memory */
            lgw_reg_wb(LGW_RX_DATA_BUF_DATA, test_buff, BUFF_SIZE);
            lgw_reg_w(LGW_RX_DATA_BUF_ADDR, test_addr); /* go back to start of segment */
            lgw_reg_rb(LGW_RX_DATA_BUF_DATA, read_buff, BUFF_SIZE);
            for (i=0; ((i<BUFF_SIZE) && (test_buff[i] == read_buff[i])); ++i);
            if (i != BUFF_SIZE) {
                printf("error during the buffer comparison\n");
                printf("Written values:\n");
                for (i=0; i<BUFF_SIZE; ++i) {
                    printf(" %02X ", test_buff[i]);
                    if (i%16 == 15) printf("\n");
                }
                printf("\n");
                printf("Read values:\n");
                for (i=0; i<BUFF_SIZE; ++i) {
                    printf(" %02X ", read_buff[i]);
                    if (i%16 == 15) printf("\n");
                }
                printf("\n");
                lgw_reg_w(LGW_RX_DATA_BUF_ADDR, test_addr); /* go back to start of segment */
                lgw_reg_rb(LGW_RX_DATA_BUF_DATA, read_buff, BUFF_SIZE);
                printf("Re-read values:\n");
                for (i=0; i<BUFF_SIZE; ++i) {
                    printf(" %02X ", read_buff[i]);
                    if (i%16 == 15) printf("\n");
                }
                printf("\n");
                return EXIT_FAILURE;
            } else {
                printf("did a %i-byte R/W on a data buffer with no error\n", BUFF_SIZE);
                ++cycle_number;
            }
        }
    } else {
        MSG("ERROR: invalid test number");
        usage();
    }

    /* close SPI link */
    i = lgw_disconnect();
    if (i != LGW_REG_SUCCESS) {
        MSG("ERROR: lgw_disconnect() did not return SUCCESS");
        return EXIT_FAILURE;
    }

    MSG("INFO: Exiting LoRa concentrator SPI stress-test program\n");
    return EXIT_SUCCESS;
}

/* --- EOF ------------------------------------------------------------------ */

