/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2014 Semtech-Cycleo

Description:
    SX1301 spectral scan

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Michael Coracin
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDENCIES --------------------------------------------------------- */

/* Fix an issue between POSIX and C99 */
#if __STDC_VERSION__ >= 199901L
    #define _XOPEN_SOURCE 600
#else
    #define _XOPEN_SOURCE 500
#endif

#include <stdint.h>     /* C99 types */
#include <stdio.h>      /* NULL printf */
#include <stdlib.h>     /* EXIT atoi */
#include <unistd.h>     /* getopt */
#include <string.h>

#include "loragw_aux.h"
#include "loragw_reg.h"
#include "loragw_hal.h"
#include "loragw_radio.h"
#include "loragw_fpga.h"

/* -------------------------------------------------------------------------- */
/* --- MACROS & CONSTANTS --------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#define DEFAULT_START_FREQ          863000000   /* start frequency, Hz */
#define DEFAULT_STOP_FREQ           870000000   /* stop frequency, Hz */
#define DEFAULT_STEP_FREQ           200000      /* frequency step, Hz */
#define DEFAULT_RSSI_PTS            65535       /* number of RSSI reads */
#define DEFAULT_CHAN_BW             LGW_SX127X_RXBW_62K5_HZ /* channel bandwidth */
#define DEFAULT_LOG_NAME            "rssi_histogram"
#define DEFAULT_SX127X_RSSI_OFFSET  -4

#define RSSI_RANGE                  256

#define MAX_FREQ                    1000000000
#define MIN_FREQ                    800000000
#define MIN_STEP_FREQ               5000

#define FPGA_FEATURE_SPECTRAL_SCAN  1
#define FPGA_FEATURE_LBT            2

/* When FPGA supports LBT, there are few more constraints on above constants */
#define LBT_DEFAULT_RSSI_PTS    129*129 /* number of RSSI reads, hard-coded in FPGA*/
#define LBT_MIN_STEP_FREQ       100000

/* -------------------------------------------------------------------------- */
/* --- GLOBAL VARIABLES ----------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main( int argc, char ** argv )
{
    int i, j, k; /* loop and temporary variables */
    int x; /* return code for functions */
    int32_t reg_val;

    /* Parameter parsing */
    double arg_lf[3] = {0,0,0};
    unsigned arg_u = 0;
    int arg_i = 0;
    char arg_s[64];

    /* Application parameters */
    uint32_t init_freq = DEFAULT_START_FREQ;
    uint32_t start_freq = DEFAULT_START_FREQ;
    uint32_t stop_freq = DEFAULT_STOP_FREQ;
    uint32_t step_freq = DEFAULT_STEP_FREQ;
    uint16_t rssi_pts = DEFAULT_RSSI_PTS;
    int8_t rssi_offset = DEFAULT_SX127X_RSSI_OFFSET;
    enum lgw_sx127x_rxbw_e channel_bw_khz = DEFAULT_CHAN_BW;
    char log_file_name[64] = DEFAULT_LOG_NAME;
    FILE * log_file = NULL;

    /* Local var */
    bool lbt_support = false;
    int freq_idx;
    int freq_nb;
    uint64_t freq_reg;
    uint32_t freq;
    uint8_t read_burst[RSSI_RANGE*2];
    uint16_t rssi_histo;
    uint16_t rssi_cumu;
    float rssi_thresh[] = {0.1,0.3,0.5,0.8,1};

    /* Parse command line options */
    while((i = getopt(argc, argv, "hf:n:b:l:o:")) != -1) {
        switch (i) {
        case 'h':
            printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
            printf(" -f <float>:<float>:<float>  Frequency vector to scan in MHz (start:step:stop)\n");
            printf("                               start>%3.3f step>%1.3f stop<%3.3f\n", MIN_FREQ/1e6, MIN_STEP_FREQ/1e6, MAX_FREQ/1e6);
            printf(" -b <uint>  Channel bandwidth in KHz [25,50,100,125,200,250,500]\n");
            printf(" -n <uint>  Total number of RSSI points [1..65535]\n");
            printf(" -o <int>   Offset in dB to be applied to the SX127x RSSI [-128..127]\n");
            printf(" -l <char>  Log file name\n");
            printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
            return EXIT_SUCCESS;

        case 'f': /* -f <float>:<float>:<float>  Frequency vector to scan in MHz, start:step:stop */
            j = sscanf(optarg, "%lf:%lf:%lf", &arg_lf[0], &arg_lf[1], &arg_lf[2]);
            if ((j!=3) || (arg_lf[0] < MIN_FREQ/1e6) || (arg_lf[0] > MAX_FREQ/1e6) || (arg_lf[1] < MIN_STEP_FREQ/1e6) || (arg_lf[2] < MIN_FREQ/1e6) || (arg_lf[2] > MAX_FREQ/1e6)) {
                printf("ERROR: argument parsing of -f argument. -h for help.\n");
                return EXIT_FAILURE;
            } else {
                start_freq = (uint32_t)((arg_lf[0] * 1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
                step_freq = (uint32_t)((arg_lf[1] * 1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
                stop_freq = (uint32_t)((arg_lf[2] * 1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
            }
            break;

        case 'b': /* -b <uint>  Channel bandwidth in KHz [25,50,100,125,200,250,500] */
            j = sscanf(optarg, "%u", &arg_u);
            if (j != 1) {
                printf("ERROR: argument parsing of -b argument. -h for help.\n");
                return EXIT_FAILURE;
            } else {
                switch (arg_u) {
                    case 25:
                        channel_bw_khz = LGW_SX127X_RXBW_12K5_HZ;
                        break;
                    case 50:
                        channel_bw_khz = LGW_SX127X_RXBW_25K_HZ;
                        break;
                    case 100:
                        channel_bw_khz = LGW_SX127X_RXBW_50K_HZ;
                        break;
                    case 125:
                        channel_bw_khz = LGW_SX127X_RXBW_62K5_HZ;
                        break;
                    case 200:
                        channel_bw_khz = LGW_SX127X_RXBW_100K_HZ;
                        break;
                    case 250:
                        channel_bw_khz = LGW_SX127X_RXBW_125K_HZ;
                        break;
                    case 500:
                        channel_bw_khz = LGW_SX127X_RXBW_250K_HZ;
                        break;
                    default:
                        printf("ERROR: argument parsing of -b argument. -h for help.\n");
                        return EXIT_FAILURE;
                }
            }
            break;

        case 'n': /* -n <uint>  Total number of RSSI points [1..65535] */
            j = sscanf(optarg, "%u", &arg_u);
            if ((j != 1) || (arg_u < 1) || (arg_u > 65535)) {
                printf("ERROR: argument parsing of -n argument. -h for help.\n");
                return EXIT_FAILURE;
            } else {
                rssi_pts = (uint16_t)arg_u;
            }
            break;

        case 'o': /* -o <int>  SX127x RSSI offset [-128..127] */
            j = sscanf(optarg, "%i", &arg_i);
            if ((j != 1) || (arg_i < -128) || (arg_i > 127)) {
                printf("ERROR: argument parsing of -o argument. -h for help.\n");
                return EXIT_FAILURE;
            } else {
                rssi_offset = (int8_t)arg_i;
            }
            break;

        case 'l': /* -l <char>  Log file name */
            j = sscanf(optarg, "%s", arg_s);
            if (j != 1) {
                printf("ERROR: argument parsing of -l argument. -h for help.\n");
                return EXIT_FAILURE;
            } else {
                sprintf(log_file_name, "%s", arg_s);
            }
            break;

        default:
            printf("ERROR: argument parsing options. -h for help.\n");
            return EXIT_FAILURE;
        }
    }

    /* Start message */
    printf("+++ Start spectral scan of LoRa gateway channels +++\n");

    x = lgw_connect(true, 0); /* SPI only, no FPGA reset/configure (for now) */
    if(x != 0) {
        printf("ERROR: Failed to connect to FPGA\n");
        return EXIT_FAILURE;
    }

    /* Check if FPGA supports Spectral Scan */
    lgw_fpga_reg_r(LGW_FPGA_FEATURE, &reg_val);
    if (TAKE_N_BITS_FROM((uint8_t)reg_val, FPGA_FEATURE_SPECTRAL_SCAN, 1) != true) {
        printf("ERROR: Spectral Scan is not supported (0x%x)\n", (uint8_t)reg_val);
        return EXIT_FAILURE;
    }

    /* Check if FPGA supports LBT, in order to apply proper constraints on spectral scan parameters */
    lgw_fpga_reg_r(LGW_FPGA_FEATURE, &reg_val);
    if (TAKE_N_BITS_FROM((uint8_t)reg_val, FPGA_FEATURE_LBT, 1) == true) {
        printf("WARNING: The FPGA supports LBT, so running spectral scan with specific constraints\n");
        printf("         => Check the parameters summary below\n");
        /* Get start frequency from FPGA */
        lgw_fpga_reg_r(LGW_FPGA_LBT_INITIAL_FREQ, &reg_val);
        switch (reg_val) {
            case 0:
                init_freq = 915000000;
                break;
            case 1:
                init_freq = 863000000;
                break;
            default:
                printf("ERROR: init frequency %d is not supported\n", reg_val);
                return EXIT_FAILURE;
        }

        /* Check parameters based on LBT constraints */
        if (start_freq < init_freq) {
            printf("ERROR: start frequency %d is not supported, should be >=%d\n", start_freq, init_freq);
            return EXIT_FAILURE;
        }
        if (stop_freq > (init_freq + 255*LBT_MIN_STEP_FREQ)) {
            printf("ERROR: stop frequency %d is not supported, should be <%d\n", stop_freq, init_freq + 255*LBT_MIN_STEP_FREQ);
            return EXIT_FAILURE;
        }
        if (step_freq < LBT_MIN_STEP_FREQ) {
            printf("ERROR: step frequency %d is not supported, should be >=%d\n", step_freq, LBT_MIN_STEP_FREQ);
            return EXIT_FAILURE;
        } else {
            /* Ensure the given step is a multiple of LBT_MIN_STEP_FREQ */
            step_freq = (step_freq / LBT_MIN_STEP_FREQ) * LBT_MIN_STEP_FREQ;
        }

        /* Overload hard-coded spectral scan parameters */
        rssi_pts = LBT_DEFAULT_RSSI_PTS;

        /* Spectral scan sequence is slightly different depending if LBT is there or not */
        lbt_support = true;
    } else {
        /* Reconnect to FPGA with sw reset and configure */
        x = lgw_disconnect();
        if(x != 0) {
            printf("ERROR: Failed to disconnect from FPGA\n");
            return EXIT_FAILURE;
        }
        x = lgw_connect(false, LGW_DEFAULT_NOTCH_FREQ); /* FPGA reset/configure */
        if(x != 0) {
            printf("ERROR: Failed to connect to FPGA\n");
            return EXIT_FAILURE;
        }
        /* Some spectral scan options are only available when there is no LBT support */
        x = lgw_fpga_reg_w(LGW_FPGA_HISTO_NB_READ, rssi_pts-1);
        if( x != LGW_REG_SUCCESS )
        {
            printf( "ERROR: Failed to configure FPGA\n" );
            return EXIT_FAILURE;
        }

        /* Initialize frequency */
        freq_reg = ((uint64_t)start_freq << 19) / (uint64_t)32000000;
        lgw_fpga_reg_w(LGW_FPGA_HISTO_SCAN_FREQ, (int32_t)freq_reg);
    }

    /* create log file */
    strcat(log_file_name,".csv");
    log_file = fopen(log_file_name, "w");
    if (log_file == NULL) {
        printf("ERROR: impossible to create log file %s\n", log_file_name);
        return EXIT_FAILURE;
    }
    printf("Writing to file: %s\n", log_file_name);

    /* Number of frequency steps */
    freq_nb = (int)((stop_freq - start_freq) / step_freq) + 1;
    printf("Scanning frequencies:\nstart: %d Hz\nstop : %d Hz\nstep : %d Hz\nnb   : %d\n", start_freq, stop_freq, step_freq, freq_nb);

    /* Main loop */
    for(j = 0; j < freq_nb; j++) {
        /* Current frequency */
        freq = start_freq + j * step_freq;
        printf("%d", freq);

        if (lbt_support == false) {
            /* Set SX127x */
            x = lgw_setup_sx127x(freq, MOD_FSK, channel_bw_khz, rssi_offset);
            if( x != 0 )
            {
                printf( "ERROR: SX127x setup failed\n" );
                return EXIT_FAILURE;
            }

            /* Start FPGA state machine for spectral scal */
            lgw_fpga_reg_w(LGW_FPGA_CTRL_FEATURE_START, 1);
        } else {
            /* Do Nothing */
            /* LBT setup has already done the necessary */
        }

        /* Clean histogram */
        lgw_fpga_reg_w(LGW_FPGA_CTRL_CLEAR_HISTO_MEM, 1);

        /* Wait for histogram clean to start */
        do {
            wait_ms(10);
            lgw_fpga_reg_r(LGW_FPGA_STATUS, &reg_val);
        }
        while((TAKE_N_BITS_FROM((uint8_t)reg_val, 0, 5)) != 1); /* Clear has started */

        /* Set scan frequency during clear process */
        if (lbt_support == false) {
            /* We can directly set the scan frequency */
            freq_reg = ((uint64_t)freq << 19) / (uint64_t)32000000;
            lgw_fpga_reg_w(LGW_FPGA_HISTO_SCAN_FREQ, (int32_t)freq_reg);
        } else {
            /* The possible scan frequencies are hard-coded in FPGA, we give an offset from init_freq */
            freq_idx = (freq - init_freq) / LBT_MIN_STEP_FREQ;
            printf(" (idx=%i) ", freq_idx);
            lgw_fpga_reg_w(LGW_FPGA_SCAN_FREQ_OFFSET, freq_idx);
        }

        /* Release FPGA state machine */
        lgw_fpga_reg_w(LGW_FPGA_CTRL_CLEAR_HISTO_MEM, 0);

        /* Wait for histogram ready */
        do {
            wait_ms(1000);
            lgw_fpga_reg_r(LGW_FPGA_STATUS, &reg_val);
        }
        while((TAKE_N_BITS_FROM((uint8_t)reg_val, 5, 1)) != 1);

        if (lbt_support == false) {
            /* Stop FPGA state machine for spectral scan */
            lgw_fpga_reg_w(LGW_FPGA_CTRL_FEATURE_START, 0);
        } else {
            /* Do Nothing */
            /* LBT is running */
        }

        /* Read histogram */
        lgw_fpga_reg_w(LGW_FPGA_CTRL_ACCESS_HISTO_MEM, 1); /* HOST gets access to FPGA RAM */
        lgw_fpga_reg_w(LGW_FPGA_HISTO_RAM_ADDR, 0);
        lgw_fpga_reg_rb(LGW_FPGA_HISTO_RAM_DATA, read_burst, RSSI_RANGE*2);
        lgw_fpga_reg_w(LGW_FPGA_CTRL_ACCESS_HISTO_MEM, 0); /* FPGA gets access to RAM back */

        /* Write data to CSV */
        fprintf(log_file, "%d", freq);
        rssi_cumu = 0;
        k = 0;
        for (i = 0; i < RSSI_RANGE; i++) {
            rssi_histo = (uint16_t)read_burst[2*i] | ((uint16_t)read_burst[2*i+1] << 8);
            fprintf(log_file, ",%.1f,%d", -i/2.0, rssi_histo);
            rssi_cumu += rssi_histo;
            if (rssi_cumu > rssi_pts) {
                printf(" - WARNING: number of RSSI points higher than expected (%u,%u)", rssi_cumu, rssi_pts);
                rssi_cumu = rssi_pts;
            }
            if (rssi_cumu > rssi_thresh[k]*rssi_pts) {
                printf("  %d%%<%.1f", (uint16_t)(rssi_thresh[k]*100), -i/2.0);
                k++;
            }
        }
        fprintf(log_file, "\n");
        printf("\n");
    }
    fclose(log_file);

    /* Close SPI */
    x = lgw_disconnect();
    if(x != 0) {
        printf("ERROR: Failed to disconnect FPGA\n");
        return EXIT_FAILURE;
    }

    printf("+++  Exiting Spectral scan program +++\n");

    return EXIT_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* --- EOF ------------------------------------------------------------------ */

