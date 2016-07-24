/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
    Minimum test program for the loragw_hal 'library'

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
#include <math.h>       /* cos */
#include <unistd.h>     /* getopt access */

#include "loragw_hal.h"
#include "loragw_reg.h"
#include "loragw_aux.h"
#include "loragw_radio.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define DEFAULT_RSSI_OFFSET 0.0
#define NB_CAL_MAX          100
#define MCU_AGC             1
#define MCU_AGC_FW_BYTE     8192 /* size of the firmware IN BYTES (= twice the number of 14b words) */
#define FW_VERSION_ADDR     0x20
#define FW_VERSION_CAL      2
#define RAM_SIZE            4096
#define FREQ_SIG_NORM       0.078125

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

#include "../src/cal_fw.var" /* external definition of the variable */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE TYPES --------------------------------------------------------- */

struct cal_res_s {
    int8_t amp_a;
    int8_t phi_a;
    int8_t amp_b;
    int8_t phi_b;
    int8_t offset_i_a [8];
    int8_t offset_q_a [8];
    int8_t offset_i_b [8];
    int8_t offset_q_b [8];
    uint8_t img_rej_a;
    uint8_t img_rej_b;
    uint8_t offset_rej_a [8];
    uint8_t offset_rej_b [8];
    uint8_t debug [8];
};

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

int load_firmware(uint8_t target, uint8_t *firmware, uint16_t size); /* defined in loragw_hal.c */

uint8_t sx125x_cal(uint8_t cal_cmd, struct cal_res_s *cal_res);

int read_capture(int16_t *i, int16_t *q, int nb_samp);

uint8_t get_img_rej(int16_t *sig_i, int16_t *sig_q, int nb_samp, double f_sig_norm);

void usage (void);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

/* describe command line options */
void usage(void) {
    printf("Library version information: %s\n", lgw_version_info());
    printf( "Available options:\n");
    printf( " -h print this help\n");
    printf( " -a <float> Radio A frequency in MHz\n");
    printf( " -b <float> Radio B frequency in MHz\n");
    printf( " -r <int> Radio type (SX1255:1255, SX1257:1257)\n");
    printf( " -n <uint> Number of calibration iterations\n");
    printf( " -k <int> Concentrator clock source (0:radio_A, 1:radio_B(default))\n");
    printf( " -t <int> Radio to run TX calibration on (0:None(default), 1:radio_A, 2:radio_B, 3:both)\n");
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv)
{
    int i, j, x;
    int32_t read_val;
    struct lgw_conf_board_s boardconf;
    struct lgw_conf_rxrf_s rfconf;
    uint8_t fw_version;
    uint8_t cal_cmd;
    uint8_t cal_status;
    struct cal_res_s cal_res [NB_CAL_MAX];
    struct cal_res_s cal_res_max;
    struct cal_res_s cal_res_min;
    int16_t sig_i [RAM_SIZE];
    int16_t sig_q [RAM_SIZE];
    uint8_t img_rej_a [NB_CAL_MAX];
    uint8_t img_rej_b [NB_CAL_MAX];
    uint8_t img_rej_a_max;
    uint8_t img_rej_a_min;
    uint8_t img_rej_b_max;
    uint8_t img_rej_b_min;
    //FILE *file;

    /* command line options */
    int xi = 0;
    double xd = 0.0;
    uint32_t fa = 0, fb = 0;
    enum lgw_radio_type_e radio_type = LGW_RADIO_TYPE_NONE;
    uint8_t clocksource = 1; /* Radio B is source by default */
    uint8_t tx_enable = 0;
    int nb_cal = 5;

    /* parse command line options */
    while ((i = getopt (argc, argv, "ha:b:r:n:k:t:")) != -1) {
        switch (i) {
            case 'h':
                usage();
                return -1;
                break;
            case 'a': /* <float> Radio A frequency in MHz */
                sscanf(optarg, "%lf", &xd);
                fa = (uint32_t)((xd*1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
                break;
            case 'b': /* <float> Radio B frequency in MHz */
                sscanf(optarg, "%lf", &xd);
                fb = (uint32_t)((xd*1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
                break;
            case 'r': /* <int> Radio type (1255, 1257) */
                sscanf(optarg, "%i", &xi);
                switch (xi) {
                    case 1255:
                        radio_type = LGW_RADIO_TYPE_SX1255;
                        break;
                    case 1257:
                        radio_type = LGW_RADIO_TYPE_SX1257;
                        break;
                    default:
                        printf("ERROR: invalid radio type\n");
                        usage();
                        return -1;
                }
                break;
            case 'n': /* <uint> Number of calibration iterations */
                i = sscanf(optarg, "%i", &xi);
                if ((i != 1) || (xi > NB_CAL_MAX)) {
                    printf("ERROR: invalid number of calibration iterations (MAX %d)\n",NB_CAL_MAX);
                    usage();
                    return -1;
                } else {
                    nb_cal = xi;
                }
                break;
            case 'k': /* <int> Concentrator clock source (Radio A or Radio B) */
                sscanf(optarg, "%i", &xi);
                clocksource = (uint8_t)xi;
                break;
            case 't': /* <int> Radio to run TX calibration on */
                sscanf(optarg, "%i", &xi);
                tx_enable = (uint8_t)xi;
                break;
            default:
                printf("ERROR: argument parsing\n");
                usage();
                return -1;
        }
    }

    /* check input parameters */
    if ((fa == 0) || (fb == 0)) {
        printf("ERROR: missing frequency input parameter:\n");
        printf("  Radio A RX: %u\n", fa);
        printf("  Radio B RX: %u\n", fb);
        usage();
        return -1;
    }

    if (radio_type == LGW_RADIO_TYPE_NONE) {
        printf("ERROR: missing radio type parameter:\n");
        usage();
        return -1;
    }

    /* starting the concentrator */
    /* board config */
    memset(&boardconf, 0, sizeof(boardconf));

    boardconf.lorawan_public = true;
    boardconf.clksrc = clocksource;
    lgw_board_setconf(boardconf);

    /* RF config */
    memset(&rfconf, 0, sizeof(rfconf));

    rfconf.enable = true;
    rfconf.freq_hz = fa;
    rfconf.rssi_offset = DEFAULT_RSSI_OFFSET;
    rfconf.type = radio_type;
    rfconf.tx_enable = false; /* ignored */
    lgw_rxrf_setconf(0, rfconf);

    rfconf.freq_hz = fb;
    rfconf.tx_enable = false; /* ignored */
    lgw_rxrf_setconf(1, rfconf);

    /* Calibration command */
    cal_cmd = 0;
    //cal_cmd |= 0x01; /* Bit 0: Calibrate Rx IQ mismatch compensation on radio A */
    //cal_cmd |= 0x02; /* Bit 1: Calibrate Rx IQ mismatch compensation on radio B */
    //cal_cmd |= 0x04; /* Bit 2: Calibrate Tx DC offset on radio A */
    //cal_cmd |= 0x08; /* Bit 3: Calibrate Tx DC offset on radio B */
    cal_cmd |= 0x10; /* Bit 4: 0: calibrate with DAC gain=2, 1: with DAC gain=3 (use 3) */

    switch (radio_type) {
        case LGW_RADIO_TYPE_SX1255:
            cal_cmd |= 0x20; /* Bit 5: 0: SX1257, 1: SX1255 */
            break;
        case LGW_RADIO_TYPE_SX1257:
            cal_cmd |= 0x00; /* Bit 5: 0: SX1257, 1: SX1255 */
            break;
        default:
            break;
    }

    cal_cmd |= 0x00; /* Bit 6-7: Board type 0: ref, 1: FPGA, 3: board X */

    /* Recap parameters*/
    printf("Library version information: %s\n", lgw_version_info());
    printf("Radio type: %d\n",radio_type);
    printf("Radio A frequency: %f MHz\n",fa/1e6);
    printf("Radio B frequency: %f MHz\n",fb/1e6);
    printf("Number of calibration iterations: %d\n",nb_cal);
    printf("Calibration command: brd: %d, chip: %d, dac: %d\n\n", cal_cmd >> 6, 1257-2*((cal_cmd & 0x20) >> 5), 2+((cal_cmd & 0x10) >> 4));

    x = lgw_connect();
    if (x == -1) {
        printf("ERROR: FAIL TO CONNECT BOARD\n");
        return -1;
    }

    /* reset the registers (also shuts the radios down) */
    lgw_soft_reset();

    /* ungate clocks (gated by default) */
    lgw_reg_w(LGW_GLOBAL_EN, 1);

    /* switch on and reset the radios (also starts the 32 MHz XTAL) */
    lgw_reg_w(LGW_RADIO_A_EN,1);
    lgw_reg_w(LGW_RADIO_B_EN,1);
    wait_ms(500); /* TODO: optimize */
    lgw_reg_w(LGW_RADIO_RST,1);
    wait_ms(5);
    lgw_reg_w(LGW_RADIO_RST,0);

    /* setup the radios */
    setup_sx125x(0, clocksource, true, radio_type, fa);
    setup_sx125x(1, clocksource, false, radio_type, fb);

    /* Set GPIO 4 high for calibration */
    lgw_reg_w(LGW_GPIO_MODE,31); /* Set all GPIOs as output */
    lgw_reg_w(LGW_GPIO_SELECT_OUTPUT,2); /* AGC MCU drives GPIOs */

    /* Load the calibration firmware  */
    load_firmware(MCU_AGC, cal_firmware, MCU_AGC_FW_BYTE);
    lgw_reg_w(LGW_MCU_RST_1,0);
    lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, FW_VERSION_ADDR);
    lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &read_val);
    fw_version = (uint8_t)read_val;
    if (fw_version != FW_VERSION_CAL) {
        printf("ERROR: Version of calibration firmware not expected, actual:%d expected:%d\n", fw_version, FW_VERSION_CAL);
        return -1;
    }

    /* Run Rx A IQ mismatch calibration only */
    for (i=0; i<nb_cal; i++) {
        cal_status = sx125x_cal(cal_cmd | 0x01, &cal_res[i]);
        x = read_capture(sig_i, sig_q, RAM_SIZE);
        /*
        file = fopen("toto.txt","w");
        for (j=0; j<RAM_SIZE; j++) {
            fprintf(file, "%d %d\n", sig_i[j], sig_q[j]);
        }
        fclose(file);
        */
        img_rej_a[i] = get_img_rej(sig_i, sig_q, RAM_SIZE, FREQ_SIG_NORM);

        printf("Rx A IQ mismatch: Amp: %3d Phi: %3d Rej: %2d dB Status: %3d | Debug: Rej: %2d dB Lna: %1d BB: %2d Dec: %2d\n", cal_res[i].amp_a, cal_res[i].phi_a, cal_res[i].img_rej_a, cal_status, img_rej_a[i], cal_res[i].debug[0], cal_res[i].debug[1], cal_res[i].debug[2]);
    }

    /* Run Rx B IQ mismatch calibation only */
    printf("\n");
    for (i=0; i<nb_cal; i++) {
        cal_status = sx125x_cal(cal_cmd | 0x02, &cal_res[i]);
        x = read_capture(sig_i, sig_q, RAM_SIZE);
        img_rej_b[i] = get_img_rej(sig_i, sig_q, RAM_SIZE, FREQ_SIG_NORM);

        printf("Rx B IQ mismatch: Amp: %3d Phi: %3d Rej: %2d dB Status: %3d | Debug: Rej: %2d dB Lna: %1d BB: %2d Dec: %2d\n", cal_res[i].amp_b, cal_res[i].phi_b, cal_res[i].img_rej_b, cal_status, img_rej_b[i], cal_res[i].debug[0], cal_res[i].debug[1], cal_res[i].debug[2]);
    }

    /* Run Tx A DC offset calibation only */
    printf("\n");
    if ((tx_enable == 1) || (tx_enable == 3)) {
        for (i=0; i<nb_cal; i++) {
            cal_status = sx125x_cal(cal_cmd | 0x04, &cal_res[i]);

            printf("Tx A DC offset I :");
            for (j=0; j<8; j++) {
                printf(" %3d", cal_res[i].offset_i_a[j]);
            }
            printf("\n");
            printf("Tx A DC offset Q :");
            for (j=0; j<8; j++) {
                printf(" %3d", cal_res[i].offset_q_a[j]);
            }
            printf("\n");
            printf("Tx A DC rejection:");
            for (j=0; j<8; j++) {
                printf(" %3d", cal_res[i].offset_rej_a[j]);
            }
            printf("\n");
            printf("Tx A DC debug BB :");
            for (j=0; j<8; j++) {
                printf(" %3d", (cal_res[i].debug[j] & 0xF0) >> 4);
            }
            printf("\n");
            printf("Tx A DC debug Dec:");
            for (j=0; j<8; j++) {
                printf(" %3d", cal_res[i].debug[j] & 0x0F);
            }
            printf("\n");
            printf("Tx A DC Status   : %3d\n", cal_status);
        }
    } else {
        printf("Tx A calibration bypassed\n");
    }

    /* Run Tx B DC offset calibation only */
    printf("\n");
    if ((tx_enable == 2) || (tx_enable == 3)) {
        for (i=0; i<nb_cal; i++) {
            cal_status = sx125x_cal(cal_cmd | 0x08, &cal_res[i]);

            printf("Tx B DC offset I :");
            for (j=0; j<8; j++) {
                printf(" %3d", cal_res[i].offset_i_b[j]);
            }
            printf("\n");
            printf("Tx B DC offset Q :");
            for (j=0; j<8; j++) {
                printf(" %3d", cal_res[i].offset_q_b[j]);
            }
            printf("\n");
            printf("Tx B DC rejection:");
            for (j=0; j<8; j++) {
                printf(" %3d", cal_res[i].offset_rej_b[j]);
            }
            printf("\n");
            printf("Tx B DC debug BB :");
            for (j=0; j<8; j++) {
                printf(" %3d", (cal_res[i].debug[j] & 0xF0) >> 4);
            }
            printf("\n");
            printf("Tx B DC debug Dec:");
            for (j=0; j<8; j++) {
                printf(" %3d", cal_res[i].debug[j] & 0x0F);
            }
            printf("\n");
            printf("Tx B DC Status   : %3d\n", cal_status);
        }
    }  else {
        printf("Tx B calibration bypassed\n");
    }

    /* Compute statistics */
    cal_res_max.amp_a = -128;
    cal_res_max.phi_a = -128;
    cal_res_max.amp_b = -128;
    cal_res_max.phi_b = -128;
    cal_res_max.img_rej_a = 0;
    cal_res_max.img_rej_b = 0;
    for (j=0; j<8; j++) {
        cal_res_max.offset_i_a[j] = -128;
        cal_res_max.offset_q_a[j] = -128;
        cal_res_max.offset_i_b[j] = -128;
        cal_res_max.offset_q_b[j] = -128;
        cal_res_max.offset_rej_a[j] = 0;
        cal_res_max.offset_rej_b[j] = 0;
    }

    cal_res_min.amp_a = 127;
    cal_res_min.phi_a = 127;
    cal_res_min.amp_b = 127;
    cal_res_min.phi_b = 127;
    cal_res_min.img_rej_a = 255;
    cal_res_min.img_rej_b = 255;
    for (j=0; j<8; j++) {
        cal_res_min.offset_i_a[j] = 127;
        cal_res_min.offset_q_a[j] = 127;
        cal_res_min.offset_i_b[j] = 127;
        cal_res_min.offset_q_b[j] = 127;
        cal_res_min.offset_rej_a[j] = 255;
        cal_res_min.offset_rej_b[j] = 255;
    }

    img_rej_a_max = 0;
    img_rej_a_min = 255;
    img_rej_b_max = 0;
    img_rej_b_min = 255;

    for (i=0; i<nb_cal; i++) {
        if (cal_res[i].amp_a > cal_res_max.amp_a) {
            cal_res_max.amp_a = cal_res[i].amp_a;
        }
        if (cal_res[i].phi_a > cal_res_max.phi_a) {
            cal_res_max.phi_a = cal_res[i].phi_a;
        }
        if (cal_res[i].amp_b > cal_res_max.amp_b) {
            cal_res_max.amp_b = cal_res[i].amp_b;
        }
        if (cal_res[i].phi_b > cal_res_max.phi_b) {
            cal_res_max.phi_b = cal_res[i].phi_b;
        }
        if (cal_res[i].phi_b > cal_res_max.phi_b) {
            cal_res_max.phi_b = cal_res[i].phi_b;
        }
        if (cal_res[i].img_rej_a > cal_res_max.img_rej_a) {
            cal_res_max.img_rej_a = cal_res[i].img_rej_a;
        }
        if (cal_res[i].img_rej_b > cal_res_max.img_rej_b) {
            cal_res_max.img_rej_b = cal_res[i].img_rej_b;
        }
        for (j=0; j<8; j++) {
            if (cal_res[i].offset_i_a[j]  > cal_res_max.offset_i_a[j]) {
                cal_res_max.offset_i_a[j] = cal_res[i].offset_i_a[j];
            }
            if (cal_res[i].offset_q_a[j] > cal_res_max.offset_q_a[j]) {
                cal_res_max.offset_q_a[j] = cal_res[i].offset_q_a[j];
            }
            if (cal_res[i].offset_i_b[j] > cal_res_max.offset_i_b[j]) {
                cal_res_max.offset_i_b[j] = cal_res[i].offset_i_b[j];
            }
            if (cal_res[i].offset_q_b[j] > cal_res_max.offset_q_b[j]) {
                cal_res_max.offset_q_b[j] = cal_res[i].offset_q_b[j];
            }
            if (cal_res[i].offset_rej_a[j] > cal_res_max.offset_rej_a[j]) {
                cal_res_max.offset_rej_a[j] = cal_res[i].offset_rej_a[j];
            }
            if (cal_res[i].offset_rej_b[j] > cal_res_max.offset_rej_b[j]) {
                cal_res_max.offset_rej_b[j] = cal_res[i].offset_rej_b[j];
            }
        }

        if (cal_res[i].amp_a < cal_res_min.amp_a) {
            cal_res_min.amp_a = cal_res[i].amp_a;
        }
        if (cal_res[i].phi_a < cal_res_min.phi_a) {
            cal_res_min.phi_a = cal_res[i].phi_a;
        }
        if (cal_res[i].amp_b < cal_res_min.amp_b) {
            cal_res_min.amp_b = cal_res[i].amp_b;
        }
        if (cal_res[i].phi_b < cal_res_min.phi_b) {
            cal_res_min.phi_b = cal_res[i].phi_b;
        }
        if (cal_res[i].phi_b < cal_res_min.phi_b) {
            cal_res_min.phi_b = cal_res[i].phi_b;
        }
        if (cal_res[i].img_rej_a < cal_res_min.img_rej_a) {
            cal_res_min.img_rej_a = cal_res[i].img_rej_a;
        }
        if (cal_res[i].img_rej_b < cal_res_min.img_rej_b) {
            cal_res_min.img_rej_b = cal_res[i].img_rej_b;
        }
        for (j=0; j<8; j++) {
            if (cal_res[i].offset_i_a[j] < cal_res_min.offset_i_a[j]) {
                cal_res_min.offset_i_a[j] = cal_res[i].offset_i_a[j];
            }
            if (cal_res[i].offset_q_a[j] < cal_res_min.offset_q_a[j]) {
                cal_res_min.offset_q_a[j] = cal_res[i].offset_q_a[j];
            }
            if (cal_res[i].offset_i_b[j] < cal_res_min.offset_i_b[j]) {
                cal_res_min.offset_i_b[j] = cal_res[i].offset_i_b[j];
            }
            if (cal_res[i].offset_q_b[j] < cal_res_min.offset_q_b[j]) {
                cal_res_min.offset_q_b[j] = cal_res[i].offset_q_b[j];
            }
            if (cal_res[i].offset_rej_a[j] < cal_res_min.offset_rej_a[j]) {
                cal_res_min.offset_rej_a[j] = cal_res[i].offset_rej_a[j];
            }
            if (cal_res[i].offset_rej_b[j] < cal_res_min.offset_rej_b[j]) {
                cal_res_min.offset_rej_b[j] = cal_res[i].offset_rej_b[j];
            }
        }

        if (img_rej_a[i] > img_rej_a_max) {
            img_rej_a_max = img_rej_a[i];
        }
        if (img_rej_a[i] < img_rej_a_min) {
            img_rej_a_min = img_rej_a[i];
        }
        if (img_rej_b[i] > img_rej_b_max) {
            img_rej_b_max = img_rej_b[i];
        }
        if (img_rej_b[i] < img_rej_b_min) {
            img_rej_b_min = img_rej_b[i];
        }
    }

    /* Print statistics */
    printf("\n");
    printf("Rx A IQ mismatch calibration statistics on %3d iterations (min, max):\n", nb_cal);
    printf("Amp: %3d %3d Phi: %3d %3d Rej: %2d %2d dB (capt.: %2d %2d dB)\n", cal_res_min.amp_a, cal_res_max.amp_a, cal_res_min.phi_a, cal_res_max.phi_a, cal_res_min.img_rej_a, cal_res_max.img_rej_a, img_rej_a_min, img_rej_a_max);

    printf("\n");
    printf("Rx B IQ mismatch calibration statistics on %3d iterations (min, max):\n", nb_cal);
    printf("Amp: %3d %3d Phi: %3d %3d Rej: %2d %2d dB (capt.: %2d %2d dB)\n", cal_res_min.amp_b, cal_res_max.amp_b, cal_res_min.phi_b, cal_res_max.phi_b, cal_res_min.img_rej_b, cal_res_max.img_rej_b, img_rej_b_min, img_rej_b_max);

    if ((tx_enable == 1) || (tx_enable == 3)) {
        printf("\n");
        printf("Tx A DC offset calibration statistics on %3d iterations (min, max):\n", nb_cal);
        for (j=0; j<8; j++) {
            printf(" Mix gain %2d: I: %3d %3d Q: %3d %3d Rej: %2d %2d dB\n", 8+j, cal_res_min.offset_i_a[j], cal_res_max.offset_i_a[j], cal_res_min.offset_q_a[j], cal_res_max.offset_q_a[j], cal_res_min.offset_rej_a[j], cal_res_max.offset_rej_a[j]);
        }
    }

    if ((tx_enable == 2) || (tx_enable == 3)) {
        printf("\n");
        printf("Tx B DC offset calibration statistics on %3d iterations (min, max):\n", nb_cal);
        for (j=0; j<8; j++) {
            printf(" Mix gain %2d: I: %3d %3d Q: %3d %3d Rej: %2d %2d dB\n", 8+j, cal_res_min.offset_i_b[j], cal_res_max.offset_i_b[j], cal_res_min.offset_q_b[j], cal_res_max.offset_q_b[j], cal_res_min.offset_rej_b[j], cal_res_max.offset_rej_b[j]);
        }
    }

    lgw_stop();

    printf("\nEnd of radio calibration test\n");

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint8_t sx125x_cal(uint8_t cal_cmd, struct cal_res_s *cal_res) {

    int i;
    int32_t read_val;
    uint8_t cal_status;

    lgw_reg_w(LGW_FORCE_HOST_RADIO_CTRL,0); /* gives to AGC MCU the control of the radios */
    lgw_reg_w(LGW_RADIO_SELECT,cal_cmd); /* send calibration configuration word */
    lgw_reg_w(LGW_MCU_RST_1,1);
    lgw_reg_w(LGW_MCU_RST_1,0);
    lgw_reg_w(LGW_PAGE_REG,3); /* Calibration will start on this condition as soon as MCU can talk to concentrator registers */
    lgw_reg_w(LGW_EMERGENCY_FORCE_HOST_CTRL,0); /* Give control of concentrator registers to MCU */

    wait_ms(2000); /* Wait for end of calibration */

    lgw_reg_w(LGW_EMERGENCY_FORCE_HOST_CTRL,1); /* Take back control */

    /* Get calibration status */
    lgw_reg_r(LGW_MCU_AGC_STATUS, &read_val);
    cal_status = (uint8_t)read_val;

    /* Check calibration flags
        bit 0: could access SX1301 registers
        bit 1: could access radio A registers
        bit 2: could access radio B registers
        bit 3: radio A RX image rejection successful
        bit 4: radio B RX image rejection successful
        bit 5: radio A TX imbalance correction successful
        bit 6: radio B TX imbalance correction successful
        bit 7: calibration finished */

    if ((cal_status & 0x01) == 0) {
        printf("WARNING: calibration could not access SX1301 registers\n");
    }
    if ((cal_status & 0x02) == 0) {
        printf("WARNING: calibration could not access radio A\n");
    }
    if ((cal_status & 0x04) == 0) {
        printf("WARNING: calibration could not access radio B\n");
    }
    if ((cal_cmd & 0x01) && ((cal_status & 0x08) == 0)) {
        printf("WARNING: problem in calibration of radio A for image rejection\n");
    }
    if ((cal_cmd & 0x02) && ((cal_status & 0x10) == 0)) {
        printf("WARNING: problem in calibration of radio B for image rejection\n");
    }
    if ((cal_cmd & 0x04) && ((cal_status & 0x20) == 0)) {
        printf("WARNING: problem in calibration of radio A for TX imbalance\n");
    }
    if ((cal_cmd & 0x08) && ((cal_status & 0x40) == 0)) {
        printf("WARNING: problem in calibration of radio B for TX imbalance\n");
    }
    if ((cal_status & 0x80) == 0) {
        printf("WARNING: Calibration not finished\n");
    }

    /* Get calibration results */
    if (cal_cmd & 0x01) {
        lgw_reg_r(LGW_IQ_MISMATCH_A_AMP_COEFF, &read_val);
        (*cal_res).amp_a = (int8_t)((read_val > 31) ? read_val - 64 : read_val);
        lgw_reg_r(LGW_IQ_MISMATCH_A_PHI_COEFF, &read_val);
        (*cal_res).phi_a = (int8_t)((read_val > 31) ? read_val - 64 : read_val);
        lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0xD0);
        lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &read_val);
        (*cal_res).img_rej_a = (uint8_t)read_val;
        lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0xD2);
        lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &read_val);
        (*cal_res).debug[0] = (uint8_t)read_val;
        lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0xD3);
        lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &read_val);
        (*cal_res).debug[1] = (uint8_t)read_val;
        lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0xD4);
        lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &read_val);
        (*cal_res).debug[2] = (uint8_t)read_val;
    }
    if (cal_cmd & 0x02) {
        lgw_reg_r(LGW_IQ_MISMATCH_B_AMP_COEFF, &read_val);
        (*cal_res).amp_b = (int8_t)((read_val > 31) ? read_val - 64 : read_val);
        lgw_reg_r(LGW_IQ_MISMATCH_B_PHI_COEFF, &read_val);
        (*cal_res).phi_b = (int8_t)((read_val > 31) ? read_val - 64 : read_val);
        lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0xD1);
        lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &read_val);
        (*cal_res).img_rej_b = (uint8_t)read_val;
        lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0xD2);
        lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &read_val);
        (*cal_res).debug[0] = (uint8_t)read_val;
        lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0xD3);
        lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &read_val);
        (*cal_res).debug[1] = (uint8_t)read_val;
        lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0xD4);
        lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &read_val);
        (*cal_res).debug[2] = (uint8_t)read_val;
    }
    if (cal_cmd & 0x04) {
        for (i=0; i<=7; ++i) {
            lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0xA0+i);
            lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &read_val);
            (*cal_res).offset_i_a[i] = (int8_t)read_val;
            lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0xA8+i);
            lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &read_val);
            (*cal_res).offset_q_a[i] = (int8_t)read_val;
            lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0xC0+i);
            lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &read_val);
            (*cal_res).offset_rej_a[i] = (uint8_t)read_val;
            lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0xD2+i);
            lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &read_val);
            (*cal_res).debug[i] = (uint8_t)read_val;
        }
    }
    if (cal_cmd & 0x08) {
        for (i=0; i<=7; ++i) {
            lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0xB0+i);
            lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &read_val);
            (*cal_res).offset_i_b[i] = (int8_t)read_val;
            lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0xB8+i);
            lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &read_val);
            (*cal_res).offset_q_b[i] = (int8_t)read_val;
            lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0xC8+i);
            lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &read_val);
            (*cal_res).offset_rej_b[i] = (uint8_t)read_val;
            lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0xD2+i);
            lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &read_val);
            (*cal_res).debug[i] = (uint8_t)read_val;
        }
    }

    return cal_status;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int read_capture(int16_t *sig_i, int16_t *sig_q, int nb_samp) {

    uint8_t read_burst[4];
    uint16_t data_i_c2;
    uint16_t data_q_c2;
    int i;

    lgw_reg_w(LGW_CAPTURE_RAM_ADDR, 0);
    for (i=0 ; i<nb_samp ; i++) {
        lgw_reg_rb(LGW_CAPTURE_RAM_DATA, read_burst, 4);
        data_i_c2 = ((uint16_t)read_burst[3] << 4) + ((uint16_t)read_burst[2] >> 4);
        data_q_c2 = ((uint16_t)read_burst[1] << 4) + ((uint16_t)read_burst[0] >> 4);
        sig_i[i] = (int16_t)((data_i_c2 > 2047) ? data_i_c2 - 4096 : data_i_c2);
        sig_q[i] = (int16_t)((data_q_c2 > 2047) ? data_q_c2 - 4096 : data_q_c2);
    }

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint8_t get_img_rej(int16_t *sig_i, int16_t *sig_q, int nb_samp, double f_sig_norm) {

    int i;
    double phase;
    double corr_sig_i, corr_sig_q, corr_sig_abs;
    double corr_img_i, corr_img_q, corr_img_abs;
    double img_rej;

    corr_sig_i = 0;
    corr_sig_q = 0;
    corr_img_i = 0;
    corr_img_q = 0;

    for (i=0 ; i<nb_samp ; i++) {
        phase = 6.28318530717959*i*f_sig_norm;
        corr_sig_i += (double)sig_i[i]*cos( phase) - (double)sig_q[i]*sin( phase);
        corr_sig_q += (double)sig_q[i]*cos( phase) + (double)sig_i[i]*sin( phase);
        corr_img_i += (double)sig_i[i]*cos(-phase) - (double)sig_q[i]*sin(-phase);
        corr_img_q += (double)sig_q[i]*cos(-phase) + (double)sig_i[i]*sin(-phase);
    }

    corr_sig_abs = sqrt( corr_sig_i*corr_sig_i + corr_sig_q*corr_sig_q );
    corr_img_abs = sqrt( corr_img_i*corr_img_i + corr_img_q*corr_img_q );

    img_rej = 20*log10(corr_sig_abs/corr_img_abs);

    return (uint8_t)img_rej;
}


/* --- EOF ------------------------------------------------------------------ */
