/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2014 Semtech-Cycleo

Description:
    SX1301 tx continuous utility

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Matthieu Leurent
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
#include <stdbool.h>    /* bool type */
#include <stdio.h>      /* printf fprintf sprintf fopen fputs */
#include <string.h>     /* memset */
#include <signal.h>     /* sigaction */
#include <unistd.h>     /* getopt access */
#include <stdlib.h>     /* exit codes */
#include <getopt.h>     /* getopt_long */

#include "loragw_hal.h"
#include "loragw_reg.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- MACROS & CONSTANTS --------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define MSG(args...) fprintf(stderr, args) /* message that is destined to the user */

#define TX_RF_CHAIN             0    /* TX only supported on radio A */
#define DEFAULT_RSSI_OFFSET     0.0

#define DEFAULT_FREQ_HZ         868e6
#define DEFAULT_DIGITAL_GAIN    0
#define DEFAULT_DAC_GAIN        3
#define DEFAULT_MIXER_GAIN      14
#define DEFAULT_PA_GAIN         3
#define DEFAULT_MODULATION      "LORA"
#define DEFAULT_SF              7
#define DEFAULT_BW_KHZ          125
#define DEFAULT_BR_KBPS         50
#define DEFAULT_FDEV_KHZ        25
#define DEFAULT_BT              2
#define DEFAULT_NOTCH_FREQ      129000U

/* -------------------------------------------------------------------------- */
/* --- GLOBAL VARIABLES ----------------------------------------------------- */

/* Signal handling variables */
static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

/* -------------------------------------------------------------------------- */
/* --- SUBFUNCTIONS DECLARATION --------------------------------------------- */

static void sig_handler(int sigio);

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv)
{
    static struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */

    int i; /* loop and temporary variables */

    /* Parameter parsing */
    int option_index = 0;
    static struct option long_options[] = {
        {"dig", 1, 0, 0},
        {"dac", 1, 0, 0},
        {"mix", 1, 0, 0},
        {"pa", 1, 0, 0},
        {"mod", 1, 0, 0},
        {"sf", 1, 0, 0},
        {"bw", 1, 0, 0},
        {"br", 1, 0, 0},
        {"fdev", 1, 0, 0},
        {"bt", 1, 0, 0},
        {"notch", 1, 0, 0},
        {0, 0, 0, 0}
    };
    unsigned int arg_u;
    float arg_f;
    char arg_s[64];

    /* Application parameters */
    uint32_t freq_hz = DEFAULT_FREQ_HZ;
    uint8_t g_dig = DEFAULT_DIGITAL_GAIN;
    uint8_t g_dac = DEFAULT_DAC_GAIN;
    uint8_t g_mix = DEFAULT_MIXER_GAIN;
    uint8_t g_pa = DEFAULT_PA_GAIN;
    char mod[64] = DEFAULT_MODULATION;
    uint8_t sf = DEFAULT_SF;
    unsigned int bw_khz = DEFAULT_BW_KHZ;
    float br_kbps = DEFAULT_BR_KBPS;
    uint8_t fdev_khz = DEFAULT_FDEV_KHZ;
    uint8_t bt = DEFAULT_BT;
    uint32_t tx_notch_freq = DEFAULT_NOTCH_FREQ;

    int32_t offset_i, offset_q;

    /* RF configuration (TX fail if RF chain is not enabled) */
    enum lgw_radio_type_e radio_type = LGW_RADIO_TYPE_SX1257;
    struct lgw_conf_board_s boardconf;
    struct lgw_conf_rxrf_s rfconf;
    struct lgw_tx_gain_lut_s txlut;
    struct lgw_pkt_tx_s txpkt;


    /* Parse command line options */
    while ((i = getopt_long (argc, argv, "hud::f:r:", long_options, &option_index)) != -1) {
        switch (i) {
            case 'h':
                printf("~~~ Library version string~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
                printf(" %s\n", lgw_version_info());
                printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
                printf(" -f      <float>  Tx RF frequency in MHz [800:1000]\n");
                printf(" -r      <int>    Radio type (SX1255:1255, SX1257:1257)\n");
                printf(" --notch <uint>   Tx notch filter frequency in KhZ [126..250]\n");
                printf(" --dig   <uint>   Digital gain trim, [0:3]\n");
                printf("                   0:1, 1:7/8, 2:3/4, 3:1/2\n");
                printf(" --mix   <uint>   Radio Tx mixer gain trim, [0:15]\n");
                printf("                   15 corresponds to maximum gain, 1 LSB corresponds to 2dB step\n");
                printf(" --pa    <uint>   PA gain trim, [0:3]\n");
                printf(" --mod   <char>   Modulation type ['LORA','FSK','CW']\n");
                printf(" --sf    <uint>   LoRa Spreading Factor, [7:12]\n");
                printf(" --bw    <uint>   LoRa bandwidth in kHz, [125,250,500]\n");
                printf(" --br    <float>  FSK bitrate in kbps, [0.5:250]\n");
                printf(" --fdev  <uint>   FSK frequency deviation in kHz, [1:250]\n");
                printf(" --bt    <uint>   FSK gaussian filter BT trim, [0:3]\n");
                printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
                return EXIT_SUCCESS;
                break;

            case 0:
                if (strcmp(long_options[option_index].name,"dig") == 0) {
                    i = sscanf(optarg, "%u", &arg_u);
                    if ((i != 1) || (arg_u > 3)) {
                        printf("ERROR: argument parsing of --dig argument. Use -h to print help\n");
                        return EXIT_FAILURE;
                    }
                    else
                    {
                        g_dig = (uint8_t)arg_u;
                    }
                }
                else if (strcmp(long_options[option_index].name,"dac") == 0) {
                    i = sscanf(optarg, "%u", &arg_u);
                    if ((i != 1) || (arg_u > 3)) {
                        printf("ERROR: argument parsing of --dac argument. Use -h to print help\n");
                        return EXIT_FAILURE;
                    }
                    else {
                        g_dac = (uint8_t)arg_u;
                    }
                }
                else if (strcmp(long_options[option_index].name,"mix") == 0) {
                    i = sscanf(optarg, "%u", &arg_u);
                    if ((i != 1) || (arg_u > 15)) {
                        printf("ERROR: argument parsing of --mix argument. Use -h to print help\n");
                        return EXIT_FAILURE;
                    }
                    else {
                        g_mix = (uint8_t)arg_u;
                    }
                }
                else if (strcmp(long_options[option_index].name,"pa") == 0) {
                    i = sscanf(optarg, "%u", &arg_u);
                    if ((i != 1) || (arg_u > 3)) {
                        printf("ERROR: argument parsing of --pa argument. Use -h to print help\n");
                        return EXIT_FAILURE;
                    }
                    else {
                        g_pa = arg_u;
                    }
                }
                else if (strcmp(long_options[option_index].name,"mod") == 0) {
                    i = sscanf(optarg, "%s", arg_s);
                    if ((i != 1) || ((strcmp(arg_s,"LORA") != 0) && (strcmp(arg_s,"FSK") != 0)  && (strcmp(arg_s,"CW") != 0))) {
                        printf("ERROR: argument parsing of --mod argument. Use -h to print help\n");
                        return EXIT_FAILURE;
                    }
                    else {
                        sprintf(mod, "%s", arg_s);
                    }
                }
                else if (strcmp(long_options[option_index].name,"sf") == 0) {
                    i = sscanf(optarg, "%u", &arg_u);
                    if ((i != 1) || (arg_u < 7) || (arg_u > 12)) {
                        printf("ERROR: argument parsing of --sf argument. Use -h to print help\n");
                        return EXIT_FAILURE;
                    }
                    else {
                        sf = (uint8_t)arg_u;
                    }
                }
                else if (strcmp(long_options[option_index].name,"bw") == 0) {
                    i = sscanf(optarg, "%u", &arg_u);
                    if ((i != 1) || ((arg_u != 125) && (arg_u != 250) && (arg_u != 500))) {
                        printf("ERROR: argument parsing of --bw argument. Use -h to print help\n");
                        return EXIT_FAILURE;
                    }
                    else {
                        bw_khz = arg_u;
                    }
                }
                else if (strcmp(long_options[option_index].name,"br") == 0) {
                    i = sscanf(optarg, "%f", &arg_f);
                    if ((i != 1) || (arg_f < 0.5) || (arg_f > 250)) {
                        printf("ERROR: argument parsing of --br argument. Use -h to print help\n");
                        return EXIT_FAILURE;
                    }
                    else {
                        br_kbps = arg_f;
                    }
                }
                else if (strcmp(long_options[option_index].name,"fdev") == 0) {
                    i = sscanf(optarg, "%u", &arg_u);
                    if ((i != 1) || (arg_u < 1) || (arg_u > 250)) {
                        printf("ERROR: argument parsing of --fdev argument. Use -h to print help\n");
                        return EXIT_FAILURE;
                    }
                    else {
                        fdev_khz = (uint8_t)arg_u;
                    }
                }
                else if (strcmp(long_options[option_index].name,"bt") == 0) {
                    i = sscanf(optarg, "%u", &arg_u);
                    if ((i != 1) || (arg_u > 3)) {
                        printf("ERROR: argument parsing of --bt argument. Use -h to print help\n");
                        return EXIT_FAILURE;
                    }
                    else {
                        bt = (uint8_t)arg_u;
                    }
                }
                else if (strcmp(long_options[option_index].name,"notch") == 0) {
                    i = sscanf(optarg, "%u", &arg_u);
                    if ((i != 1) || ((arg_u < 126) || (arg_u > 250))) {
                        printf("ERROR: argument parsing of --notch argument. Use -h to print help\n");
                        return EXIT_FAILURE;
                    }
                    else {
                        tx_notch_freq = (uint32_t)arg_u * 1000U;
                    }
                }
                else {
                    printf("ERROR: argument parsing options. Use -h to print help\n");
                    return EXIT_FAILURE;
                }
                break;

        case 'f':
            i = sscanf(optarg, "%f", &arg_f);
            if ((i != 1) || (arg_f < 1)) {
                printf("ERROR: argument parsing of -f argument. Use -h to print help\n");
                return EXIT_FAILURE;
            }
            else {
                freq_hz = (uint32_t)((arg_f * 1e6) + 0.5);
            }
            break;

        case 'r':
            i = sscanf(optarg, "%u", &arg_u);
            switch (arg_u) {
                case 1255:
                    radio_type = LGW_RADIO_TYPE_SX1255;
                    break;
                case 1257:
                    radio_type = LGW_RADIO_TYPE_SX1257;
                    break;
                default:
                    printf("ERROR: argument parsing of -r argument. Use -h to print help\n");
                    return EXIT_FAILURE;
            }
            break;

        default:
            printf("ERROR: argument parsing options. Use -h to print help\n");
            return EXIT_FAILURE;
        }
    }

    /* Configure signal handling */
    sigemptyset( &sigact.sa_mask );
    sigact.sa_flags = 0;
    sigact.sa_handler = sig_handler;
    sigaction( SIGQUIT, &sigact, NULL );
    sigaction( SIGINT, &sigact, NULL );
    sigaction( SIGTERM, &sigact, NULL );

    /* Board config */
    memset(&boardconf, 0, sizeof(boardconf));
    boardconf.lorawan_public = true;
    boardconf.clksrc = 1; /* Radio B is source by default */
    lgw_board_setconf(boardconf);

    /* RF config */
    memset(&rfconf, 0, sizeof(rfconf));
    rfconf.enable = true;
    rfconf.freq_hz = freq_hz;
    rfconf.rssi_offset = DEFAULT_RSSI_OFFSET;
    rfconf.type = radio_type;
    rfconf.tx_enable = true;
    rfconf.tx_notch_freq = tx_notch_freq;
    lgw_rxrf_setconf(TX_RF_CHAIN, rfconf);

    /* Tx gain LUT */
    memset(&txlut, 0, sizeof txlut);
    txlut.size = 1;
    txlut.lut[0].dig_gain = g_dig;
    txlut.lut[0].pa_gain = g_pa;
    txlut.lut[0].dac_gain = g_dac;
    txlut.lut[0].mix_gain = g_mix;
    txlut.lut[0].rf_power = 0;
    lgw_txgain_setconf(&txlut);

    /* Start the concentrator */
    i = lgw_start();
    if (i == LGW_HAL_SUCCESS) {
        MSG("INFO: concentrator started, packet can be sent\n");
    } else {
        MSG("ERROR: failed to start the concentrator\n");
        return EXIT_FAILURE;
    }

    /* fill-up payload and parameters */
    memset(&txpkt, 0, sizeof(txpkt));
    txpkt.freq_hz = freq_hz;
    txpkt.tx_mode = IMMEDIATE;
    txpkt.rf_chain = TX_RF_CHAIN;
    txpkt.rf_power = 0;
    if (strcmp(mod, "FSK") == 0) {
        txpkt.modulation = MOD_FSK;
        txpkt.datarate = br_kbps * 1e3;
    } else {
        txpkt.modulation = MOD_LORA;
        switch (bw_khz) {
            case 125: txpkt.bandwidth = BW_125KHZ; break;
            case 250: txpkt.bandwidth = BW_250KHZ; break;
            case 500: txpkt.bandwidth = BW_500KHZ; break;
            default:
                MSG("ERROR: invalid 'bw' variable\n");
                return EXIT_FAILURE;
        }
        switch (sf) {
            case  7: txpkt.datarate = DR_LORA_SF7;  break;
            case  8: txpkt.datarate = DR_LORA_SF8;  break;
            case  9: txpkt.datarate = DR_LORA_SF9;  break;
            case 10: txpkt.datarate = DR_LORA_SF10; break;
            case 11: txpkt.datarate = DR_LORA_SF11; break;
            case 12: txpkt.datarate = DR_LORA_SF12; break;
            default:
                MSG("ERROR: invalid 'sf' variable\n");
                return EXIT_FAILURE;
        }
    }
    txpkt.coderate = CR_LORA_4_5;
    txpkt.f_dev = fdev_khz;
    txpkt.preamble = 65535;
    txpkt.invert_pol = false;
    txpkt.no_crc = true;
    txpkt.no_header = true;
    txpkt.size = 1;
    txpkt.payload[0] = 0;

    /* Overwrite settings */
    lgw_reg_w(LGW_TX_MODE, 1); /* Tx continuous */
    lgw_reg_w(LGW_FSK_TX_GAUSSIAN_SELECT_BT, bt);
    if (strcmp(mod, "CW") == 0) {
        /* Enable signal generator with DC */
        lgw_reg_w(LGW_SIG_GEN_FREQ, 0);
        lgw_reg_w(LGW_SIG_GEN_EN, 1);
        lgw_reg_w(LGW_TX_OFFSET_I, 0);
        lgw_reg_w(LGW_TX_OFFSET_Q, 0);
    }

    /* Send packet */
    i = lgw_send(txpkt);

    /* Recap all settings */
    printf("SX1301 library version: %s\n", lgw_version_info());
    if (strcmp(mod, "LORA") == 0) {
        printf("Modulation: LORA SF:%d BW:%d kHz\n", sf, bw_khz);
    }
    else if (strcmp(mod, "FSK") == 0) {
        printf("Modulation: FSK BR:%3.3f kbps FDEV:%d kHz BT:%d\n", br_kbps, fdev_khz, bt);
    }
    else if (strcmp(mod, "CW") == 0) {
        printf("Modulation: CW\n");
    }
    switch(rfconf.type) {
        case LGW_RADIO_TYPE_SX1255:
            printf("Radio Type: SX1255\n");
            break;
        case LGW_RADIO_TYPE_SX1257:
            printf("Radio Type: SX1257\n");
            break;
        default:
            printf("ERROR: undefined radio type\n");
            break;
    }
    printf("Frequency: %4.3f MHz\n", freq_hz/1e6);
    printf("TX Gains: Digital:%d DAC:%d Mixer:%d PA:%d\n", g_dig, g_dac, g_mix, g_pa);
    if (strcmp(mod, "CW") != 0) {
        lgw_reg_r(LGW_TX_OFFSET_I, &offset_i);
        lgw_reg_r(LGW_TX_OFFSET_Q, &offset_q);
        printf("Calibrated DC offsets: I:%d Q:%d\n", offset_i, offset_q);
    }

    /* waiting for user input */
    while ((quit_sig != 1) && (exit_sig != 1)) {
        wait_ms(100);
    }

    /* clean up before leaving */
    lgw_stop();

    return 0;
}

/* -------------------------------------------------------------------------- */
/* --- SUBFUNCTIONS DEFINITION ---------------------------------------------- */

static void sig_handler(int sigio)
{
    if (sigio == SIGQUIT) {
        quit_sig = 1;
    }
    else if((sigio == SIGINT) || (sigio == SIGTERM)) {
        exit_sig = 1;
    }
}

/* --- EOF ------------------------------------------------------------------ */
