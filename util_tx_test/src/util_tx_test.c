/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
    Send a bunch of packets on a settable frequency

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

#include <string.h>     /* memset */
#include <signal.h>     /* sigaction */
#include <unistd.h>     /* getopt access */
#include <stdlib.h>     /* exit codes */
#include <getopt.h>     /* getopt_long */

#include "loragw_hal.h"
#include "loragw_reg.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define MSG(args...) fprintf(stderr, args) /* message that is destined to the user */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define TX_RF_CHAIN                 0 /* TX only supported on radio A */
#define DEFAULT_RSSI_OFFSET         0.0
#define DEFAULT_MODULATION          "LORA"
#define DEFAULT_BR_KBPS             50
#define DEFAULT_FDEV_KHZ            25
#define DEFAULT_NOTCH_FREQ          129000U /* 129 kHz */
#define DEFAULT_SX127X_RSSI_OFFSET  -4 /* dB */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES (GLOBAL) ------------------------------------------- */

/* signal handling variables */
struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */
static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

/* TX gain LUT table */
static struct lgw_tx_gain_lut_s txgain_lut = {
    .size = 5,
    .lut[0] = {
        .dig_gain = 0,
        .pa_gain = 0,
        .dac_gain = 3,
        .mix_gain = 12,
        .rf_power = 0
    },
    .lut[1] = {
        .dig_gain = 0,
        .pa_gain = 1,
        .dac_gain = 3,
        .mix_gain = 12,
        .rf_power = 10
    },
    .lut[2] = {
        .dig_gain = 0,
        .pa_gain = 2,
        .dac_gain = 3,
        .mix_gain = 10,
        .rf_power = 14
    },
    .lut[3] = {
        .dig_gain = 0,
        .pa_gain = 3,
        .dac_gain = 3,
        .mix_gain = 9,
        .rf_power = 20
    },
    .lut[4] = {
        .dig_gain = 0,
        .pa_gain = 3,
        .dac_gain = 3,
        .mix_gain = 14,
        .rf_power = 27
    }};

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
    int i;

    printf("*** Library version information ***\n%s\n\n", lgw_version_info());
    printf("Available options:\n");
    printf(" -h                 print this help\n");
    printf(" -r         <int>   radio type (SX1255:1255, SX1257:1257)\n");
    printf(" -n         <uint>  TX notch filter frequency in kHz [126..250]\n");
    printf(" -f         <float> target frequency in MHz\n");
    printf(" -k         <uint>  concentrator clock source (0:Radio A, 1:Radio B)\n");
    printf(" -m         <str>   modulation type ['LORA', 'FSK']\n");
    printf(" -b         <uint>  LoRa bandwidth in kHz [125, 250, 500]\n");
    printf(" -s         <uint>  LoRa Spreading Factor [7-12]\n");
    printf(" -c         <uint>  LoRa Coding Rate [1-4]\n");
    printf(" -d         <uint>  FSK frequency deviation in kHz [1:250]\n");
    printf(" -q         <float> FSK bitrate in kbps [0.5:250]\n");
    printf(" -p         <int>   RF power (dBm) [ ");
    for (i = 0; i < txgain_lut.size; i++) {
        printf("%ddBm ", txgain_lut.lut[i].rf_power);
    }
    printf("]\n");
    printf(" -l         <uint>  LoRa preamble length (symbols)\n");
    printf(" -z         <uint>  payload size (bytes, <256)\n");
    printf(" -i                 send packet using inverted modulation polarity\n");
    printf(" -t         <uint>  pause between packets (ms)\n");
    printf(" -x         <int>   nb of times the sequence is repeated (-1 loop until stopped)\n");
    printf(" --lbt-freq         <float> lbt first channel frequency in MHz\n");
    printf(" --lbt-nbch         <uint>  lbt number of channels [1..8]\n");
    printf(" --lbt-sctm         <uint>  lbt scan time in usec to be applied to all channels [128, 5000]\n");
    printf(" --lbt-rssi         <int>   lbt rssi target in dBm [-128..0]\n");
    printf(" --lbt-rssi-offset  <int>   rssi offset in dB to be applied to SX127x RSSI [-128..127]\n");
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv)
{
    int i;
    uint8_t status_var;

    /* user entry parameters */
    int xi = 0;
    unsigned int xu = 0;
    double xd = 0.0;
    float xf = 0.0;
    char arg_s[64];

    /* application parameters */
    char mod[64] = DEFAULT_MODULATION;
    uint32_t f_target = 0; /* target frequency - invalid default value, has to be specified by user */
    int sf = 10; /* SF10 by default */
    int cr = 1; /* CR1 aka 4/5 by default */
    int bw = 125; /* 125kHz bandwidth by default */
    int pow = 14; /* 14 dBm by default */
    int preamb = 8; /* 8 symbol preamble by default */
    int pl_size = 16; /* 16 bytes payload by default */
    int delay = 1000; /* 1 second between packets by default */
    int repeat = -1; /* by default, repeat until stopped */
    bool invert = false;
    float br_kbps = DEFAULT_BR_KBPS;
    uint8_t fdev_khz = DEFAULT_FDEV_KHZ;
    bool lbt_enable = false;
    uint32_t lbt_f_target = 0;
    uint32_t lbt_sc_time = 5000;
    int8_t lbt_rssi_target_dBm = -80;
    int8_t lbt_rssi_offset_dB = DEFAULT_SX127X_RSSI_OFFSET;
    uint8_t  lbt_nb_channel = 1;
    uint32_t sx1301_count_us;
    uint32_t tx_notch_freq = DEFAULT_NOTCH_FREQ;

    /* RF configuration (TX fail if RF chain is not enabled) */
    enum lgw_radio_type_e radio_type = LGW_RADIO_TYPE_NONE;
    uint8_t clocksource = 1; /* Radio B is source by default */
    struct lgw_conf_board_s boardconf;
    struct lgw_conf_lbt_s lbtconf;
    struct lgw_conf_rxrf_s rfconf;

    /* allocate memory for packet sending */
    struct lgw_pkt_tx_s txpkt; /* array containing 1 outbound packet + metadata */

    /* loop variables (also use as counters in the packet payload) */
    uint16_t cycle_count = 0;

    /* Parameter parsing */
    int option_index = 0;
    static struct option long_options[] = {
        {"lbt-freq", required_argument, 0, 0},
        {"lbt-sctm", required_argument, 0, 0},
        {"lbt-rssi", required_argument, 0, 0},
        {"lbt-nbch", required_argument, 0, 0},
        {"lbt-rssi-offset", required_argument, 0, 0},
        {0, 0, 0, 0}
    };

    /* parse command line options */
    while ((i = getopt_long (argc, argv, "hif:n:m:b:s:c:p:l:z:t:x:r:k:d:q:", long_options, &option_index)) != -1) {
        switch (i) {
            case 'h':
                usage();
                return EXIT_FAILURE;
                break;

            case 'f': /* <float> Target frequency in MHz */
                i = sscanf(optarg, "%lf", &xd);
                if ((i != 1) || (xd < 30.0) || (xd > 3000.0)) {
                    MSG("ERROR: invalid TX frequency\n");
                    usage();
                    return EXIT_FAILURE;
                } else {
                    f_target = (uint32_t)((xd*1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
                }
                break;

            case 'n': /* <uint> TX notch filter frequency in kHz */
                i = sscanf(optarg, "%i", &xi);
                if ((i != 1) || ((xi < 126) || (xi > 250))) {
                    MSG("ERROR: invalid TX notch filter frequency\n");
                    usage();
                    return EXIT_FAILURE;
                } else {
                    tx_notch_freq = xi*1000;
                }
                break;

            case 'm': /* <str> Modulation type */
                i = sscanf(optarg, "%s", arg_s);
                if ((i != 1) || ((strcmp(arg_s,"LORA") != 0) && (strcmp(arg_s,"FSK")))) {
                    MSG("ERROR: invalid modulation type\n");
                    usage();
                    return EXIT_FAILURE;
                } else {
                    sprintf(mod, "%s", arg_s);
                }
                break;

            case 'b': /* <int> Modulation bandwidth in kHz */
                i = sscanf(optarg, "%i", &xi);
                if ((i != 1) || ((xi != 125) && (xi != 250) && (xi != 500))) {
                    MSG("ERROR: invalid LoRa bandwidth\n");
                    usage();
                    return EXIT_FAILURE;
                } else {
                    bw = xi;
                }
                break;

            case 's': /* <int> Spreading Factor */
                i = sscanf(optarg, "%i", &xi);
                if ((i != 1) || (xi < 7) || (xi > 12)) {
                    MSG("ERROR: invalid spreading factor\n");
                    usage();
                    return EXIT_FAILURE;
                } else {
                    sf = xi;
                }
                break;

            case 'c': /* <int> Coding Rate */
                i = sscanf(optarg, "%i", &xi);
                if ((i != 1) || (xi < 1) || (xi > 4)) {
                    MSG("ERROR: invalid coding rate\n");
                    usage();
                    return EXIT_FAILURE;
                } else {
                    cr = xi;
                }
                break;

            case 'p': /* <int> RF power */
                i = sscanf(optarg, "%i", &xi);
                if ((i != 1) || (xi < -60) || (xi > 60)) {
                    MSG("ERROR: invalid RF power\n");
                    usage();
                    return EXIT_FAILURE;
                } else {
                    pow = xi;
                }
                break;

            case 'd': /* <uint> FSK frequency deviation */
                i = sscanf(optarg, "%u", &xu);
                if ((i != 1) || (xu < 1) || (xu > 250)) {
                    MSG("ERROR: invalid FSK frequency deviation\n");
                    usage();
                    return EXIT_FAILURE;
                } else {
                    fdev_khz = (uint8_t)xu;
                }
                break;

            case 'q': /* <float> FSK bitrate */
                i = sscanf(optarg, "%f", &xf);
                if ((i != 1) || (xf < 0.5) || (xf > 250)) {
                    MSG("ERROR: invalid FSK bitrate\n");
                    usage();
                    return EXIT_FAILURE;
                } else {
                    br_kbps = xf;
                }
                break;

            case 'l': /* <uint> preamble length (symbols) */
                i = sscanf(optarg, "%i", &xi);
                if ((i != 1) || (xi < 6)) {
                    MSG("ERROR: preamble length must be >6 symbols \n");
                    usage();
                    return EXIT_FAILURE;
                } else {
                    preamb = xi;
                }
                break;

            case 'z': /* <uint> payload length (bytes) */
                i = sscanf(optarg, "%i", &xi);
                if ((i != 1) || (xi <= 0)) {
                    MSG("ERROR: invalid payload size\n");
                    usage();
                    return EXIT_FAILURE;
                } else {
                    pl_size = xi;
                }
                break;

            case 't': /* <int> pause between packets (ms) */
                i = sscanf(optarg, "%i", &xi);
                if ((i != 1) || (xi < 0)) {
                    MSG("ERROR: invalid time between packets\n");
                    usage();
                    return EXIT_FAILURE;
                } else {
                    delay = xi;
                }
                break;

            case 'x': /* <int> numbers of times the sequence is repeated */
                i = sscanf(optarg, "%i", &xi);
                if ((i != 1) || (xi < -1)) {
                    MSG("ERROR: invalid number of repeats\n");
                    usage();
                    return EXIT_FAILURE;
                } else {
                    repeat = xi;
                }
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
                        return EXIT_FAILURE;
                }
                break;

            case 'i': /* Send packet using inverted modulation polarity */
                invert = true;
                break;

            case 'k': /* <int> Concentrator clock source (Radio A or Radio B) */
                i = sscanf(optarg, "%i", &xi);
                if ((i != 1) || ((xi != 0) && (xi != 1))) {
                    MSG("ERROR: invalid clock source\n");
                    usage();
                    return EXIT_FAILURE;
                } else {
                    clocksource = (uint8_t)xi;
                }
                break;

            case 0:
                if( strcmp(long_options[option_index].name, "lbt-freq") == 0 ) { /* <float> LBT first channel frequency in MHz */
                    i = sscanf(optarg, "%lf", &xd);
                    if ((i != 1) || (xd < 30.0) || (xd > 3000.0)) {
                        MSG("ERROR: invalid LBT start frequency\n");
                        usage();
                        return EXIT_FAILURE;
                    } else {
                        lbt_f_target = (uint32_t)((xd*1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
                        lbt_enable = true;
                    }
                } else if( strcmp(long_options[option_index].name, "lbt-sctm") == 0 ) { /* <int> LBT scan time in usec */
                    if (lbt_enable == true) {
                        i = sscanf(optarg, "%i", &xi);
                        if ((i != 1) || (xi < 0)) {
                            MSG("ERROR: invalid LBT scan time\n");
                            usage();
                            return EXIT_FAILURE;
                        } else {
                            lbt_sc_time = xi;
                        }
                    } else {
                        MSG("ERROR: invalid parameter, LBT start frequency must be set\n");
                        usage();
                        return EXIT_FAILURE;
                    }
                } else if( strcmp(long_options[option_index].name, "lbt-rssi") == 0 ) { /* <int> LBT RSSI target */
                    if (lbt_enable == true) {
                        i = sscanf(optarg, "%i", &xi);
                        if ((i != 1) || ((xi < -128) && (xi > 0))) {
                            MSG("ERROR: invalid LBT RSSI target\n");
                            usage();
                            return EXIT_FAILURE;
                        } else {
                            lbt_rssi_target_dBm = xi;
                        }
                    } else {
                        MSG("ERROR: invalid parameter, LBT start frequency must be set\n");
                        usage();
                        return EXIT_FAILURE;
                    }
                } else if( strcmp(long_options[option_index].name, "lbt-rssi-offset") == 0 ) { /* <int> LBT RSSI offset */
                    if (lbt_enable == true) {
                        i = sscanf(optarg, "%i", &xi);
                        if ((i != 1) || ((xi < -128) && (xi > 127))) {
                            MSG("ERROR: invalid LBT RSSI offset\n");
                            usage();
                            return EXIT_FAILURE;
                        } else {
                            lbt_rssi_offset_dB = xi;
                        }
                    } else {
                        MSG("ERROR: invalid parameter, LBT start frequency must be set\n");
                        usage();
                        return EXIT_FAILURE;
                    }
                } else if( strcmp(long_options[option_index].name, "lbt-nbch") == 0 ) { /* <int> LBT number of channels */
                    if (lbt_enable == true) {
                        i = sscanf(optarg, "%i", &xi);
                        if ((i != 1) || (xi < 0)) {
                            MSG("ERROR: invalid LBT number of channels\n");
                            usage();
                            return EXIT_FAILURE;
                        } else {
                            lbt_nb_channel = xi;
                        }
                    } else {
                        MSG("ERROR: invalid parameter, LBT start frequency must be set\n");
                        usage();
                        return EXIT_FAILURE;
                    }
                }
                break;
            default:
                MSG("ERROR: argument parsing\n");
                usage();
                return EXIT_FAILURE;
        }
    }

    /* check parameter sanity */
    if (f_target == 0) {
        MSG("ERROR: frequency parameter not set, please use -f option to specify it.\n");
        return EXIT_FAILURE;
    }
    if (radio_type == LGW_RADIO_TYPE_NONE) {
        MSG("ERROR: radio type parameter not properly set, please use -r option to specify it.\n");
        return EXIT_FAILURE;
    }

    /* Summary of packet parameters */
    if (strcmp(mod, "FSK") == 0) {
        printf("Sending %i FSK packets on %u Hz (FDev %u kHz, Bitrate %.2f, %i bytes payload, %i symbols preamble) at %i dBm, with %i ms between each\n", repeat, f_target, fdev_khz, br_kbps, pl_size, preamb, pow, delay);
    } else {
        printf("Sending %i LoRa packets on %u Hz (BW %i kHz, SF %i, CR %i, %i bytes payload, %i symbols preamble) at %i dBm, with %i ms between each\n", repeat, f_target, bw, sf, cr, pl_size, preamb, pow, delay);
    }

    /* configure signal handling */
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigact.sa_handler = sig_handler;
    sigaction(SIGQUIT, &sigact, NULL);
    sigaction(SIGINT, &sigact, NULL);
    sigaction(SIGTERM, &sigact, NULL);

    /* starting the concentrator */
    /* board config */
    memset(&boardconf, 0, sizeof(boardconf));
    boardconf.lorawan_public = true;
    boardconf.clksrc = clocksource;
    lgw_board_setconf(boardconf);

    /* LBT config */
    if (lbt_enable) {
        memset(&lbtconf, 0, sizeof(lbtconf));
        lbtconf.enable = true;
        lbtconf.nb_channel = lbt_nb_channel;
        lbtconf.rssi_target = lbt_rssi_target_dBm;
        lbtconf.rssi_offset = lbt_rssi_offset_dB;
        lbtconf.channels[0].freq_hz = lbt_f_target;
        lbtconf.channels[0].scan_time_us = lbt_sc_time;
        for (i=1; i<lbt_nb_channel; i++) {
            lbtconf.channels[i].freq_hz = lbtconf.channels[i-1].freq_hz + 200E3; /* 200kHz offset for all channels */
            lbtconf.channels[i].scan_time_us = lbt_sc_time;
        }
        lgw_lbt_setconf(lbtconf);
    }

    /* RF config */
    memset(&rfconf, 0, sizeof(rfconf));
    rfconf.enable = true;
    rfconf.freq_hz = f_target;
    rfconf.rssi_offset = DEFAULT_RSSI_OFFSET;
    rfconf.type = radio_type;
    for (i = 0; i < LGW_RF_CHAIN_NB; i++) {
        if (i == TX_RF_CHAIN) {
            rfconf.tx_enable = true;
            rfconf.tx_notch_freq = tx_notch_freq;
        } else {
            rfconf.tx_enable = false;
        }
        lgw_rxrf_setconf(i, rfconf);
    }

    /* TX gain config */
    lgw_txgain_setconf(&txgain_lut);

    /* Start concentrator */
    i = lgw_start();
    if (i == LGW_HAL_SUCCESS) {
        MSG("INFO: concentrator started, packet can be sent\n");
    } else {
        MSG("ERROR: failed to start the concentrator\n");
        return EXIT_FAILURE;
    }

    /* fill-up payload and parameters */
    memset(&txpkt, 0, sizeof(txpkt));
    txpkt.freq_hz = f_target;
    if (lbt_enable == true) {
        txpkt.tx_mode = TIMESTAMPED;
    } else {
        txpkt.tx_mode = IMMEDIATE;
    }
    txpkt.rf_chain = TX_RF_CHAIN;
    txpkt.rf_power = pow;
    if( strcmp( mod, "FSK" ) == 0 ) {
        txpkt.modulation = MOD_FSK;
        txpkt.datarate = br_kbps * 1e3;
        txpkt.f_dev = fdev_khz;
    } else {
        txpkt.modulation = MOD_LORA;
        switch (bw) {
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
        switch (cr) {
            case 1: txpkt.coderate = CR_LORA_4_5; break;
            case 2: txpkt.coderate = CR_LORA_4_6; break;
            case 3: txpkt.coderate = CR_LORA_4_7; break;
            case 4: txpkt.coderate = CR_LORA_4_8; break;
            default:
                MSG("ERROR: invalid 'cr' variable\n");
                return EXIT_FAILURE;
        }
    }
    txpkt.invert_pol = invert;
    txpkt.preamble = preamb;
    txpkt.size = pl_size;
    strcpy((char *)txpkt.payload, "TEST**abcdefghijklmnopqrstuvwxyz#0123456789#ABCDEFGHIJKLMNOPQRSTUVWXYZ#0123456789#abcdefghijklmnopqrstuvwxyz#0123456789#ABCDEFGHIJKLMNOPQRSTUVWXYZ#0123456789#abcdefghijklmnopqrstuvwxyz#0123456789#ABCDEFGHIJKLMNOPQRSTUVWXYZ#0123456789#abcdefghijklmnopqrs#" ); /* abc.. is for padding */

    /* main loop */
    cycle_count = 0;
    while ((repeat == -1) || (cycle_count < repeat)) {
        ++cycle_count;

        /* refresh counters in payload (big endian, for readability) */
        txpkt.payload[4] = (uint8_t)(cycle_count >> 8); /* MSB */
        txpkt.payload[5] = (uint8_t)(cycle_count & 0x00FF); /* LSB */

        /* When LBT is enabled, immediate send is not allowed, so we need
            to set a timestamp to the packet */
        if (lbt_enable == true) {
            /* Get the current SX1301 time */
            lgw_reg_w(LGW_GPS_EN, 0);
            lgw_get_trigcnt(&sx1301_count_us);
            lgw_reg_w(LGW_GPS_EN, 1);

            /* Set packet timestamp to current time + few milliseconds */
            txpkt.count_us = sx1301_count_us + 50E3;
        }

        /* send packet */
        printf("Sending packet number %u ...", cycle_count);
        i = lgw_send(txpkt); /* non-blocking scheduling of TX packet */
        if (i == LGW_HAL_ERROR) {
            printf("ERROR\n");
            return EXIT_FAILURE;
        } else if (i == LGW_LBT_ISSUE ) {
            printf("Failed: Not allowed (LBT)\n");
        } else {
            /* wait for packet to finish sending */
            do {
                wait_ms(5);
                lgw_status(TX_STATUS, &status_var); /* get TX status */
            } while (status_var != TX_FREE);
            printf("OK\n");
        }

        /* wait inter-packet delay */
        wait_ms(delay);

        /* exit loop on user signals */
        if ((quit_sig == 1) || (exit_sig == 1)) {
            break;
        }
    }

    /* clean up before leaving */
    lgw_stop();

    printf("Exiting LoRa concentrator TX test program\n");
    return EXIT_SUCCESS;
}

/* --- EOF ------------------------------------------------------------------ */
