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

#include <stdint.h>        /* C99 types */
#include <stdbool.h>       /* bool type */
#include <stdio.h>         /* printf */
#include <string.h>        /* memset */
#include <signal.h>        /* sigaction */
#include <unistd.h>        /* getopt access */

#include "loragw_hal.h"
#include "loragw_reg.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define DEFAULT_RSSI_OFFSET 0.0

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

/* describe command line options */
void usage(void) {
    printf("Library version information: %s\n", lgw_version_info());
    printf( "Available options:\n");
    printf( " -h print this help\n");
    printf( " -a <float> Radio A RX frequency in MHz\n");
    printf( " -b <float> Radio B RX frequency in MHz\n");
    printf( " -t <float> Radio TX frequency in MHz\n");
    printf( " -r <int> Radio type (SX1255:1255, SX1257:1257)\n");
    printf( " -k <int> Concentrator clock source (0: radio_A, 1: radio_B(default))\n");
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv)
{
    struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */

    struct lgw_conf_board_s boardconf;
    struct lgw_conf_rxrf_s rfconf;
    struct lgw_conf_rxif_s ifconf;

    struct lgw_pkt_rx_s rxpkt[4]; /* array containing up to 4 inbound packets metadata */
    struct lgw_pkt_tx_s txpkt; /* configuration and metadata for an outbound packet */
    struct lgw_pkt_rx_s *p; /* pointer on a RX packet */

    int i, j;
    int nb_pkt;
    uint32_t fa = 0, fb = 0, ft = 0;
    enum lgw_radio_type_e radio_type = LGW_RADIO_TYPE_NONE;
    uint8_t clocksource = 1; /* Radio B is source by default */

    uint32_t tx_cnt = 0;
    unsigned long loop_cnt = 0;
    uint8_t status_var = 0;
    double xd = 0.0;
    int xi = 0;

    /* parse command line options */
    while ((i = getopt (argc, argv, "ha:b:t:r:k:")) != -1) {
        switch (i) {
            case 'h':
                usage();
                return -1;
                break;
            case 'a': /* <float> Radio A RX frequency in MHz */
                sscanf(optarg, "%lf", &xd);
                fa = (uint32_t)((xd*1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
                break;
            case 'b': /* <float> Radio B RX frequency in MHz */
                sscanf(optarg, "%lf", &xd);
                fb = (uint32_t)((xd*1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
                break;
            case 't': /* <float> Radio TX frequency in MHz */
                sscanf(optarg, "%lf", &xd);
                ft = (uint32_t)((xd*1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
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
            case 'k': /* <int> Concentrator clock source (Radio A or Radio B) */
                sscanf(optarg, "%i", &xi);
                clocksource = (uint8_t)xi;
                break;
            default:
                printf("ERROR: argument parsing\n");
                usage();
                return -1;
        }
    }

    /* check input parameters */
    if ((fa == 0) || (fb == 0) || (ft == 0)) {
        printf("ERROR: missing frequency input parameter:\n");
        printf("  Radio A RX: %u\n", fa);
        printf("  Radio B RX: %u\n", fb);
        printf("  Radio TX: %u\n", ft);
        usage();
        return -1;
    }

    if (radio_type == LGW_RADIO_TYPE_NONE) {
        printf("ERROR: missing radio type parameter:\n");
        usage();
        return -1;
    }

    /* configure signal handling */
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigact.sa_handler = sig_handler;
    sigaction(SIGQUIT, &sigact, NULL);
    sigaction(SIGINT, &sigact, NULL);
    sigaction(SIGTERM, &sigact, NULL);

    /* beginning of LoRa concentrator-specific code */
    printf("Beginning of test for loragw_hal.c\n");

    printf("*** Library version information ***\n%s\n\n", lgw_version_info());

    /* set configuration for board */
    memset(&boardconf, 0, sizeof(boardconf));

    boardconf.lorawan_public = true;
    boardconf.clksrc = clocksource;
    lgw_board_setconf(boardconf);

    /* set configuration for RF chains */
    memset(&rfconf, 0, sizeof(rfconf));

    rfconf.enable = true;
    rfconf.freq_hz = fa;
    rfconf.rssi_offset = DEFAULT_RSSI_OFFSET;
    rfconf.type = radio_type;
    rfconf.tx_enable = true;
    lgw_rxrf_setconf(0, rfconf); /* radio A, f0 */

    rfconf.enable = true;
    rfconf.freq_hz = fb;
    rfconf.rssi_offset = DEFAULT_RSSI_OFFSET;
    rfconf.type = radio_type;
    rfconf.tx_enable = false;
    lgw_rxrf_setconf(1, rfconf); /* radio B, f1 */

    /* set configuration for LoRa multi-SF channels (bandwidth cannot be set) */
    memset(&ifconf, 0, sizeof(ifconf));

    ifconf.enable = true;
    ifconf.rf_chain = 1;
    ifconf.freq_hz = -400000;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(0, ifconf); /* chain 0: LoRa 125kHz, all SF, on f1 - 0.4 MHz */

    ifconf.enable = true;
    ifconf.rf_chain = 1;
    ifconf.freq_hz = -200000;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(1, ifconf); /* chain 1: LoRa 125kHz, all SF, on f1 - 0.2 MHz */

    ifconf.enable = true;
    ifconf.rf_chain = 1;
    ifconf.freq_hz = 0;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(2, ifconf); /* chain 2: LoRa 125kHz, all SF, on f1 - 0.0 MHz */

    ifconf.enable = true;
    ifconf.rf_chain = 0;
    ifconf.freq_hz = -400000;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(3, ifconf); /* chain 3: LoRa 125kHz, all SF, on f0 - 0.4 MHz */

    ifconf.enable = true;
    ifconf.rf_chain = 0;
    ifconf.freq_hz = -200000;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(4, ifconf); /* chain 4: LoRa 125kHz, all SF, on f0 - 0.2 MHz */

    ifconf.enable = true;
    ifconf.rf_chain = 0;
    ifconf.freq_hz = 0;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(5, ifconf); /* chain 5: LoRa 125kHz, all SF, on f0 + 0.0 MHz */

    ifconf.enable = true;
    ifconf.rf_chain = 0;
    ifconf.freq_hz = 200000;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(6, ifconf); /* chain 6: LoRa 125kHz, all SF, on f0 + 0.2 MHz */

    ifconf.enable = true;
    ifconf.rf_chain = 0;
    ifconf.freq_hz = 400000;
    ifconf.datarate = DR_LORA_MULTI;
    lgw_rxif_setconf(7, ifconf); /* chain 7: LoRa 125kHz, all SF, on f0 + 0.4 MHz */

    /* set configuration for LoRa 'stand alone' channel */
    memset(&ifconf, 0, sizeof(ifconf));
    ifconf.enable = true;
    ifconf.rf_chain = 0;
    ifconf.freq_hz = 0;
    ifconf.bandwidth = BW_250KHZ;
    ifconf.datarate = DR_LORA_SF10;
    lgw_rxif_setconf(8, ifconf); /* chain 8: LoRa 250kHz, SF10, on f0 MHz */

    /* set configuration for FSK channel */
    memset(&ifconf, 0, sizeof(ifconf));
    ifconf.enable = true;
    ifconf.rf_chain = 1;
    ifconf.freq_hz = 0;
    ifconf.bandwidth = BW_250KHZ;
    ifconf.datarate = 64000;
    lgw_rxif_setconf(9, ifconf); /* chain 9: FSK 64kbps, on f1 MHz */

    /* set configuration for TX packet */
    memset(&txpkt, 0, sizeof(txpkt));
    txpkt.freq_hz = ft;
    txpkt.tx_mode = IMMEDIATE;
    txpkt.rf_power = 10;
    txpkt.modulation = MOD_LORA;
    txpkt.bandwidth = BW_125KHZ;
    txpkt.datarate = DR_LORA_SF9;
    txpkt.coderate = CR_LORA_4_5;
    strcpy((char *)txpkt.payload, "TX.TEST.LORA.GW.????" );
    txpkt.size = 20;
    txpkt.preamble = 6;
    txpkt.rf_chain = 0;
/*
    memset(&txpkt, 0, sizeof(txpkt));
    txpkt.freq_hz = F_TX;
    txpkt.tx_mode = IMMEDIATE;
    txpkt.rf_power = 10;
    txpkt.modulation = MOD_FSK;
    txpkt.f_dev = 50;
    txpkt.datarate = 64000;
    strcpy((char *)txpkt.payload, "TX.TEST.LORA.GW.????" );
    txpkt.size = 20;
    txpkt.preamble = 4;
    txpkt.rf_chain = 0;
*/

    /* connect, configure and start the LoRa concentrator */
    i = lgw_start();
    if (i == LGW_HAL_SUCCESS) {
        printf("*** Concentrator started ***\n");
    } else {
        printf("*** Impossible to start concentrator ***\n");
        return -1;
    }

    /* once configured, dump content of registers to a file, for reference */
    // FILE * reg_dump = NULL;
    // reg_dump = fopen("reg_dump.log", "w");
    // if (reg_dump != NULL) {
        // lgw_reg_check(reg_dump);
        // fclose(reg_dump);
    // }

    while ((quit_sig != 1) && (exit_sig != 1)) {
        loop_cnt++;

        /* fetch N packets */
        nb_pkt = lgw_receive(ARRAY_SIZE(rxpkt), rxpkt);

        if (nb_pkt == 0) {
            wait_ms(300);
        } else {
            /* display received packets */
            for(i=0; i < nb_pkt; ++i) {
                p = &rxpkt[i];
                printf("---\nRcv pkt #%d >>", i+1);
                if (p->status == STAT_CRC_OK) {
                    printf(" if_chain:%2d", p->if_chain);
                    printf(" tstamp:%010u", p->count_us);
                    printf(" size:%3u", p->size);
                    switch (p-> modulation) {
                        case MOD_LORA: printf(" LoRa"); break;
                        case MOD_FSK: printf(" FSK"); break;
                        default: printf(" modulation?");
                    }
                    switch (p->datarate) {
                        case DR_LORA_SF7: printf(" SF7"); break;
                        case DR_LORA_SF8: printf(" SF8"); break;
                        case DR_LORA_SF9: printf(" SF9"); break;
                        case DR_LORA_SF10: printf(" SF10"); break;
                        case DR_LORA_SF11: printf(" SF11"); break;
                        case DR_LORA_SF12: printf(" SF12"); break;
                        default: printf(" datarate?");
                    }
                    switch (p->coderate) {
                        case CR_LORA_4_5: printf(" CR1(4/5)"); break;
                        case CR_LORA_4_6: printf(" CR2(2/3)"); break;
                        case CR_LORA_4_7: printf(" CR3(4/7)"); break;
                        case CR_LORA_4_8: printf(" CR4(1/2)"); break;
                        default: printf(" coderate?");
                    }
                    printf("\n");
                    printf(" RSSI:%+6.1f SNR:%+5.1f (min:%+5.1f, max:%+5.1f) payload:\n", p->rssi, p->snr, p->snr_min, p->snr_max);

                    for (j = 0; j < p->size; ++j) {
                        printf(" %02X", p->payload[j]);
                    }
                    printf(" #\n");
                } else if (p->status == STAT_CRC_BAD) {
                    printf(" if_chain:%2d", p->if_chain);
                    printf(" tstamp:%010u", p->count_us);
                    printf(" size:%3u\n", p->size);
                    printf(" CRC error, damaged packet\n\n");
                } else if (p->status == STAT_NO_CRC){
                    printf(" if_chain:%2d", p->if_chain);
                    printf(" tstamp:%010u", p->count_us);
                    printf(" size:%3u\n", p->size);
                    printf(" no CRC\n\n");
                } else {
                    printf(" if_chain:%2d", p->if_chain);
                    printf(" tstamp:%010u", p->count_us);
                    printf(" size:%3u\n", p->size);
                    printf(" invalid status ?!?\n\n");
                }
            }
        }

        /* send a packet every X loop */
        if (loop_cnt%16 == 0) {
            /* 32b counter in the payload, big endian */
            txpkt.payload[16] = 0xff & (tx_cnt >> 24);
            txpkt.payload[17] = 0xff & (tx_cnt >> 16);
            txpkt.payload[18] = 0xff & (tx_cnt >> 8);
            txpkt.payload[19] = 0xff & tx_cnt;
            i = lgw_send(txpkt); /* non-blocking scheduling of TX packet */
            j = 0;
            printf("+++\nSending packet #%d, rf path %d, return %d\nstatus -> ", tx_cnt, txpkt.rf_chain, i);
            do {
                ++j;
                wait_ms(100);
                lgw_status(TX_STATUS, &status_var); /* get TX status */
                printf("%d:", status_var);
            } while ((status_var != TX_FREE) && (j < 100));
            ++tx_cnt;
            printf("\nTX finished\n");
        }
    }

    if (exit_sig == 1) {
        /* clean up before leaving */
        lgw_stop();
    }

    printf("\nEnd of test for loragw_hal.c\n");
    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
