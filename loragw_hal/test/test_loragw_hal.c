/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    ©2013 Semtech-Cycleo

Description:
	Minimum test program for the loragw_hal 'library'
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

/* fix an issue between POSIX and C99 */
#if __STDC_VERSION__ >= 199901L
	#define _XOPEN_SOURCE 600
#else
	#define _XOPEN_SOURCE 500
#endif

#include <stdint.h>		/* C99 types */
#include <stdlib.h>		/* malloc & free */
#include <stdbool.h>	/* bool type */
#include <stdio.h>		/* printf */
#include <string.h>		/* memset */
#include <signal.h>		/* sigaction */

#include "loragw_hal.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

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

int main(int argc, char **argv)
{
	struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */
	
	struct lgw_conf_rxrf_s rfconf;
	struct lgw_conf_rxif_s ifconf;
	
	struct lgw_pkt_rx_s rxpkt[4]; /* array containing up to 4 inbound packets metadata */
	struct lgw_pkt_tx_s txpkt; /* configuration and metadata for an outbound packet */
	uint8_t txbuf[256]; /* buffer for the TX payload */
	struct lgw_pkt_rx_s *p; /* pointer on a RX packet */
	
	int i, j;
	int nb_pkt;
	uint8_t x;
	
	int tx_cnt = 0;
	int tx_path = 0;
	struct lgw_pkt_tx_s txs;
	
	/* configure signal handling */
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigact.sa_handler = sig_handler;
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	
	/* beginning of Lora gateway-specific code */
	printf("Beginning of test for loragw_hal.c\n");
	
	/* set configuration for RF chains */
	memset(&rfconf, 0, sizeof(rfconf));
	
	rfconf.enable = true;
	rfconf.freq_hz = 866187500;
	lgw_rxrf_setconf(0, rfconf); /* radio A */
	
	rfconf.enable = true;
	rfconf.freq_hz = 866437500;
	lgw_rxrf_setconf(1, rfconf); /* radio B */
	
	/* set configuration for Lora multi-SF channels (bandwidth cannot be set) */
	memset(&ifconf, 0, sizeof(ifconf));
	
	ifconf.enable = true;
	ifconf.rf_chain = 0;
	ifconf.freq_hz = -187500;
	ifconf.bandwidth = BW_125KHZ;
	ifconf.datarate = DR_LORA_MULTI;
	lgw_rxif_setconf(0, ifconf); /* chain 0: 1st radio, bleeper channel 1, all SF */
	
	ifconf.enable = true;
	ifconf.rf_chain = 0;
	ifconf.freq_hz = -62500;
	ifconf.bandwidth = BW_125KHZ;
	ifconf.datarate = DR_LORA_SF8 | DR_LORA_SF10;
	lgw_rxif_setconf(1, ifconf); /* chain 1: 1st radio, bleeper channel 2, SF8 & SF10 only */
	
	ifconf.enable = false;
	ifconf.rf_chain = 0;
	ifconf.freq_hz = 0;
	ifconf.bandwidth = 0;
	ifconf.datarate = 0;
	lgw_rxif_setconf(2, ifconf); /* chain 2: 1st radio, disabled */
	
	ifconf.enable = true;
	ifconf.rf_chain = 1;
	ifconf.freq_hz = -187500;
	ifconf.bandwidth = BW_125KHZ;
	ifconf.datarate = DR_LORA_MULTI;
	lgw_rxif_setconf(3, ifconf); /* chain 3: 2nd radio, bleeper channel 3, all SF */
	
	/* set configuration for Lora 'stand alone' channel */
	ifconf.enable = true;
	ifconf.rf_chain = 0;
	ifconf.freq_hz = 187500;
	ifconf.bandwidth = BW_125KHZ;
	ifconf.datarate = DR_LORA_SF10;
	lgw_rxif_setconf(8, ifconf); /* chain 8: bleeper channel 4, SF10 only */
	
	/* set configuration for TX packet */
	memset(&txs, 0, sizeof(txs));
	txs.freq_hz = 866250000;
	txs.tx_mode = IMMEDIATE;
	txs.modulation = MOD_LORA;
	txs.bandwidth = BW_250KHZ;
	txs.datarate = DR_LORA_SF10;
	txs.coderate = CR_LORA_4_5;
	txs.payload = "TX.TEST.LORA.GATEWAY";
	txs.size = 20;
	
	/* connect, configure and start the Lora gateway */
	lgw_start();
	
	while ((quit_sig != 1) && (exit_sig != 1)) {
		/* fetch N packets */
		nb_pkt = lgw_receive(ARRAY_SIZE(rxpkt), rxpkt);
		
		if (nb_pkt == 0) {
			wait_ms(300);
		} else {
			printf("\nLora gateway, %d packets received:\n\n", nb_pkt);
			/* display received packets */
			for(i=0; i < nb_pkt; ++i) {
				p = &rxpkt[i];
				printf("---\nPkt #%d >>", i+1);
				if (p->status == STAT_CRC_OK) {
					printf(" if_chain:%2d", p->if_chain);
					printf(" tstamp:%010u", p->count_us);
					printf(" size:%3u", p->size);
					switch (p-> modulation) {
						case MOD_LORA: printf(" Lora"); break;
						case MOD_FSK: printf(" FSK"); break;
						case MOD_GFSK: printf(" GFSK"); break;
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
			
			/* free the memory used for RX payload(s) */
			for(i=0; i < nb_pkt; ++i) {
				free(rxpkt[i].payload);
			}
		}
		
		/* send a packet every X loop */
		if (tx_cnt >= 32) {
			tx_cnt = 0;
			
			txs.rf_chain = tx_path; /* alternate between path A and B */
			i = lgw_send(txs);
			printf("Packet sent, rf path %d, status %d\n", txs.rf_chain, i);
			
			tx_path = (tx_path+1) % 2;
		} else {
			++tx_cnt;
		}
	}
	
	if (exit_sig == 1) {
		/* clean up before leaving */
		lgw_stop();
	}
	
	printf("End of test for loragw_hal.c\n");
	return 0;
}

/* --- EOF ------------------------------------------------------------------ */
