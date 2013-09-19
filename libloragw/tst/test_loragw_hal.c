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
	struct lgw_pkt_rx_s *p; /* pointer on a RX packet */
	
	int i, j;
	int nb_pkt;
	uint8_t x;
	
	uint32_t tx_cnt = 0;
	unsigned long loop_cnt = 0;
	int tx_path = 0;
	struct lgw_pkt_tx_s txs;
	uint8_t status_var = 0;
	
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
	rfconf.freq_hz = 866000000;
	lgw_rxrf_setconf(0, rfconf); /* radio A */
	
	rfconf.enable = true;
	rfconf.freq_hz = 868000000;
	lgw_rxrf_setconf(1, rfconf); /* radio B */
	
	/* set configuration for Lora multi-SF channels (bandwidth cannot be set) */
	memset(&ifconf, 0, sizeof(ifconf));
	
	ifconf.enable = true;
	ifconf.rf_chain = 0;
	ifconf.freq_hz = -300000;
	ifconf.bandwidth = BW_125KHZ;
	ifconf.datarate = DR_LORA_MULTI;
	lgw_rxif_setconf(0, ifconf); /* chain 0: Lora 125kHz, all SF, on 865.7 MHz */
	
	ifconf.enable = true;
	ifconf.rf_chain = 0;
	ifconf.freq_hz = 300000;
	ifconf.bandwidth = BW_125KHZ;
	ifconf.datarate = DR_LORA_MULTI;
	lgw_rxif_setconf(1, ifconf); /* chain 1: Lora 125kHz, all SF, on 866.3 MHz */
	
	ifconf.enable = true;
	ifconf.rf_chain = 1;
	ifconf.freq_hz = -300000;
	ifconf.bandwidth = BW_125KHZ;
	ifconf.datarate = DR_LORA_MULTI;
	lgw_rxif_setconf(2, ifconf); /* chain 2: Lora 125kHz, all SF, on 867.7 MHz */
	
	ifconf.enable = true;
	ifconf.rf_chain = 1;
	ifconf.freq_hz = 300000;
	ifconf.bandwidth = BW_125KHZ;
	ifconf.datarate = DR_LORA_MULTI;
	lgw_rxif_setconf(3, ifconf); /* chain 3: Lora 125kHz, all SF, on 868.3 MHz */
	
	/* set configuration for Lora 'stand alone' channel */
	ifconf.enable = true;
	ifconf.rf_chain = 0;
	ifconf.freq_hz = 0;
	ifconf.bandwidth = BW_250KHZ;
	ifconf.datarate = DR_LORA_SF10;
	lgw_rxif_setconf(8, ifconf); /* chain 8: Lora 250kHz, SF10, on 866.0 MHz */
	
	/* set configuration for FSK channel */
	ifconf.enable = true;
	ifconf.rf_chain = 1;
	ifconf.freq_hz = 0;
	ifconf.bandwidth = BW_250KHZ;
	ifconf.datarate = 64000;
	lgw_rxif_setconf(9, ifconf); /* chain 9: FSK 64kbps, fdev 32kHz, variable payload, on 868.0 MHz */
	
	/* set configuration for TX packet */
	
	memset(&txs, 0, sizeof(txs));
	txs.freq_hz = 867000000;
	txs.tx_mode = IMMEDIATE;
	txs.modulation = MOD_LORA;
	txs.bandwidth = BW_250KHZ;
	txs.datarate = DR_LORA_SF10;
	txs.coderate = CR_LORA_4_5;
	strcpy((char *)txs.payload, "TX.TEST.LORA.GW.????" );
	txs.size = 20;
	txs.preamble = 6;
	txs.rf_chain = 0;
/*	
	memset(&txs, 0, sizeof(txs));
	txs.freq_hz = 867000000;
	txs.tx_mode = IMMEDIATE;
	txs.modulation = MOD_FSK;
	txs.f_dev = 50;
	txs.datarate = 64000;
	strcpy((char *)txs.payload, "TX.TEST.LORA.GW.????" );
	txs.size = 20;
	txs.preamble = 4;
	txs.rf_chain = 0;
*/	
	
//	printf("***\n%s\n***\n", lgw_version_info());
	
	/* connect, configure and start the Lora gateway */
	lgw_start();
	
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
						case MOD_LORA: printf(" Lora"); break;
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
			txs.payload[16] = 0xff & (tx_cnt >> 24);
			txs.payload[17] = 0xff & (tx_cnt >> 16);
			txs.payload[18] = 0xff & (tx_cnt >> 8);
			txs.payload[19] = 0xff & tx_cnt;
			i = lgw_send(txs); /* non-blocking scheduling of TX packet */
			j = 0;
			printf("+++\nSending packet #%d, rf path %d, return %d\nstatus -> ", tx_cnt, txs.rf_chain, i);
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
