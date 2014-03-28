/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
	Configure LoRa concentrator board and record received packets in a log file

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

#include <stdint.h>		/* C99 types */
#include <stdbool.h>	/* bool type */
#include <stdio.h>		/* fprintf sprintf fopen */

#include <signal.h>		/* sigaction */
#include <time.h>		/* time strftime gmtime */
#include <unistd.h>		/* getopt */
#include <stdlib.h>		/* EXIT_* constants */

#include "loragw_hal.h" /* only for min and max frequency constants */
#include "loragw_reg.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a)	(sizeof(a) / sizeof((a)[0]))
#define MSG(args...)	fprintf(stderr, args)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define		MCU_ARB		0
#define		MCU_AGC		1

const uint32_t rf_rx_lowfreq_[LGW_RF_CHAIN_NB] = LGW_RF_RX_LOWFREQ;
const uint32_t rf_rx_upfreq_[LGW_RF_CHAIN_NB] = LGW_RF_RX_UPFREQ;

#define		MCU_ARB_FW_BYTE		8192 /* size of the firmware IN BYTES (= twice the number of 14b words) */
#define		MCU_AGC_FW_BYTE		8192 /* size of the firmware IN BYTES (= twice the number of 14b words) */

/*
SX1257 frequency setting :
F_register(24bit) = F_rf (Hz) / F_step(Hz)
                  = F_rf (Hz) * 2^19 / F_xtal(Hz)
                  = F_rf (Hz) * 2^19 / 32e6
                  = F_rf (Hz) * 256/15625

SX1255 frequency setting :
F_register(24bit) = F_rf (Hz) / F_step(Hz)
                  = F_rf (Hz) * 2^20 / F_xtal(Hz)
                  = F_rf (Hz) * 2^20 / 32e6
                  = F_rf (Hz) * 512/15625
*/
#define 	SX125x_32MHz_FRAC	15625	/* irreductible fraction for PLL register caculation */

#define		SX125x_CLK_OUT			1	
#define		SX125x_TX_DAC_CLK_SEL	1	/* 0:int, 1:ext */
#define		SX125x_RX_LNA_GAIN		1	/* 1 to 6, 1 highest gain */
#define		SX125x_RX_BB_GAIN		12	/* 0 to 15 , 15 highest gain */
#define		SX125x_RX_ADC_BW		7	/* 0 to 7, 2:100<BW<200, 5:200<BW<400,7:400<BW (kHz) */
#define		SX125x_RX_ADC_TRIM		6	/* 0 to 7, 6 for 32MHz ref, 5 for 36MHz ref */
#define		SX125x_RXBB_BW			2

#define		RF_CHAIN				0	/* we'll use radio A only */

#define		PLL_LOCK_MAX_ATTEMPTS	6
#define		MEAS_IF					(-100000) /* IF in Hz for the RSSI measurement */
#define		RSSI_OFFSET				0.0	/* TODO: calibration */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES (GLOBAL) ------------------------------------------- */

#include "rssi_fw.var" /* external definition of the variable */

/* signal handling variables */
struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */
static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

static void sig_handler(int sigio);

int load_firmware_(uint8_t target, uint8_t *firmware, uint16_t size);

void sx125x_write_(uint8_t channel, uint8_t addr, uint8_t data);

uint8_t sx125x_read_(uint8_t channel, uint8_t addr);

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

/* size is the firmware size in bytes (not 14b words) */
int load_firmware_(uint8_t target, uint8_t *firmware, uint16_t size) {
	int reg_rst;
	int reg_sel;
	
	/* check parameters */
	if (target == MCU_ARB) {
		if (size != MCU_ARB_FW_BYTE) {
			MSG("ERROR: NOT A VALID SIZE FOR MCU ARG FIRMWARE\n");
			return -1;
		}
		reg_rst = LGW_MCU_RST_0;
		reg_sel = LGW_MCU_SELECT_MUX_0;
	}else if (target == MCU_AGC) {
		if (size != MCU_AGC_FW_BYTE) {
			MSG("ERROR: NOT A VALID SIZE FOR MCU AGC FIRMWARE\n");
			return -1;
		}
		reg_rst = LGW_MCU_RST_1;
		reg_sel = LGW_MCU_SELECT_MUX_1;
	} else {
		MSG("ERROR: NOT A VALID TARGET FOR LOADING FIRMWARE\n");
		return -1;
	}
	
	/* reset the targeted MCU */
	lgw_reg_w(reg_rst, 1);
	
	/* set mux to access MCU program RAM and set address to 0 */
	lgw_reg_w(reg_sel, 0);
	lgw_reg_w(LGW_MCU_PROM_ADDR, 0);
	
	/* write the program in one burst */
	lgw_reg_wb(LGW_MCU_PROM_DATA, firmware, size);
	
	/* give back control of the MCU program ram to the MCU */
	lgw_reg_w(reg_sel, 1);
	
	return 0;
}

void sx125x_write_(uint8_t channel, uint8_t addr, uint8_t data) {
	int reg_add, reg_dat, reg_cs;
	
	/* checking input parameters */
	if (channel >= LGW_RF_CHAIN_NB) {
		MSG("ERROR: INVALID RF_CHAIN\n");
		return;
	}
	if (addr >= 0x7F) {
		MSG("ERROR: ADDRESS OUT OF RANGE\n");
		return;
	}
	
	/* selecting the target radio */
	switch (channel) {
		case 0:
			reg_add = LGW_SPI_RADIO_A__ADDR;
			reg_dat = LGW_SPI_RADIO_A__DATA;
			reg_cs	= LGW_SPI_RADIO_A__CS;
			break;
			
		case 1:
			reg_add = LGW_SPI_RADIO_B__ADDR;
			reg_dat = LGW_SPI_RADIO_B__DATA;
			reg_cs	= LGW_SPI_RADIO_B__CS;
			break;
			
		default:
			MSG("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT\n", channel);
			return;
	}
	
	/* SPI master data write procedure */
	lgw_reg_w(reg_cs, 0);
	lgw_reg_w(reg_add, 0x80 | addr); /* MSB at 1 for write operation */
	lgw_reg_w(reg_dat, data);
	lgw_reg_w(reg_cs, 1);
	lgw_reg_w(reg_cs, 0);
	
	return;
}

uint8_t sx125x_read_(uint8_t channel, uint8_t addr) {
	int reg_add, reg_dat, reg_cs, reg_rb;
	int32_t read_value;
	
	/* checking input parameters */
	if (channel >= LGW_RF_CHAIN_NB) {
		MSG("ERROR: INVALID RF_CHAIN\n");
		return 0;
	}
	if (addr >= 0x7F) {
		MSG("ERROR: ADDRESS OUT OF RANGE\n");
		return 0;
	}
	
	/* selecting the target radio */
	switch (channel) {
		case 0:
			reg_add = LGW_SPI_RADIO_A__ADDR;
			reg_dat = LGW_SPI_RADIO_A__DATA;
			reg_cs	= LGW_SPI_RADIO_A__CS;
			reg_rb	= LGW_SPI_RADIO_A__DATA_READBACK;
			break;
			
		case 1:
			reg_add = LGW_SPI_RADIO_B__ADDR;
			reg_dat = LGW_SPI_RADIO_B__DATA;
			reg_cs	= LGW_SPI_RADIO_B__CS;
			reg_rb	= LGW_SPI_RADIO_B__DATA_READBACK;
			break;
			
		default:
			MSG("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT\n", channel);
			return 0;
	}
	
	/* SPI master data read procedure */
	lgw_reg_w(reg_cs, 0);
	lgw_reg_w(reg_add, addr); /* MSB at 0 for read operation */
	lgw_reg_w(reg_dat, 0);
	lgw_reg_w(reg_cs, 1);
	lgw_reg_w(reg_cs, 0);
	lgw_reg_r(reg_rb, &read_value);
	
	return (uint8_t)read_value;
}

/* describe command line options */
void usage(void) {
	printf("*** Library version information ***\n%s\n\n", lgw_version_info());
	printf( "Available options:\n");
	printf( "-h print this help\n");
	printf( "-f <Fstart>:<Fstop> or <Fstart>:<Fstop>:<Fstep> in MHz (scient. nota. OK)\n");
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv)
{
	int i; /* temporary variable(s) */
	int32_t reg_val;
	
	/* user chosen parameters */
	double f1 = 0.0;
	double f2 = 0.0;
	double fs = 0.0;
	
	/* frequency scan parameters (default values) */
	uint32_t f_start = rf_rx_lowfreq_[RF_CHAIN]; /* in Hz */
	uint32_t f_stop = rf_rx_upfreq_[RF_CHAIN]; /* in Hz */
	uint32_t f_step = 200000; /* 200 kHz step by default */
	
	/* RSSI measurement results */
	int8_t rssi_max; /* max RSSI during X measurements */
	int high_count; /* nb of measurements above a threshold defined by maximum RSSI */
	
	/* clock, log file and log rotation management */
	FILE * log_file = NULL;
	char log_file_name[64];
	char iso_date[20];
	time_t now_time;
	
	/* variables for PLL register calculation */
	uint32_t f_target; /* loop variable */
	uint32_t freq_hz;
	uint32_t part_int;
	uint32_t part_frac;
	int cpt_attempts = 0;
	
	/* parse command line options */
	while ((i = getopt (argc, argv, "hf:")) != -1) {
		switch (i) {
			case 'h':
				usage();
				return EXIT_SUCCESS;
			
			case 'f':
				sscanf(optarg, "%lf:%lf:%lf", &f1, &f2, &fs);
				/* check configuration sanity */
				if (f2 < f1) {
					MSG("ERROR: stop frequency must be bigger than start frequency\n");
					return EXIT_FAILURE;
				}
				if ((f1 < 30.0) || (f1 > 3000.0)) {
					MSG("ERROR: invalid start frequency %f MHz\n", f1);
					return EXIT_FAILURE;
				}
				if ((f2 < 30.0) || (f2 > 3000.0)) {
					MSG("ERROR: invalid stop frequency %f MHz\n", f2);
					return EXIT_FAILURE;
				}
				f_start = (uint32_t)((f1*1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
				f_stop = (uint32_t)((f2*1e6) + 0.5);
				if (fs > 0.01) {
					f_step = (uint32_t)((fs*1e6) + 0.5);
				}
				break;
			
			default:
				MSG("ERROR: argument parsing use -h option for help\n");
				usage();
				return EXIT_FAILURE;
		}
	}
	printf("Scanning from %u Hz to %u Hz with a %u Hz frequency step\n", f_start, f_stop, f_step);
	
	/* configure signal handling */
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigact.sa_handler = sig_handler;
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	
	/* establish connection with concentrator and reset it */
	if (lgw_connect() == LGW_REG_ERROR) {
		MSG("ERROR: fail to connect to concentrator board\n");
		return EXIT_FAILURE;
	}
	lgw_soft_reset();
	
	/* Ungate concentrator clock, switch on the radios and reset them */
	lgw_reg_w(LGW_GLOBAL_EN, 1);
	lgw_reg_w(LGW_RADIO_A_EN,1);
	lgw_reg_w(LGW_RADIO_B_EN,1);
	wait_ms(500);
	lgw_reg_w(LGW_RADIO_RST,1);
	wait_ms(5);
	lgw_reg_w(LGW_RADIO_RST,0);
	wait_ms(5);
	
	/* enter basic parameters for the radio */
	sx125x_write_(RF_CHAIN, 0x10, SX125x_TX_DAC_CLK_SEL + SX125x_CLK_OUT*2);
	sx125x_write_(RF_CHAIN, 0x0C, 0 + SX125x_RX_BB_GAIN*2 + SX125x_RX_LNA_GAIN*32); /* not required, firmware should take care of that */
	sx125x_write_(RF_CHAIN, 0x0D, SX125x_RXBB_BW + SX125x_RX_ADC_TRIM*4 + SX125x_RX_ADC_BW*32);
	
	/* configure the IF and concentrator parameters */
	lgw_reg_w(LGW_IF_FREQ_0, -282); /* default -384 */
	lgw_reg_w(LGW_IF_FREQ_1, -128); /* default -128 */
	
	lgw_reg_w(LGW_RSSI_BB_FILTER_ALPHA,9); /* default 7 */
	lgw_reg_w(LGW_RSSI_DEC_FILTER_ALPHA,7); /* default 5 */
	lgw_reg_w(LGW_RSSI_CHANN_FILTER_ALPHA,3); /* default 8 */
	lgw_reg_w(LGW_RSSI_CHANN_DEFAULT_VALUE,90); /* default 100 */
	lgw_reg_w(LGW_RSSI_DEC_DEFAULT_VALUE,90); /* default 100 */
	
	/* Load firmware */
	load_firmware_(MCU_AGC, rssi_firmware, MCU_AGC_FW_BYTE);
	lgw_reg_w(LGW_FORCE_HOST_FE_CTRL,0);
	lgw_reg_w(LGW_FORCE_DEC_FILTER_GAIN,0);
	
	/* open log file */
	time(&now_time);
	strftime(iso_date,ARRAY_SIZE(iso_date),"%Y%m%dT%H%M%SZ",gmtime(&now_time)); /* format yyyymmddThhmmssZ */
	sprintf(log_file_name, "band_survey_%s.csv", iso_date);
	log_file = fopen(log_file_name, "a"); /* create log file, append if file already exist */
	if (log_file == NULL) {
		MSG("ERROR: impossible to create log file %s\n", log_file_name);
		return EXIT_FAILURE;
	}
	i = fprintf(log_file, "\"Frequency (Hz)\",\"RSSI (dB)\",\"high meas (nb)\"\n");
	if (i < 0) {
		MSG("ERROR: impossible to write to log file %s\n", log_file_name);
		return EXIT_FAILURE;
	}
	
	/* main loop */
	f_target = f_start;
	
	while ((quit_sig != 1) && (exit_sig != 1) && (f_target <= f_stop)) {
		
		/* set PLL to target frequency */
		freq_hz = f_target - MEAS_IF;
		
		#if (CFG_RADIO_1257 == 1)
		part_int = freq_hz / (SX125x_32MHz_FRAC << 8); /* integer part, gives the MSB */
		part_frac = ((freq_hz % (SX125x_32MHz_FRAC << 8)) << 8) / SX125x_32MHz_FRAC; /* fractional part, gives middle part and LSB */
		#elif (CFG_RADIO_1255 == 1)
		part_int = freq_hz / (SX125x_32MHz_FRAC << 7); /* integer part, gives the MSB */
		part_frac = ((freq_hz % (SX125x_32MHz_FRAC << 7)) << 9) / SX125x_32MHz_FRAC; /* fractional part, gives middle part and LSB */
		#endif
		
		sx125x_write_(RF_CHAIN, 0x01,0xFF & part_int); /* Most Significant Byte */
		sx125x_write_(RF_CHAIN, 0x02,0xFF & (part_frac >> 8)); /* middle byte */
		sx125x_write_(RF_CHAIN, 0x03,0xFF & part_frac); /* Least Significant Byte */
		
		/* start and PLL lock */
		cpt_attempts = 0;
		do {
			if (cpt_attempts >= PLL_LOCK_MAX_ATTEMPTS) {
				MSG("ERROR: fail to lock PLL\n");
				return -1;
			}
			sx125x_write_(RF_CHAIN, 0x00, 1); /* enable Xtal oscillator */
			sx125x_write_(RF_CHAIN, 0x00, 3); /* Enable RX (PLL+FE) */
			++cpt_attempts;
			wait_ms(1);
		} while((sx125x_read_(RF_CHAIN, 0x11) & 0x02) == 0);
		
		/* give control of the radio to the MCU and get it out of reset */
		lgw_reg_w(LGW_FORCE_HOST_RADIO_CTRL,0);
		lgw_reg_w(LGW_MCU_RST_1, 0);
		
		/* wait for the firmware to finish running and fetch the result */
		do {
			wait_ms(1);
			lgw_reg_r(LGW_MCU_AGC_STATUS, &reg_val);
		} while (reg_val != 1);
		
		lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR,0x20);
		lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &reg_val);
		rssi_max = (int8_t)reg_val;
		
		lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR,0x21);
		lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &reg_val);
		high_count = reg_val;
		
		lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR,0x22);
		lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &reg_val);
		high_count += (reg_val << 8);
		
		/* log the measurement */
		i = fprintf(log_file, "%u, %i, %u\n", f_target, rssi_max, high_count);
		if (i < 0) {
			MSG("ERROR: impossible to write to log file %s\n", log_file_name);
			return EXIT_FAILURE;
		}
		
		/* reset MCU and take back control of radio */
		lgw_reg_w(LGW_MCU_RST_1, 1);
		lgw_reg_w(LGW_FORCE_HOST_RADIO_CTRL,1);
		
		/* iterate loop */
		f_target += f_step;
	}
	
	fclose(log_file);
	lgw_soft_reset();
	lgw_disconnect();
	
	printf("Exiting band survey program\n");
	return EXIT_SUCCESS;
}

/* --- EOF ------------------------------------------------------------------ */
