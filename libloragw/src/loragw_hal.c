/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
	LoRa concentrator Hardware Abstraction Layer

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Sylvain Miermont
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>		/* C99 types */
#include <stdbool.h>	/* bool type */
#include <stdio.h>		/* printf fprintf */
#include <string.h>		/* memcpy */

#include "loragw_reg.h"
#include "loragw_hal.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_HAL == 1
	#define DEBUG_MSG(str)				fprintf(stderr, str)
	#define DEBUG_PRINTF(fmt, args...)	fprintf(stderr,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
	#define DEBUG_ARRAY(a,b,c)			for(a=0;a<b;++a) fprintf(stderr,"%x.",c[a]);fprintf(stderr,"end\n")
	#define CHECK_NULL(a)				if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_HAL_ERROR;}
#else
	#define DEBUG_MSG(str)
	#define DEBUG_PRINTF(fmt, args...)
	#define DEBUG_ARRAY(a,b,c)			for(a=0;a!=0;){}
	#define CHECK_NULL(a)				if(a==NULL){return LGW_HAL_ERROR;}
#endif

#define IF_HZ_TO_REG(f)		(f << 5)/15625
#define	SET_PPM_ON(bw,dr)	(((bw == BW_125KHZ) && ((dr == DR_LORA_SF11) || (dr == DR_LORA_SF12))) || ((bw == BW_250KHZ) && (dr == DR_LORA_SF12)))
#define TRACE()				fprintf(stderr, "@ %s %d\n", __FUNCTION__, __LINE__);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS & TYPES -------------------------------------------- */

#define		MCU_ARB		0
#define		MCU_AGC		1

#define		MCU_ARB_FW_BYTE		8192 /* size of the firmware IN BYTES (= twice the number of 14b words) */
#define		MCU_AGC_FW_BYTE		8192 /* size of the firmware IN BYTES (= twice the number of 14b words) */

#define		TX_METADATA_NB		16
#define		RX_METADATA_NB		16

#define		AGC_CMD_WAIT		16
#define		AGC_CMD_ABORT		17

#define		MIN_LORA_PREAMBLE		4
#define		STD_LORA_PREAMBLE		6
#define		MIN_FSK_PREAMBLE		3
#define		STD_FSK_PREAMBLE		5
#define		PLL_LOCK_MAX_ATTEMPTS	5

#define		TX_START_DELAY		1500

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

#define		SX125x_TX_DAC_CLK_SEL	1	/* 0:int, 1:ext */
#define		SX125x_TX_DAC_GAIN		2	/* 3:0, 2:-3, 1:-6, 0:-9 dBFS (default 2) */
#define		SX125x_TX_MIX_GAIN		14	/* -38 + 2*TxMixGain dB (default 14) */
#define		SX125x_TX_PLL_BW		3	/* 0:75, 1:150, 2:225, 3:300 kHz (default 3) */
#define		SX125x_TX_ANA_BW		0	/* 17.5 / 2*(41-TxAnaBw) MHz (default 0) */
#define		SX125x_TX_DAC_BW		5	/* 24 + 8*TxDacBw Nb FIR taps (default 2) */
#define		SX125x_RX_LNA_GAIN		1	/* 1 to 6, 1 highest gain */
#define		SX125x_RX_BB_GAIN		12	/* 0 to 15 , 15 highest gain */
#define 	SX125x_LNA_ZIN			1	/* 0:50, 1:200 Ohms (default 1) */
#define		SX125x_RX_ADC_BW		7	/* 0 to 7, 2:100<BW<200, 5:200<BW<400,7:400<BW kHz SSB (default 7) */
#define		SX125x_RX_ADC_TRIM		6	/* 0 to 7, 6 for 32MHz ref, 5 for 36MHz ref */
#define 	SX125x_RX_BB_BW			0	/* 0:750, 1:500, 2:375; 3:250 kHz SSB (default 1, max 3) */
#define 	SX125x_RX_PLL_BW		0	/* 0:75, 1:150, 2:225, 3:300 kHz (default 3, max 3) */
#define 	SX125x_ADC_TEMP			0	/* ADC temperature measurement mode (default 0) */
#define 	SX125x_XOSC_GM_STARTUP	13	/* (default 13) */
#define 	SX125x_XOSC_DISABLE		2	/* Disable of Xtal Oscillator blocks bit0:regulator, bit1:core(gm), bit2:amplifier */

#define		RSSI_MULTI_BIAS			-35		/* difference between "multi" modem RSSI offset and "stand-alone" modem RSSI offset */
#define		RSSI_FSK_BIAS			-37.0	/* difference between FSK modem RSSI offset and "stand-alone" modem RSSI offset */
#define		RSSI_FSK_REF			-70.0	/* linearize FSK RSSI curve around -70 dBm */
#define		RSSI_FSK_SLOPE			0.8

/* Board-specific RSSI calibration constants */
#if (CFG_BRD_NANO868 == 1)
	#define		RSSI_BOARD_OFFSET		176.0
#elif (CFG_BRD_1301REF868 == 1)
	#define		RSSI_BOARD_OFFSET		166.0
#elif (CFG_BRD_KERLINK868 == 1)
	#define		RSSI_BOARD_OFFSET		165.0
#elif (CFG_BRD_1301REF433 == 1)
	#define		RSSI_BOARD_OFFSET		176.5
#elif (CFG_BRD_KERLINK433 == 1)
	#define		RSSI_BOARD_OFFSET		178.0
#elif (CFG_BRD_CISCO433 == 1)
	#define		RSSI_BOARD_OFFSET		175.5
#elif (CFG_BRD_CISCO470 == 1)
	#define		RSSI_BOARD_OFFSET		173.5
#elif (CFG_BRD_CISCO780 == 1)
	#define		RSSI_BOARD_OFFSET		168.0
/* === ADD CUSTOMIZATION FOR YOUR OWN BOARD HERE ===
#elif (CFG_BRD_MYBOARD == 1)
*/
#elif (CFG_BRD_NONE == 1)
	#define		RSSI_BOARD_OFFSET		0
#endif

/* constant arrays defining hardware capability */

const uint8_t ifmod_config[LGW_IF_CHAIN_NB] = LGW_IFMODEM_CONFIG;

const uint32_t rf_rx_lowfreq[LGW_RF_CHAIN_NB] = LGW_RF_RX_LOWFREQ;
const uint32_t rf_rx_upfreq[LGW_RF_CHAIN_NB] = LGW_RF_RX_UPFREQ;
const uint32_t rf_rx_bandwidth[LGW_RF_CHAIN_NB] = LGW_RF_RX_BANDWIDTH;
const uint32_t rf_tx_lowfreq[LGW_RF_CHAIN_NB] = LGW_RF_TX_LOWFREQ;
const uint32_t rf_tx_upfreq[LGW_RF_CHAIN_NB] = LGW_RF_TX_UPFREQ;
const bool rf_tx_enable[LGW_RF_CHAIN_NB] = LGW_RF_TX_ENABLE;
const bool rf_clkout[LGW_RF_CHAIN_NB] = LGW_RF_CLKOUT;

/* TX power management */

#define	TX_POW_LUT_SIZE	16

typedef struct {
	uint8_t	pa_gain;	/* 2 bits, control of the external PA (SX1301 I/O) */
	uint8_t	dac_gain;	/* 2 bits, control of the radio DAC */
	uint8_t	mix_gain;	/* 4 bits, control of the radio mixer */
	int8_t	rf_power;	/* measured TX power at the board connector, in dBm */
} tx_pow_t;

/* Default table (TX power associated to each value is board-dependant)
	+-------+------+------+------+------+
	|       |  PA  | DAC  | mix. | ctrl |
	| index | ctrl | ctrl | ctrl | byte |
	+-------+--------------------+------+
	|    0  |   0      3      8  | 0x38 |
	|    1  |   0      3     10  | 0x3A |
	|    2  |   0      3     12  | 0x3C |
	|    3  |   1      3      8  | 0x78 |
	|    4  |   1      3     10  | 0x7A |
	|    5  |   1      3     12  | 0x7C |
	|    6  |   1      3     13  | 0x7D |
	|    7  |   1      3     15  | 0x7F |
	|    8  |   2      3      9  | 0xB9 |
	|    9  |   2      3     10  | 0xBA |
	|   10  |   2      3     11  | 0xBB |
	|   11  |   3      3     10  | 0xFA |
	|   12  |   3      3     11  | 0xFB |
	|   13  |   3      3     12  | 0xFC |
	|   14  |   3      3     13  | 0xFD |
	|   15  |   3      3     15  | 0xFF |
	+-------+--------------------+------+
*/

#if (CFG_BRD_NANO868 == 1)
	#define	CUSTOM_TX_POW_TABLE		1 /* is custom table loading sequence needed ? */
	const tx_pow_t tx_pow_table[TX_POW_LUT_SIZE] = {\
		{	0,	3,	 8,	 2},\
		{	0,	3,	 9,	 3},\
		{	0,	3,	10,	 5},\
		{	0,	3,	12,	 7},\
		{	0,	3,	14,	 9},\
		{	0,	3,	15,	10},\
		{	1,	3,	 8,	12},\
		{	1,	3,	 9,	14},\
		{	1,	3,	10,	15},\
		{	1,	3,	11,	17},\
		{	1,	3,	12,	18},\
		{	1,	3,	13,	20},\
		{	2,	3,	 8,	21},\
		{	2,	3,	 9,	23},\
		{	2,	3,	11,	25},\
		{	2,	3,	13,	27},\
	}; /* calibrated */
#elif (CFG_BRD_1301REF868 == 1)
	#define	CUSTOM_TX_POW_TABLE		1
	const tx_pow_t tx_pow_table[TX_POW_LUT_SIZE] = {\
		{	0,	3,	 8,	-6},\
		{	0,	3,	10,	-3},\
		{	0,	3,	12,	 0},\
		{	1,	3,	 8,	 3},\
		{	1,	3,	10,	 6},\
		{	1,	3,	12,	10},\
		{	1,	3,	13,	11},\
		{	2,	3,	 9,	12},\
		{	1,	3,	15,	13},\
		{	2,	3,	10,	14},\
		{	2,	3,	11,	16},\
		{	3,	3,	 9,	20},\
		{	3,	3,	10,	23},\
		{	3,	3,	11,	25},\
		{	3,	3,	12,	26},\
		{	3,	3,	14,	27},\
	}; /* calibrated */
#elif (CFG_BRD_1301REF433 == 1)
	#define	CUSTOM_TX_POW_TABLE		1
	// WARNING: TABLE NO CALIBRATED, COPIED FOR 868
	const tx_pow_t tx_pow_table[TX_POW_LUT_SIZE] = {\
		{	0,	3,	 8,	-9},\
		{	0,	3,	10,	-6},\
		{	0,	3,	12,	-3},\
		{	1,	3,	 8,	 0},\
		{	1,	3,	10,	 4},\
		{	1,	3,	12,	 7},\
		{	1,	3,	13,	 8},\
		{	1,	3,	15,	 9},\
		{	2,	3,	 9,	10},\
		{	2,	3,	10,	12},\
		{	2,	3,	11,	13},\
		{	3,	3,	10,	21},\
		{	3,	3,	12,	23},\
		{	3,	3,	12,	24},\
		{	3,	3,	13,	25},\
		{	3,	3,	15,	26},\
	}; /* TODO: approximative calibration, needs to be adjusted */
#elif (CFG_BRD_KERLINK868 == 1)
	#define	CUSTOM_TX_POW_TABLE		1
	const tx_pow_t tx_pow_table[TX_POW_LUT_SIZE] = {\
		{	0,	3,	 9,-10},\
		{	0,	3,	12,	-6},\
		{	0,	3,	15,	-3},\
		{	1,	3,	 9,	 0},\
		{	1,	3,	12,	 5},\
		{	1,	3,	14,	 7},\
		{	1,	3,	15,	 8},\
		{	2,	3,	10,	10},\
		{	2,	3,	11,	12},\
		{	2,	3,	13,	15},\
		{	3,	3,	 9,	17},\
		{	3,	3,	10,	19},\
		{	3,	3,	11,	21},\
		{	3,	3,	12,	22},\
		{	3,	3,	13,	23},\
		{	3,	3,	15,	24},\
	}; /* calibrated */
#elif (CFG_BRD_KERLINK433 == 1)
	#define	CUSTOM_TX_POW_TABLE		1
	const tx_pow_t tx_pow_table[TX_POW_LUT_SIZE] = {\
		{	0,	3,	 8, -6},\
		{	0,	3,	11,	 0},\
		{	0,	3,	14,	 3},\
		{	1,	3,	 9,	 6},\
		{	1,	3,	10,	 8},\
		{	1,	3,	11,	10},\
		{	1,	3,	12,	11},\
		{	1,	3,	13,	12},\
		{	1,	3,	14,	13},\
		{	1,	3,	15,	14},\
		{	2,	3,	11,	20},\
		{	2,	3,	12,	21},\
		{	3,	3,	 8,	22},\
		{	3,	3,	 9,	24},\
		{	3,	3,	10,	25},\
		{	3,	3,	12,	26},\
	}; /* calibrated */
#elif (CFG_BRD_CISCO433 == 1)
	#define	CUSTOM_TX_POW_TABLE		1
	const tx_pow_t tx_pow_table[TX_POW_LUT_SIZE] = {\
		{	0,	3,	 8,	-7},\
		{	0,	3,	10,	-3},\
		{	0,	3,	12,	 0},\
		{	1,	3,	 8,	 4},\
		{	1,	3,	10,	 7},\
		{	1,	3,	11,	 8},\
		{	1,	3,	12,	 9},\
		{	2,	3,	 8,	11},\
		{	2,	3,	 9,	14},\
		{	2,	3,	10,	15},\
		{	2,	3,	11,	17},\
		{	2,	3,	12,	18},\
		{	3,	3,	 8,	20},\
		{	3,	3,	 9,	22},\
		{	3,	3,	10,	23},\
		{	3,	3,	11,	24},\
	}; /* calibrated */
#elif (CFG_BRD_CISCO470 == 1)
	#define	CUSTOM_TX_POW_TABLE		1
	const tx_pow_t tx_pow_table[TX_POW_LUT_SIZE] = {\
		{	0,	3,	 9,	 0},\
		{	0,	3,	13,	 4},\
		{	1,	3,	 8,	 8},\
		{	1,	3,	 9,	10},\
		{	1,	3,	10,	11},\
		{	1,	3,	11,	12},\
		{	1,	3,	12,	13},\
		{	1,	3,	13,	14},\
		{	1,	3,	14,	15},\
		{	2,	3,	 8,	16},\
		{	2,	3,	 9,	18},\
		{	2,	3,	10,	20},\
		{	2,	3,	11,	21},\
		{	2,	3,	13,	22},\
		{	3,	3,	 9,	23},\
		{	3,	3,	11,	24},\
	}; /* calibrated */
#elif (CFG_BRD_CISCO780 == 1)
	#define	CUSTOM_TX_POW_TABLE		1
	const tx_pow_t tx_pow_table[TX_POW_LUT_SIZE] = {\
		{	0,	3,	 8,	-12},\
		{	0,	3,	11,	 -6},\
		{	0,	3,	14,	 -3},\
		{	1,	3,	 9,	  0},\
		{	1,	3,	11,	  3},\
		{	1,	3,	13,	  6},\
		{	2,	3,	10,	  8},\
		{	2,	3,	11,	 10},\
		{	2,	3,	12,	 12},\
		{	2,	3,	13,	 14},\
		{	2,	3,	15,	 16},\
		{	3,	3,	10,	 18},\
		{	3,	3,	11,	 20},\
		{	3,	3,	12,	 22},\
		{	3,	3,	14,	 24},\
		{	3,	3,	15,	 25},\
	}; /* calibrated */
/* === ADD CUSTOMIZATION FOR YOUR OWN BOARD HERE ===
#elif (CFG_BRD_MYBOARD == 1)
*/
#elif (CFG_BRD_NONE == 1)
	#define	CUSTOM_TX_POW_TABLE		0
	const tx_pow_t tx_pow_table[TX_POW_LUT_SIZE] = {\
		{	0,	3,	8,	0},\
		{	0,	3,	10,	1},\
		{	0,	3,	12,	2},\
		{	1,	3,	8,	3},\
		{	1,	3,	10,	4},\
		{	1,	3,	12,	5},\
		{	1,	3,	13,	6},\
		{	1,	3,	15,	7},\
		{	2,	3,	9,	8},\
		{	2,	3,	10,	9},\
		{	2,	3,	11,	10},\
		{	3,	3,	10,	11},\
		{	3,	3,	12,	12},\
		{	3,	3,	12,	13},\
		{	3,	3,	13,	14},\
		{	3,	3,	15,	15},\
	}; // uncalibrated table, lgw_pkt_tx_s.rf_power selects table index */
#endif

/* Strings for version (and options) identification */

#if (CFG_SPI_NATIVE == 1)
	#define		CFG_SPI_STR		"native"
#elif (CFG_SPI_FTDI == 1)
	#define		CFG_SPI_STR		"ftdi"
#else
	#define		CFG_SPI_STR		"spi?"
#endif

#if (CFG_CHIP_1301 == 1)
	#define		CFG_CHIP_STR	"sx1301"
#elif (CFG_CHIP_FPGA == 1)
	#define		CFG_CHIP_STR	"fpga1301"
#else
	#define		CFG_CHIP_STR	"chip?"
#endif

#if (CFG_RADIO_1257 == 1)
	#define		CFG_RADIO_STR	"sx1257"
#elif (CFG_RADIO_1255 == 1)
	#define		CFG_RADIO_STR	"sx1255"
#else
	#define		CFG_RADIO_STR	"radio?"
#endif

#if (CFG_BAND_FULL == 1)
	#define		CFG_BAND_STR	"full"
#elif (CFG_BAND_868 == 1)
	#define		CFG_BAND_STR	"eu868"
#elif (CFG_BAND_915 == 1)
	#define		CFG_BAND_STR	"us915"
#elif (CFG_BAND_470 == 1)
	#define		CFG_BAND_STR	"cn470"
#elif (CFG_BAND_433 == 1)
	#define		CFG_BAND_STR	"eu433"
#elif (CFG_BAND_780 == 1)
	#define		CFG_BAND_STR	"cn780"
#else
	#define		CFG_BAND_STR	"band?"
#endif

#if (CFG_BRD_NANO868 == 1)
	#define		CFG_BRD_STR		"dev_nano_868"
#elif (CFG_BRD_1301REF868 == 1)
	#define		CFG_BRD_STR		"ref_1301_868"
#elif (CFG_BRD_1301REF433 == 1)
	#define		CFG_BRD_STR		"ref_1301_433"
#elif (CFG_BRD_KERLINK868 == 1)
	#define		CFG_BRD_STR		"kerlink_868"
#elif (CFG_BRD_KERLINK433 == 1)
	#define		CFG_BRD_STR		"kerlink_433"
#elif (CFG_BRD_CISCO433 == 1)
	#define		CFG_BRD_STR		"cisco_433"
#elif (CFG_BRD_CISCO470 == 1)
	#define		CFG_BRD_STR		"cisco_470"
#elif (CFG_BRD_CISCO780 == 1)
	#define		CFG_BRD_STR		"cisco_780"
/* === ADD CUSTOMIZATION FOR YOUR OWN BOARD HERE ===
#elif (CFG_BRD_MYBOARD == 1)
*/
#elif (CFG_BRD_NONE == 1)
	#define		CFG_BRD_STR		"no_brd"
#else
	#define		CFG_BRD_STR		"brd?"
#endif

#if (CFG_NET_PRIVATE == 1)
	#define		CFG_NET_STR		"private"
#elif (CFG_NET_LORAMAC == 1)
	#define		CFG_NET_STR		"lora_mac"
#else
	#define		CFG_NET_STR		"network?"
#endif

/* Version string, used to identify the library version/options once compiled */
const char lgw_version_string[] = "Version: " LIBLORAGW_VERSION "; Options: " CFG_SPI_STR " " CFG_CHIP_STR " " CFG_RADIO_STR " " CFG_BAND_STR " " CFG_BRD_STR " " CFG_NET_STR ";";

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

#include "arb_fw.var" /* external definition of the variable */
#include "agc_fw.var" /* external definition of the variable */
#include "cal_fw.var" /* external definition of the variable */

/*
The following static variables are the configuration set that the user can
modify using rxrf_setconf and rxif_setconf functions.
The function _start then use that set to configure the hardware.

Parameters validity and coherency is verified by the _setconf functions and
the _start function assumes 
*/

static bool lgw_is_started;

static bool rf_enable[LGW_RF_CHAIN_NB];
static uint32_t rf_rx_freq[LGW_RF_CHAIN_NB]; /* absolute, in Hz */

static bool if_enable[LGW_IF_CHAIN_NB];
static bool if_rf_chain[LGW_IF_CHAIN_NB]; /* for each IF, 0 -> radio A, 1 -> radio B */
static int32_t if_freq[LGW_IF_CHAIN_NB]; /* relative to radio frequency, +/- in Hz */

static uint8_t lora_multi_sfmask[LGW_MULTI_NB]; /* enables SF for LoRa 'multi' modems */

static uint8_t lora_rx_bw; /* bandwidth setting for LoRa standalone modem */
static uint8_t lora_rx_sf; /* spreading factor setting for LoRa standalone modem */
static bool lora_rx_ppm_offset;

static uint8_t fsk_rx_bw; /* bandwidth setting of FSK modem */
static uint32_t fsk_rx_dr; /* FSK modem datarate in bauds */
static uint8_t fsk_sync_word_size = 3; /* default number of bytes for FSK sync word */
static uint64_t fsk_sync_word= 0xC194C1; /* default FSK sync word (ALIGNED RIGHT, MSbit first) */

/* TX I/Q imbalance coefficients for mixer gain = 8 to 15 */
static int8_t cal_offset_a_i[8]; /* TX I offset for radio A */
static int8_t cal_offset_a_q[8]; /* TX Q offset for radio A */
static int8_t cal_offset_b_i[8]; /* TX I offset for radio B */
static int8_t cal_offset_b_q[8]; /* TX Q offset for radio B */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

int load_firmware(uint8_t target, uint8_t *firmware, uint16_t size);

void sx125x_write(uint8_t channel, uint8_t addr, uint8_t data);

uint8_t sx125x_read(uint8_t channel, uint8_t addr);

int setup_sx125x(uint8_t rf_chain, uint32_t freq_hz);

void lgw_constant_adjust(void);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

/* size is the firmware size in bytes (not 14b words) */
int load_firmware(uint8_t target, uint8_t *firmware, uint16_t size) {
	int reg_rst;
	int reg_sel;
	
	/* check parameters */
	CHECK_NULL(firmware);
	if (target == MCU_ARB) {
		if (size != MCU_ARB_FW_BYTE) {
			DEBUG_MSG("ERROR: NOT A VALID SIZE FOR MCU ARG FIRMWARE\n");
			return -1;
		}
		reg_rst = LGW_MCU_RST_0;
		reg_sel = LGW_MCU_SELECT_MUX_0;
	}else if (target == MCU_AGC) {
		if (size != MCU_AGC_FW_BYTE) {
			DEBUG_MSG("ERROR: NOT A VALID SIZE FOR MCU AGC FIRMWARE\n");
			return -1;
		}
		reg_rst = LGW_MCU_RST_1;
		reg_sel = LGW_MCU_SELECT_MUX_1;
	} else {
		DEBUG_MSG("ERROR: NOT A VALID TARGET FOR LOADING FIRMWARE\n");
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

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void sx125x_write(uint8_t channel, uint8_t addr, uint8_t data) {
	int reg_add, reg_dat, reg_cs;
	
	/* checking input parameters */
	if (channel >= LGW_RF_CHAIN_NB) {
		DEBUG_MSG("ERROR: INVALID RF_CHAIN\n");
		return;
	}
	if (addr >= 0x7F) {
		DEBUG_MSG("ERROR: ADDRESS OUT OF RANGE\n");
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
			DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT\n", channel);
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

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint8_t sx125x_read(uint8_t channel, uint8_t addr) {
	int reg_add, reg_dat, reg_cs, reg_rb;
	int32_t read_value;
	
	/* checking input parameters */
	if (channel >= LGW_RF_CHAIN_NB) {
		DEBUG_MSG("ERROR: INVALID RF_CHAIN\n");
		return 0;
	}
	if (addr >= 0x7F) {
		DEBUG_MSG("ERROR: ADDRESS OUT OF RANGE\n");
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
			DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT\n", channel);
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

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int setup_sx125x(uint8_t rf_chain, uint32_t freq_hz) {
	uint32_t part_int;
	uint32_t part_frac;
	int cpt_attempts = 0;
	
	if (rf_chain >= LGW_RF_CHAIN_NB) {
		DEBUG_MSG("ERROR: INVALID RF_CHAIN\n");
		return -1;
	}
	
	/* Get version to identify SX1255/57 silicon revision */
	DEBUG_PRINTF("Note: SX125x #%d version register returned 0x%02x\n", rf_chain, sx125x_read(rf_chain, 0x07));
	
	/* General radio setup */
	if (rf_clkout[rf_chain] == true) {
		sx125x_write(rf_chain, 0x10, SX125x_TX_DAC_CLK_SEL + 2);
		DEBUG_PRINTF("Note: SX125x #%d clock output enabled\n", rf_chain);
	} else {
		sx125x_write(rf_chain, 0x10, SX125x_TX_DAC_CLK_SEL);
		DEBUG_PRINTF("Note: SX125x #%d clock output disabled\n", rf_chain);
	}
	#if (CFG_RADIO_1257 == 1)
	sx125x_write(rf_chain, 0x26, SX125x_XOSC_GM_STARTUP + SX125x_XOSC_DISABLE*16);
	#elif (CFG_RADIO_1255 == 1)
	sx125x_write(rf_chain, 0x28, SX125x_XOSC_GM_STARTUP + SX125x_XOSC_DISABLE*16);
	#endif
	
	if (rf_enable[rf_chain] == true) {
		/* Tx gain and trim */
		sx125x_write(rf_chain, 0x08, SX125x_TX_MIX_GAIN + SX125x_TX_DAC_GAIN*16);
		sx125x_write(rf_chain, 0x0A, SX125x_TX_ANA_BW + SX125x_TX_PLL_BW*32);
		sx125x_write(rf_chain, 0x0B, SX125x_TX_DAC_BW);
		
		/* Rx gain and trim */
		sx125x_write(rf_chain, 0x0C, SX125x_LNA_ZIN + SX125x_RX_BB_GAIN*2 + SX125x_RX_LNA_GAIN*32);
		sx125x_write(rf_chain, 0x0D, SX125x_RX_BB_BW + SX125x_RX_ADC_TRIM*4 + SX125x_RX_ADC_BW*32);
		sx125x_write(rf_chain, 0x0E, SX125x_ADC_TEMP + SX125x_RX_PLL_BW*2);
		
		/* set RX PLL frequency */
		#if (CFG_RADIO_1257 == 1)
		part_int = freq_hz / (SX125x_32MHz_FRAC << 8); /* integer part, gives the MSB */
		part_frac = ((freq_hz % (SX125x_32MHz_FRAC << 8)) << 8) / SX125x_32MHz_FRAC; /* fractional part, gives middle part and LSB */
		#elif (CFG_RADIO_1255 == 1)
		part_int = freq_hz / (SX125x_32MHz_FRAC << 7); /* integer part, gives the MSB */
		part_frac = ((freq_hz % (SX125x_32MHz_FRAC << 7)) << 9) / SX125x_32MHz_FRAC; /* fractional part, gives middle part and LSB */
		#endif
		sx125x_write(rf_chain, 0x01,0xFF & part_int); /* Most Significant Byte */
		sx125x_write(rf_chain, 0x02,0xFF & (part_frac >> 8)); /* middle byte */
		sx125x_write(rf_chain, 0x03,0xFF & part_frac); /* Least Significant Byte */
		
		/* start and PLL lock */
		do {
			if (cpt_attempts >= PLL_LOCK_MAX_ATTEMPTS) {
				DEBUG_MSG("ERROR: FAIL TO LOCK PLL\n");
				return -1;
			}
			sx125x_write(rf_chain, 0x00, 1); /* enable Xtal oscillator */
			sx125x_write(rf_chain, 0x00, 3); /* Enable RX (PLL+FE) */
			++cpt_attempts;
			DEBUG_PRINTF("Note: SX125x #%d PLL start (attempt %d)\n", rf_chain, cpt_attempts);
			wait_ms(1);
		} while((sx125x_read(rf_chain, 0x11) & 0x02) == 0);
	} else {
		DEBUG_PRINTF("Note: SX125x #%d kept in standby mode\n", rf_chain);
	}
	
	return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void lgw_constant_adjust(void) {
	
	/* I/Q path setup */
	// lgw_reg_w(LGW_RX_INVERT_IQ,0); /* default 0 */
	// lgw_reg_w(LGW_MODEM_INVERT_IQ,1); /* default 1 */
	// lgw_reg_w(LGW_CHIRP_INVERT_RX,1); /* default 1 */
	// lgw_reg_w(LGW_RX_EDGE_SELECT,0); /* default 0 */
	// lgw_reg_w(LGW_MBWSSF_MODEM_INVERT_IQ,0); /* default 0 */
	// lgw_reg_w(LGW_DC_NOTCH_EN,1); /* default 1 */
	lgw_reg_w(LGW_RSSI_BB_FILTER_ALPHA,6); /* default 7 */
	lgw_reg_w(LGW_RSSI_DEC_FILTER_ALPHA,7); /* default 5 */
	lgw_reg_w(LGW_RSSI_CHANN_FILTER_ALPHA,7); /* default 8 */
	lgw_reg_w(LGW_RSSI_BB_DEFAULT_VALUE,23); /* default 32 */
	lgw_reg_w(LGW_RSSI_CHANN_DEFAULT_VALUE,85); /* default 100 */
	lgw_reg_w(LGW_RSSI_DEC_DEFAULT_VALUE,66); /* default 100 */
	lgw_reg_w(LGW_DEC_GAIN_OFFSET,7); /* default 8 */
	lgw_reg_w(LGW_CHAN_GAIN_OFFSET,6); /* default 7 */
	
	/* Correlator setup */
	// lgw_reg_w(LGW_CORR_DETECT_EN,126); /* default 126 */
	// lgw_reg_w(LGW_CORR_NUM_SAME_PEAK,4); /* default 4 */
	// lgw_reg_w(LGW_CORR_MAC_GAIN,5); /* default 5 */
	// lgw_reg_w(LGW_CORR_SAME_PEAKS_OPTION_SF6,0); /* default 0 */
	// lgw_reg_w(LGW_CORR_SAME_PEAKS_OPTION_SF7,1); /* default 1 */
	// lgw_reg_w(LGW_CORR_SAME_PEAKS_OPTION_SF8,1); /* default 1 */
	// lgw_reg_w(LGW_CORR_SAME_PEAKS_OPTION_SF9,1); /* default 1 */
	// lgw_reg_w(LGW_CORR_SAME_PEAKS_OPTION_SF10,1); /* default 1 */
	// lgw_reg_w(LGW_CORR_SAME_PEAKS_OPTION_SF11,1); /* default 1 */
	// lgw_reg_w(LGW_CORR_SAME_PEAKS_OPTION_SF12,1); /* default 1 */
	// lgw_reg_w(LGW_CORR_SIG_NOISE_RATIO_SF6,4); /* default 4 */
	// lgw_reg_w(LGW_CORR_SIG_NOISE_RATIO_SF7,4); /* default 4 */
	// lgw_reg_w(LGW_CORR_SIG_NOISE_RATIO_SF8,4); /* default 4 */
	// lgw_reg_w(LGW_CORR_SIG_NOISE_RATIO_SF9,4); /* default 4 */
	// lgw_reg_w(LGW_CORR_SIG_NOISE_RATIO_SF10,4); /* default 4 */
	// lgw_reg_w(LGW_CORR_SIG_NOISE_RATIO_SF11,4); /* default 4 */
	// lgw_reg_w(LGW_CORR_SIG_NOISE_RATIO_SF12,4); /* default 4 */
	
	/* LoRa 'multi' demodulators setup */
	// lgw_reg_w(LGW_PREAMBLE_SYMB1_NB,10); /* default 10 */
	// lgw_reg_w(LGW_FREQ_TO_TIME_INVERT,29); /* default 29 */
	// lgw_reg_w(LGW_FRAME_SYNCH_GAIN,1); /* default 1 */
	// lgw_reg_w(LGW_SYNCH_DETECT_TH,1); /* default 1 */
	// lgw_reg_w(LGW_ZERO_PAD,0); /* default 0 */
	lgw_reg_w(LGW_SNR_AVG_CST,3); /* default 2 */
	#if (CFG_NET_LORAMAC == 1)
	lgw_reg_w(LGW_FRAME_SYNCH_PEAK1_POS,3); /* default 1 */
	lgw_reg_w(LGW_FRAME_SYNCH_PEAK2_POS,4); /* default 2 */
	#elif (CFG_NET_PRIVATE == 1)
	//lgw_reg_w(LGW_FRAME_SYNCH_PEAK1_POS,1); /* default 1 */
	//lgw_reg_w(LGW_FRAME_SYNCH_PEAK2_POS,2); /* default 2 */
	#endif
	// lgw_reg_w(LGW_PREAMBLE_FINE_TIMING_GAIN,1); /* default 1 */
	// lgw_reg_w(LGW_ONLY_CRC_EN,1); /* default 1 */
	// lgw_reg_w(LGW_PAYLOAD_FINE_TIMING_GAIN,2); /* default 2 */
	// lgw_reg_w(LGW_TRACKING_INTEGRAL,0); /* default 0 */
	// lgw_reg_w(LGW_ADJUST_MODEM_START_OFFSET_RDX8,0); /* default 0 */
	// lgw_reg_w(LGW_ADJUST_MODEM_START_OFFSET_SF12_RDX4,4092); /* default 4092 */
	// lgw_reg_w(LGW_MAX_PAYLOAD_LEN,255); /* default 255 */
	
	/* LoRa standalone 'MBWSSF' demodulator setup */
	// lgw_reg_w(LGW_MBWSSF_PREAMBLE_SYMB1_NB,10); /* default 10 */
	// lgw_reg_w(LGW_MBWSSF_FREQ_TO_TIME_INVERT,29); /* default 29 */
	// lgw_reg_w(LGW_MBWSSF_FRAME_SYNCH_GAIN,1); /* default 1 */
	// lgw_reg_w(LGW_MBWSSF_SYNCH_DETECT_TH,1); /* default 1 */
	// lgw_reg_w(LGW_MBWSSF_ZERO_PAD,0); /* default 0 */
	#if (CFG_NET_LORAMAC == 1)
	lgw_reg_w(LGW_MBWSSF_FRAME_SYNCH_PEAK1_POS,3); /* default 1 */
	lgw_reg_w(LGW_MBWSSF_FRAME_SYNCH_PEAK2_POS,4); /* default 2 */
	#elif (CFG_NET_PRIVATE == 1)
	//lgw_reg_w(LGW_MBWSSF_FRAME_SYNCH_PEAK1_POS,1); /* default 1 */
	//lgw_reg_w(LGW_MBWSSF_FRAME_SYNCH_PEAK2_POS,2); /* default 2 */
	#endif
	// lgw_reg_w(LGW_MBWSSF_ONLY_CRC_EN,1); /* default 1 */
	// lgw_reg_w(LGW_MBWSSF_PAYLOAD_FINE_TIMING_GAIN,2); /* default 2 */
	// lgw_reg_w(LGW_MBWSSF_PREAMBLE_FINE_TIMING_GAIN,1); /* default 1 */
	// lgw_reg_w(LGW_MBWSSF_TRACKING_INTEGRAL,0); /* default 0 */
	// lgw_reg_w(LGW_MBWSSF_AGC_FREEZE_ON_DETECT,1); /* default 1 */
	
	/* FSK datapath setup */
	lgw_reg_w(LGW_FSK_RX_INVERT,1); /* default 0 */
	lgw_reg_w(LGW_FSK_MODEM_INVERT_IQ,1); /* default 0 */
	
	/* FSK demodulator setup */
	lgw_reg_w(LGW_FSK_RSSI_LENGTH,4); /* default 0 */
	lgw_reg_w(LGW_FSK_PKT_MODE,1); /* variable length, default 0 */
	lgw_reg_w(LGW_FSK_CRC_EN,1); /* default 0 */
	lgw_reg_w(LGW_FSK_DCFREE_ENC,2); /* default 0 */
	// lgw_reg_w(LGW_FSK_CRC_IBM,0); /* default 0 */
	lgw_reg_w(LGW_FSK_ERROR_OSR_TOL,10); /* default 0 */
	lgw_reg_w(LGW_FSK_PKT_LENGTH,255); /* max packet length in variable length mode */
	// lgw_reg_w(LGW_FSK_NODE_ADRS,0); /* default 0 */
	// lgw_reg_w(LGW_FSK_BROADCAST,0); /* default 0 */
	// lgw_reg_w(LGW_FSK_AUTO_AFC_ON,0); /* default 0 */
	lgw_reg_w(LGW_FSK_PATTERN_TIMEOUT_CFG,128); /* sync timeout (allow 8 bytes preamble + 8 bytes sync word, default 0 */
	
	/* TX general parameters */
	lgw_reg_w(LGW_TX_START_DELAY, TX_START_DELAY); /* default 0 */
	
	/* TX LoRa */
	// lgw_reg_w(LGW_TX_MODE,0); /* default 0 */
	lgw_reg_w(LGW_TX_SWAP_IQ,1); /* "normal" polarity; default 0 */
	#if (CFG_NET_LORAMAC == 1)
	lgw_reg_w(LGW_TX_FRAME_SYNCH_PEAK1_POS,3); /* default 1 */
	lgw_reg_w(LGW_TX_FRAME_SYNCH_PEAK2_POS,4); /* default 2 */
	#elif (CFG_NET_PRIVATE == 1)
	//lgw_reg_w(LGW_TX_FRAME_SYNCH_PEAK1_POS,1); /* default 1 */
	//lgw_reg_w(LGW_TX_FRAME_SYNCH_PEAK2_POS,2); /* default 2 */
	#endif	
	
	/* TX FSK */
	// lgw_reg_w(LGW_FSK_TX_GAUSSIAN_EN,1); /* default 1 */
	lgw_reg_w(LGW_FSK_TX_GAUSSIAN_SELECT_BT,2); /* Gaussian filter always on TX, default 0 */
	// lgw_reg_w(LGW_FSK_TX_PATTERN_EN,1); /* default 1 */
	// lgw_reg_w(LGW_FSK_TX_PREAMBLE_SEQ,0); /* default 0 */
	
	return;
}

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int lgw_rxrf_setconf(uint8_t rf_chain, struct lgw_conf_rxrf_s conf) {
	
	/* check if the concentrator is running */
	if (lgw_is_started == true) {
		DEBUG_MSG("ERROR: CONCENTRATOR IS RUNNING, STOP IT BEFORE TOUCHING CONFIGURATION\n");
		return LGW_HAL_ERROR;
	}
	
	/* check input range (segfault prevention) */
	if (rf_chain >= LGW_RF_CHAIN_NB) {
		DEBUG_MSG("ERROR: NOT A VALID RF_CHAIN NUMBER\n");
		return LGW_HAL_ERROR;
	}
	
	/* check input parameters */
	if (conf.freq_hz > rf_rx_upfreq[rf_chain]) {
		DEBUG_MSG("ERROR: FREQUENCY TOO HIGH FOR THAT RF_CHAIN\n");
		return LGW_HAL_ERROR;
	} else if (conf.freq_hz < rf_rx_lowfreq[rf_chain]) {
		DEBUG_MSG("ERROR: FREQUENCY TOO LOW FOR THAT RF_CHAIN\n");
		return LGW_HAL_ERROR;
	}
	
	/* set internal config according to parameters */
	rf_enable[rf_chain] = conf.enable;
	rf_rx_freq[rf_chain] = conf.freq_hz;
	
	DEBUG_PRINTF("Note: rf_chain %d configuration; en:%d freq:%d\n", rf_chain, rf_enable[rf_chain], rf_rx_freq[rf_chain]);
	
	return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_rxif_setconf(uint8_t if_chain, struct lgw_conf_rxif_s conf) {
	
	/* check if the concentrator is running */
	if (lgw_is_started == true) {
		DEBUG_MSG("ERROR: CONCENTRATOR IS RUNNING, STOP IT BEFORE TOUCHING CONFIGURATION\n");
		return LGW_HAL_ERROR;
	}
	
	/* check input range (segfault prevention) */
	if (if_chain >= LGW_IF_CHAIN_NB) {
		DEBUG_PRINTF("ERROR: %d NOT A VALID IF_CHAIN NUMBER\n", if_chain);
		return LGW_HAL_ERROR;
	}
	
	/* if chain is disabled, don't care about most parameters */
	if (conf.enable == false) {
		if_enable[if_chain] = false;
		if_freq[if_chain] = 0;
		DEBUG_PRINTF("Note: if_chain %d disabled\n", if_chain);
		return LGW_HAL_SUCCESS;
	}
	
	/* check 'general' parameters */
	if (ifmod_config[if_chain] == IF_UNDEFINED) {
		DEBUG_PRINTF("ERROR: IF CHAIN %d NOT CONFIGURABLE\n", if_chain);
	}
	if (conf.rf_chain >= LGW_RF_CHAIN_NB) {
		DEBUG_MSG("ERROR: INVALID RF_CHAIN TO ASSOCIATE WITH A LORA_STD IF CHAIN\n");
		return LGW_HAL_ERROR;
	}
	if ((conf.freq_hz + LGW_REF_BW/2) > ((int32_t)rf_rx_bandwidth[conf.rf_chain] / 2)) {
		DEBUG_PRINTF("ERROR: IF FREQUENCY %d TOO HIGH\n", conf.freq_hz);
		return LGW_HAL_ERROR;
	} else if ((conf.freq_hz - LGW_REF_BW/2) < -((int32_t)rf_rx_bandwidth[conf.rf_chain] / 2)) {
		DEBUG_PRINTF("ERROR: IF FREQUENCY %d TOO LOW\n", conf.freq_hz);
		return LGW_HAL_ERROR;
	}
	/* WARNING: if the channel is 250 or 500kHz wide, that check is insufficient */
	
	/* check parameters according to the type of IF chain + modem, 
	fill default if necessary, and commit configuration if everything is OK */
	switch (ifmod_config[if_chain]) {
		case IF_LORA_STD:
			/* fill default parameters if needed */
			if (conf.bandwidth == BW_UNDEFINED) {
				conf.bandwidth = BW_250KHZ;
			}
			if (conf.datarate == DR_UNDEFINED) {
				conf.datarate = DR_LORA_SF9;
			}
			/* check BW & DR */
			if (!IS_LORA_BW(conf.bandwidth)) {
				DEBUG_MSG("ERROR: BANDWIDTH NOT SUPPORTED BY LORA_STD IF CHAIN\n");
				return LGW_HAL_ERROR;
			}
			if (!IS_LORA_STD_DR(conf.datarate)) {
				DEBUG_MSG("ERROR: DATARATE NOT SUPPORTED BY LORA_STD IF CHAIN\n");
				return LGW_HAL_ERROR;
			}
			/* set internal configuration  */
			if_enable[if_chain] = conf.enable;
			if_rf_chain[if_chain] = conf.rf_chain;
			if_freq[if_chain] = conf.freq_hz;
			lora_rx_bw = conf.bandwidth;
			lora_rx_sf = (uint8_t)(DR_LORA_MULTI & conf.datarate); /* filter SF out of the 7-12 range */
			if (SET_PPM_ON(conf.bandwidth, conf.datarate)) {
				lora_rx_ppm_offset = true;
			} else {
				lora_rx_ppm_offset = false;
			}
			
			DEBUG_PRINTF("Note: LoRa 'std' if_chain %d configuration; en:%d freq:%d bw:%d dr:%d\n", if_chain, if_enable[if_chain], if_freq[if_chain], lora_rx_bw, lora_rx_sf);
			break;
		
		case IF_LORA_MULTI:
			/* fill default parameters if needed */
			if (conf.bandwidth == BW_UNDEFINED) {
				conf.bandwidth = BW_125KHZ;
			}
			if (conf.datarate == DR_UNDEFINED) {
				conf.datarate = DR_LORA_MULTI;
			}
			/* check BW & DR */
			if (conf.bandwidth != BW_125KHZ) {
				DEBUG_MSG("ERROR: BANDWIDTH NOT SUPPORTED BY LORA_MULTI IF CHAIN\n");
				return LGW_HAL_ERROR;
			}
			if (!IS_LORA_MULTI_DR(conf.datarate)) {
				DEBUG_MSG("ERROR: DATARATE(S) NOT SUPPORTED BY LORA_MULTI IF CHAIN\n");
				return LGW_HAL_ERROR;
			}
			/* set internal configuration  */
			if_enable[if_chain] = conf.enable;
			if_rf_chain[if_chain] = conf.rf_chain;
			if_freq[if_chain] = conf.freq_hz;
			lora_multi_sfmask[if_chain] = (uint8_t)(DR_LORA_MULTI & conf.datarate); /* filter SF out of the 7-12 range */
			
			DEBUG_PRINTF("Note: LoRa 'multi' if_chain %d configuration; en:%d freq:%d SF_mask:0x%02x\n", if_chain, if_enable[if_chain], if_freq[if_chain], lora_multi_sfmask[if_chain]);
			break;
		
		case IF_FSK_STD:
			/* fill default parameters if needed */
			if (conf.bandwidth == BW_UNDEFINED) {
				conf.bandwidth = BW_250KHZ;
			}
			if (conf.datarate == DR_UNDEFINED) {
				conf.datarate = 64000; /* default datarate */
			}
			/* check BW & DR */
			if(!IS_FSK_BW(conf.bandwidth)) {
				DEBUG_MSG("ERROR: BANDWIDTH NOT SUPPORTED BY FSK IF CHAIN\n");
				return LGW_HAL_ERROR;
			}
			if(!IS_FSK_DR(conf.datarate)) {
				DEBUG_MSG("ERROR: DATARATE NOT SUPPORTED BY FSK IF CHAIN\n");
				return LGW_HAL_ERROR;
			}
			/* set internal configuration  */
			if_enable[if_chain] = conf.enable;
			if_rf_chain[if_chain] = conf.rf_chain;
			if_freq[if_chain] = conf.freq_hz;
			fsk_rx_bw = conf.bandwidth;
			fsk_rx_dr = conf.datarate;
			if (conf.sync_word > 0) {
				fsk_sync_word_size = conf.sync_word_size;
				fsk_sync_word = conf.sync_word;
			}	
			DEBUG_PRINTF("Note: FSK if_chain %d configuration; en:%d freq:%d bw:%d dr:%d (%d real dr) sync:0x%0*llX\n", if_chain, if_enable[if_chain], if_freq[if_chain], fsk_rx_bw, fsk_rx_dr, LGW_XTAL_FREQU/(LGW_XTAL_FREQU/fsk_rx_dr), 2*fsk_sync_word_size, fsk_sync_word);
			break;
		
		default:
			DEBUG_PRINTF("ERROR: IF CHAIN %d TYPE NOT SUPPORTED\n", if_chain);
			return LGW_HAL_ERROR;
	}
	
	return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_start(void) {
	int i;
	int reg_stat;
	unsigned x;
	uint8_t radio_select;
	int32_t read_val;
	uint8_t load_val;
	
	uint8_t cal_cmd;
	uint16_t cal_time;
	uint8_t cal_status;
	
	uint64_t fsk_sync_word_reg;
	
	if (lgw_is_started == true) {
		DEBUG_MSG("Note: LoRa concentrator already started, restarting it now\n");
	}
	
	reg_stat = lgw_connect();
	if (reg_stat == LGW_REG_ERROR) {
		DEBUG_MSG("ERROR: FAIL TO CONNECT BOARD\n");
		return LGW_HAL_ERROR;
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
	setup_sx125x(0, rf_rx_freq[0]);
	setup_sx125x(1, rf_rx_freq[1]);
	
	/* select calibration command */
	cal_cmd = 0;
	cal_cmd |= rf_enable[0] ? 0x01 : 0x00; /* Bit 0: Calibrate Rx IQ mismatch compensation on radio A */
	cal_cmd |= rf_enable[1] ? 0x02 : 0x00; /* Bit 1: Calibrate Rx IQ mismatch compensation on radio B */
	cal_cmd |= (rf_enable[0] && rf_tx_enable[0]) ? 0x04 : 0x00; /* Bit 2: Calibrate Tx DC offset on radio A */
	cal_cmd |= (rf_enable[1] && rf_tx_enable[1]) ? 0x08 : 0x00; /* Bit 3: Calibrate Tx DC offset on radio B */
	cal_cmd |= 0x10; /* Bit 4: 0: calibrate with DAC gain=2, 1: with DAC gain=3 (use 3) */
	
	#if (CFG_RADIO_1257 == 1)
	cal_cmd |= 0x00; /* Bit 5: 0: SX1257, 1: SX1255 */
	#elif (CFG_RADIO_1255 == 1)
	cal_cmd |= 0x20; /* Bit 5: 0: SX1257, 1: SX1255 */
	#endif
	
	#if ((CFG_BRD_1301REF868 == 1) || (CFG_BRD_1301REF433 == 1) || (CFG_BRD_KERLINK868 == 1) || (CFG_BRD_KERLINK433 == 1) || (CFG_BRD_CISCO433 == 1) || (CFG_BRD_CISCO470 == 1) || (CFG_BRD_CISCO780 == 1))
	cal_cmd |= 0x00; /* Bit 6-7: Board type 0: ref, 1: FPGA, 3: board X */
	cal_time = 2300; /* measured between 2.1 and 2.2 sec, because 1 TX only */
	#elif (CFG_BRD_NANO868 == 1)
	cal_cmd |= 0x40; /* Bit 6-7: Board type 0: ref, 1: FPGA, 3: board X */
	cal_time = 5200; /* measured between 5.0 and 5.1 sec */
	#else
	cal_cmd |= 0xC0; /* Bit 6-7: Board type 0: ref, 1: FPGA, 3: board X */
	cal_time = 4200; /* measured between 4.0 and 4.1 sec */
	#endif
	
	/* Load the calibration firmware  */
	load_firmware(MCU_AGC, cal_firmware, MCU_AGC_FW_BYTE);
	lgw_reg_w(LGW_FORCE_HOST_RADIO_CTRL,0); /* gives to AGC MCU the control of the radios */
	lgw_reg_w(LGW_RADIO_SELECT,cal_cmd); /* send calibration configuration word */
	lgw_reg_w(LGW_MCU_RST_1,0);
	lgw_reg_w(LGW_PAGE_REG,3); /* Calibration will start on this condition as soon as MCU can talk to concentrator registers */
	lgw_reg_w(LGW_EMERGENCY_FORCE_HOST_CTRL,0); /* Give control of concentrator registers to MCU */
	
	/* Wait for calibration to end */
	DEBUG_PRINTF("Note: calibration started (time: %u ms)\n", cal_time);
	wait_ms(cal_time); /* Wait for end of calibration */
	lgw_reg_w(LGW_EMERGENCY_FORCE_HOST_CTRL,1); /* Take back control */
	
	/* Get calibration status */
	lgw_reg_r(LGW_MCU_AGC_STATUS, &read_val);
	cal_status = (uint8_t)read_val;
	/*
		bit 7: calibration finished
		bit 0: could access SX1301 registers
		bit 1: could access radio A registers
		bit 2: could access radio B registers
		bit 3: radio A RX image rejection successful
		bit 4: radio B RX image rejection successful
		bit 5: radio A TX imbalance correction successful
		bit 6: radio B TX imbalance correction successful
	*/
	if ((cal_status & 0x81) != 0x81) {
		DEBUG_PRINTF("ERROR: CALIBRATION FAILURE (STATUS = %u)\n", cal_status);
		return LGW_HAL_ERROR;
	} else {
		DEBUG_PRINTF("Note: calibration finished (status = %u)\n", cal_status);
	}
	if (rf_enable[0] && ((cal_status & 0x02) == 0)) {
		DEBUG_MSG("WARNING: calibration could not access radio A\n");
	}
	if (rf_enable[1] && ((cal_status & 0x04) == 0)) {
		DEBUG_MSG("WARNING: calibration could not access radio B\n");
	}
	if (rf_enable[0] && ((cal_status & 0x08) == 0)) {
		DEBUG_MSG("WARNING: problem in calibration of radio A for image rejection\n");
	}
	if (rf_enable[1] && ((cal_status & 0x10) == 0)) {
		DEBUG_MSG("WARNING: problem in calibration of radio B for image rejection\n");
	}
	if (rf_enable[0] && rf_tx_enable[0] && ((cal_status & 0x20) == 0)) {
		DEBUG_MSG("WARNING: problem in calibration of radio A for TX imbalance\n");
	}
	if (rf_enable[1] && rf_tx_enable[1] && ((cal_status & 0x40) == 0)) {
		DEBUG_MSG("WARNING: problem in calibration of radio B for TX imbalance\n");
	}
	
	/* Get TX DC offset values */
	for(i=0; i<=7; ++i) {
		lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0xA0+i);
		lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &read_val);
		cal_offset_a_i[i] = (int8_t)read_val;
		lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0xA8+i);
		lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &read_val);
		cal_offset_a_q[i] = (int8_t)read_val;
		lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0xB0+i);
		lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &read_val);
		cal_offset_b_i[i] = (int8_t)read_val;
		lgw_reg_w(LGW_DBG_AGC_MCU_RAM_ADDR, 0xB8+i);
		lgw_reg_r(LGW_DBG_AGC_MCU_RAM_DATA, &read_val);
		cal_offset_b_q[i] = (int8_t)read_val;
	}
	
	/* load adjusted parameters */
	lgw_constant_adjust();
	
	/* Freq-to-time-drift calculation */
	x = (2 * 8192000000) / (uint64_t)(rf_rx_lowfreq[0] + rf_rx_upfreq[0]); /* 64b calculation */
	if (x > 63) {
		x = 63;
	}
	lgw_reg_w(LGW_FREQ_TO_TIME_DRIFT, x); /* default 9 */
	x = (2 * 32768000000) / (uint64_t)(rf_rx_lowfreq[0] + rf_rx_upfreq[0]); /* 64b calculation */
	if (x > 63) {
		x = 63;
	}
	lgw_reg_w(LGW_MBWSSF_FREQ_TO_TIME_DRIFT, x); /* default 36 */
	
	/* configure LoRa 'multi' demodulators aka. LoRa 'sensor' channels (IF0-3) */
	
	radio_select = 0; /* IF mapping to radio A/B (per bit, 0=A, 1=B) */
	for(i=0; i<LGW_MULTI_NB; ++i) {
		radio_select += (if_rf_chain[i] == 1 ? 1 << i : 0); /* transform bool array into binary word */
	}
	/*
	lgw_reg_w(LGW_RADIO_SELECT, radio_select);
	
	LGW_RADIO_SELECT is used for communication with the firmware, "radio_select"
	will be loaded in LGW_RADIO_SELECT at the end of start procedure.
	*/
	
	lgw_reg_w(LGW_IF_FREQ_0, IF_HZ_TO_REG(if_freq[0])); /* default -384 */
	lgw_reg_w(LGW_IF_FREQ_1, IF_HZ_TO_REG(if_freq[1])); /* default -128 */
	lgw_reg_w(LGW_IF_FREQ_2, IF_HZ_TO_REG(if_freq[2])); /* default 128 */
	lgw_reg_w(LGW_IF_FREQ_3, IF_HZ_TO_REG(if_freq[3])); /* default 384 */
	#if (CFG_CHIP_1301 == 1)
	lgw_reg_w(LGW_IF_FREQ_4, IF_HZ_TO_REG(if_freq[4])); /* default -384 */
	lgw_reg_w(LGW_IF_FREQ_5, IF_HZ_TO_REG(if_freq[5])); /* default -128 */
	lgw_reg_w(LGW_IF_FREQ_6, IF_HZ_TO_REG(if_freq[6])); /* default 128 */
	lgw_reg_w(LGW_IF_FREQ_7, IF_HZ_TO_REG(if_freq[7])); /* default 384 */
	#endif
	
	lgw_reg_w(LGW_CORR0_DETECT_EN, (if_enable[0] == true) ? lora_multi_sfmask[0] : 0); /* default 0 */
	lgw_reg_w(LGW_CORR1_DETECT_EN, (if_enable[1] == true) ? lora_multi_sfmask[1] : 0); /* default 0 */
	lgw_reg_w(LGW_CORR2_DETECT_EN, (if_enable[2] == true) ? lora_multi_sfmask[2] : 0); /* default 0 */
	lgw_reg_w(LGW_CORR3_DETECT_EN, (if_enable[3] == true) ? lora_multi_sfmask[3] : 0); /* default 0 */
	#if (CFG_CHIP_1301 == 1)
	lgw_reg_w(LGW_CORR4_DETECT_EN, (if_enable[4] == true) ? lora_multi_sfmask[4] : 0); /* default 0 */
	lgw_reg_w(LGW_CORR5_DETECT_EN, (if_enable[5] == true) ? lora_multi_sfmask[5] : 0); /* default 0 */
	lgw_reg_w(LGW_CORR6_DETECT_EN, (if_enable[6] == true) ? lora_multi_sfmask[6] : 0); /* default 0 */
	lgw_reg_w(LGW_CORR7_DETECT_EN, (if_enable[7] == true) ? lora_multi_sfmask[7] : 0); /* default 0 */
	#endif
	
	lgw_reg_w(LGW_PPM_OFFSET, 0x60); /* as the threshold is 16ms, use 0x60 to enable ppm_offset for SF12 and SF11 @125kHz*/
	
	lgw_reg_w(LGW_CONCENTRATOR_MODEM_ENABLE,1); /* default 0 */
	
	/* configure LoRa 'stand-alone' modem (IF8) */
	lgw_reg_w(LGW_IF_FREQ_8, IF_HZ_TO_REG(if_freq[8])); /* MBWSSF modem (default 0) */
	if (if_enable[8] == true) {
		lgw_reg_w(LGW_MBWSSF_RADIO_SELECT, if_rf_chain[8]);
		switch(lora_rx_bw) {
			case BW_125KHZ: lgw_reg_w(LGW_MBWSSF_MODEM_BW,0); break;
			case BW_250KHZ: lgw_reg_w(LGW_MBWSSF_MODEM_BW,1); break;
			case BW_500KHZ: lgw_reg_w(LGW_MBWSSF_MODEM_BW,2); break;
			default:
				DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT\n", lora_rx_bw);
				return LGW_HAL_ERROR;
		}
		switch(lora_rx_sf) {
			case DR_LORA_SF7: lgw_reg_w(LGW_MBWSSF_RATE_SF,7); break;
			case DR_LORA_SF8: lgw_reg_w(LGW_MBWSSF_RATE_SF,8); break;
			case DR_LORA_SF9: lgw_reg_w(LGW_MBWSSF_RATE_SF,9); break;
			case DR_LORA_SF10: lgw_reg_w(LGW_MBWSSF_RATE_SF,10); break;
			case DR_LORA_SF11: lgw_reg_w(LGW_MBWSSF_RATE_SF,11); break;
			case DR_LORA_SF12: lgw_reg_w(LGW_MBWSSF_RATE_SF,12); break;
			default:
				DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT\n", lora_rx_sf);
				return LGW_HAL_ERROR;
		}
		lgw_reg_w(LGW_MBWSSF_PPM_OFFSET, lora_rx_ppm_offset); /* default 0 */
		lgw_reg_w(LGW_MBWSSF_MODEM_ENABLE, 1); /* default 0 */
	} else {
		lgw_reg_w(LGW_MBWSSF_MODEM_ENABLE, 0);
	}
	
	/* configure FSK modem (IF9) */
	lgw_reg_w(LGW_IF_FREQ_9, IF_HZ_TO_REG(if_freq[9])); /* FSK modem, default 0 */
	lgw_reg_w(LGW_FSK_PSIZE, fsk_sync_word_size-1);
	lgw_reg_w(LGW_FSK_TX_PSIZE, fsk_sync_word_size-1);
	fsk_sync_word_reg = fsk_sync_word << (8 * (8 - fsk_sync_word_size));
	lgw_reg_w(LGW_FSK_REF_PATTERN_LSB, (uint32_t)(0xFFFFFFFF & fsk_sync_word_reg));
	lgw_reg_w(LGW_FSK_REF_PATTERN_MSB, (uint32_t)(0xFFFFFFFF & (fsk_sync_word_reg >> 32)));
	if (if_enable[9] == true) {
		lgw_reg_w(LGW_FSK_RADIO_SELECT, if_rf_chain[9]);
		lgw_reg_w(LGW_FSK_BR_RATIO,LGW_XTAL_FREQU/fsk_rx_dr); /* setting the dividing ratio for datarate */
		lgw_reg_w(LGW_FSK_CH_BW_EXPO,fsk_rx_bw);
		lgw_reg_w(LGW_FSK_MODEM_ENABLE,1); /* default 0 */
	} else {
		lgw_reg_w(LGW_FSK_MODEM_ENABLE,0);
	}
	
	/* Load firmware */
	load_firmware(MCU_ARB, arb_firmware, MCU_ARB_FW_BYTE);
	load_firmware(MCU_AGC, agc_firmware, MCU_AGC_FW_BYTE);
	
	/* gives the AGC MCU control over radio, RF front-end and filter gain */
	lgw_reg_w(LGW_FORCE_HOST_RADIO_CTRL,0);
	lgw_reg_w(LGW_FORCE_HOST_FE_CTRL,0);
	lgw_reg_w(LGW_FORCE_DEC_FILTER_GAIN,0);
	
	/* Get MCUs out of reset */
	lgw_reg_w(LGW_RADIO_SELECT, 0); /* MUST not be = to 1 or 2 at firmware init */
	lgw_reg_w(LGW_MCU_RST_0, 0);
	lgw_reg_w(LGW_MCU_RST_1, 0);
	
	DEBUG_MSG("Info: Initialising AGC firmware...\n");
	wait_ms(1);
	
	lgw_reg_r(LGW_MCU_AGC_STATUS, &read_val);
	if (read_val != 0x10) {
		DEBUG_PRINTF("ERROR: AGC FIRMWARE INITIALIZATION FAILURE, STATUS 0x%02X\n", (uint8_t)read_val);
		return LGW_HAL_ERROR;
	}
	
	/* Update Tx gain LUT and start AGC */
	
	#if (CUSTOM_TX_POW_TABLE == 1)
		DEBUG_MSG("Info: loading custom TX gain table\n");
		for(i=0; i<TX_POW_LUT_SIZE; ++i) {
			lgw_reg_w(LGW_RADIO_SELECT, AGC_CMD_WAIT); /* start a transaction */
			wait_ms(1);
			load_val = tx_pow_table[i].mix_gain + (16 * tx_pow_table[i].dac_gain) + (64 * tx_pow_table[i].pa_gain);
			lgw_reg_w(LGW_RADIO_SELECT, load_val);
			wait_ms(1);
			lgw_reg_r(LGW_MCU_AGC_STATUS, &read_val);
			if (read_val != (0x30 + i)) {
				DEBUG_PRINTF("ERROR: AGC FIRMWARE INITIALIZATION FAILURE, STATUS 0x%02X\n", (uint8_t)read_val);
				return LGW_HAL_ERROR;
			}
		}
	#else
		lgw_reg_w(LGW_RADIO_SELECT, AGC_CMD_WAIT); /* start a transaction */
		wait_ms(1);
		load_val = AGC_CMD_ABORT;
		lgw_reg_w(LGW_RADIO_SELECT, load_val); 
		wait_ms(1);
		DEBUG_MSG("Info: TX gain LUT update skipped, using default LUT\n");
		lgw_reg_r(LGW_MCU_AGC_STATUS, &read_val);
		if (read_val != 0x30) {
			DEBUG_PRINTF("ERROR: AGC FIRMWARE INITIALIZATION FAILURE, STATUS 0x%02X\n", (uint8_t)read_val);
			return LGW_HAL_ERROR;
		}
	#endif
	
	/* Load Tx freq MSBs (always 3 if f > 768 for SX1257 or f > 384 for SX1255 */
	lgw_reg_w(LGW_RADIO_SELECT, AGC_CMD_WAIT);
	wait_ms(1);
	lgw_reg_w(LGW_RADIO_SELECT, 3);
	wait_ms(1);
	
	/* Load chan_select firmware option */
	lgw_reg_w(LGW_RADIO_SELECT, AGC_CMD_WAIT);
	wait_ms(1);
	lgw_reg_w(LGW_RADIO_SELECT, 0);
	wait_ms(1);
	
	/* End AGC firmware init and check status */
	lgw_reg_w(LGW_RADIO_SELECT, AGC_CMD_WAIT);
	wait_ms(1);
	lgw_reg_w(LGW_RADIO_SELECT, radio_select); /* Load intended value of RADIO_SELECT */
	wait_ms(1);
	DEBUG_MSG("Info: putting back original RADIO_SELECT value\n");
	lgw_reg_r(LGW_MCU_AGC_STATUS, &read_val);
	if (read_val != 0x40) {
		DEBUG_PRINTF("ERROR: AGC FIRMWARE INITIALIZATION FAILURE, STATUS 0x%02X\n", (uint8_t)read_val);
		return LGW_HAL_ERROR;
	}
	
	/* enable GPS event capture */
	lgw_reg_w(LGW_GPS_EN,1);
	
	/* enable LEDs */
	lgw_reg_w(LGW_GPIO_MODE,31);
	// lgw_reg_w(LGW_GPIO_SELECT_OUTPUT,0); /* default 0 */
	/* LED table :
	DGPIO0 -> packets waiting in the RX FIFO, ready to be fetched
	DGPIO1 -> multi-SF RX LoRa modems activity (channels 0 to 7)
	DGPIO2 -> stand-alone RX LoRa modem activity (channel 8))
	DGPIO3 -> FSK RX modem activity (channel 9)
	DGPIO4 -> TX modem active (either LoRa or FSK)
	*/
	
	lgw_is_started = true;
	return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_stop(void) {
	lgw_soft_reset();
	lgw_disconnect();
	
	lgw_is_started = false;
	return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_receive(uint8_t max_pkt, struct lgw_pkt_rx_s *pkt_data) {
	int nb_pkt_fetch; /* loop variable and return value */
	struct lgw_pkt_rx_s *p; /* pointer to the current structure in the struct array */
	uint8_t buff[255+RX_METADATA_NB]; /* buffer to store the result of SPI read bursts */
	unsigned sz; /* size of the payload, uses to address metadata */
	int ifmod; /* type of if_chain/modem a packet was received by */
	int stat_fifo; /* the packet status as indicated in the FIFO */
	uint32_t raw_timestamp; /* timestamp when internal 'RX finished' was triggered */
	uint32_t delay_x, delay_y, delay_z; /* temporary variable for timestamp offset calculation */
	uint32_t timestamp_correction; /* correction to account for processing delay */
	uint32_t sf, cr, bw_pow, crc_en, ppm; /* used to calculate timestamp correction */
	
	/* check if the concentrator is running */
	if (lgw_is_started == false) {
		DEBUG_MSG("ERROR: CONCENTRATOR IS NOT RUNNING, START IT BEFORE RECEIVING\n");
		return LGW_HAL_ERROR;
	}
	
	/* check input variables */
	if (max_pkt <= 0) {
		DEBUG_PRINTF("ERROR: %d = INVALID MAX NUMBER OF PACKETS TO FETCH\n", max_pkt);
		return LGW_HAL_ERROR;
	}
	CHECK_NULL(pkt_data);
	
	/* iterate max_pkt times at most */
	for (nb_pkt_fetch = 0; nb_pkt_fetch < max_pkt; ++nb_pkt_fetch) {
		
		/* point to the proper struct in the struct array */
		p = &pkt_data[nb_pkt_fetch];
		
		/* fetch all the RX FIFO data */
		lgw_reg_rb(LGW_RX_PACKET_DATA_FIFO_NUM_STORED, buff, 5);
		
		/* how many packets are in the RX buffer ? Break if zero */
		if (buff[0] == 0) {
			break; /* no more packets to fetch, exit out of FOR loop */
		}
		
		DEBUG_PRINTF("FIFO content: %x %x %x %x %x\n",buff[0],buff[1],buff[2],buff[3],buff[4]);
		
		p->size = buff[4];
		sz = p->size;
		stat_fifo = buff[3]; /* will be used later, need to save it before overwriting buff */
		
		/* get payload + metadata */
		lgw_reg_rb(LGW_RX_DATA_BUF_DATA, buff, sz+RX_METADATA_NB);
		
		/* copy payload to result struct */
		memcpy((void *)p->payload, (void *)buff, sz);
		
		/* process metadata */
		p->if_chain = buff[sz+0];
		ifmod = ifmod_config[p->if_chain];
		DEBUG_PRINTF("[%d %d]\n", p->if_chain, ifmod);
		p->rssi = (float)buff[sz+5] - RSSI_BOARD_OFFSET;
		
		if ((ifmod == IF_LORA_MULTI) || (ifmod == IF_LORA_STD)) {
			DEBUG_MSG("Note: LoRa packet\n");
			switch(stat_fifo & 0x07) {
				case 5:
					p->status = STAT_CRC_OK;
					crc_en = 1;
					break;
				case 7:
					p->status = STAT_CRC_BAD;
					crc_en = 1;
					break;
				case 1:
					p->status = STAT_NO_CRC;
					crc_en = 0;
					break;
				default:
					p->status = STAT_UNDEFINED;
					crc_en = 0;
			}
			p->modulation = MOD_LORA;
			p->snr = ((float)((int8_t)buff[sz+2]))/4;
			p->snr_min = ((float)((int8_t)buff[sz+3]))/4;
			p->snr_max = ((float)((int8_t)buff[sz+4]))/4;
			if (ifmod == IF_LORA_MULTI) {
				p->bandwidth = BW_125KHZ; /* fixed in hardware */
			} else {
				p->bandwidth = lora_rx_bw; /* get the parameter from the config variable */
			}
			sf = (buff[sz+1] >> 4) & 0x0F;
			switch (sf) {
				case 7: p->datarate = DR_LORA_SF7; break;
				case 8: p->datarate = DR_LORA_SF8; break;
				case 9: p->datarate = DR_LORA_SF9; break;
				case 10: p->datarate = DR_LORA_SF10; break;
				case 11: p->datarate = DR_LORA_SF11; break;
				case 12: p->datarate = DR_LORA_SF12; break;
				default: p->datarate = DR_UNDEFINED;
			}
			cr = (buff[sz+1] >> 1) & 0x07;
			switch (cr) {
				case 1: p->coderate = CR_LORA_4_5; break;
				case 2: p->coderate = CR_LORA_4_6; break;
				case 3: p->coderate = CR_LORA_4_7; break;
				case 4: p->coderate = CR_LORA_4_8; break;
				default: p->coderate = CR_UNDEFINED;
			}
			
			/* determine if 'PPM mode' is on, needed for timestamp correction */
			if (SET_PPM_ON(p->bandwidth,p->datarate)) {
				ppm = 1;
			} else {
				ppm = 0;
			}
			
			/* timestamp correction code, base delay */
			if (ifmod == IF_LORA_STD) { /* if packet was received on the stand-alone LoRa modem */
				switch (lora_rx_bw) {
					case BW_125KHZ:
						delay_x = 64;
						bw_pow = 1;
						break;
					case BW_250KHZ:
						delay_x = 32;
						bw_pow = 2;
						break;
					case BW_500KHZ:
						delay_x = 16;
						bw_pow = 4;
						break;
					default:
						DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT\n", p->bandwidth);	
						delay_x = 0;
						bw_pow = 0;
				}
			} else { /* packet was received on one of the sensor channels = 125kHz */
				delay_x = 114;
				bw_pow = 1;
			}
			
			/* timestamp correction code, variable delay */
			if ((sf >= 6) && (sf <= 12) && (bw_pow > 0)) {
				if ((2*(sz + 2*crc_en) - (sf-7)) <= 0) { /* payload fits entirely in first 8 symbols */
					delay_y = ( ((1<<(sf-1)) * (sf+1)) + (3 * (1<<(sf-4))) ) / bw_pow;
					delay_z = 32 * (2*(sz+2*crc_en) + 5) / bw_pow;
				} else {
					delay_y = ( ((1<<(sf-1)) * (sf+1)) + ((4 - ppm) * (1<<(sf-4))) ) / bw_pow;
					delay_z = (16 + 4*cr) * (((2*(sz+2*crc_en)-sf+6) % (sf - 2*ppm)) + 1) / bw_pow;
				}
				timestamp_correction = delay_x + delay_y + delay_z;
			} else {
				timestamp_correction = 0;
				DEBUG_MSG("WARNING: invalid packet, no timestamp correction\n");
			}
			
			/* RSSI correction */
			if (ifmod == IF_LORA_MULTI) {
				p->rssi -= RSSI_MULTI_BIAS;
			}
			
		} else if (ifmod == IF_FSK_STD) {
			DEBUG_MSG("Note: FSK packet\n");
			switch(stat_fifo & 0x07) {
				case 5: p->status = STAT_CRC_OK; break;
				case 7: p->status = STAT_CRC_BAD; break;
				case 1: p->status = STAT_NO_CRC; break;
				default: p->status = STAT_UNDEFINED;
			}
			p->modulation = MOD_FSK;
			p->snr = -128.0;
			p->snr_min = -128.0;
			p->snr_max = -128.0;
			p->bandwidth = fsk_rx_bw;
			p->datarate = fsk_rx_dr;
			p->coderate = CR_UNDEFINED;
			timestamp_correction = ((uint32_t)680000 / fsk_rx_dr) - 20;
			
			/* RSSI correction */
			p->rssi -= RSSI_FSK_BIAS;
			p->rssi = ((p->rssi - RSSI_FSK_REF) * RSSI_FSK_SLOPE) + RSSI_FSK_REF;
		} else {
			DEBUG_MSG("ERROR: UNEXPECTED PACKET ORIGIN\n");
			p->status = STAT_UNDEFINED;
			p->modulation = MOD_UNDEFINED;
			p->rssi = -128.0;
			p->snr = -128.0;
			p->snr_min = -128.0;
			p->snr_max = -128.0;
			p->bandwidth = BW_UNDEFINED;
			p->datarate = DR_UNDEFINED;
			p->coderate = CR_UNDEFINED;
			timestamp_correction = 0;
		}
		
		raw_timestamp = (uint32_t)buff[sz+6] + ((uint32_t)buff[sz+7] << 8) + ((uint32_t)buff[sz+8] << 16) + ((uint32_t)buff[sz+9] << 24);
		p->count_us = raw_timestamp - timestamp_correction;
		p->crc = (uint16_t)buff[sz+10] + ((uint16_t)buff[sz+11] << 8);
		
		/* get back info from configuration so that application doesn't have to keep track of it */
		p->rf_chain = (uint8_t)if_rf_chain[p->if_chain];
		p->freq_hz = (uint32_t)((int32_t)rf_rx_freq[p->rf_chain] + if_freq[p->if_chain]);
		
		/* advance packet FIFO */
		lgw_reg_w(LGW_RX_PACKET_DATA_FIFO_NUM_STORED, 0);
	}
	
	return nb_pkt_fetch;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_send(struct lgw_pkt_tx_s pkt_data) {
	int i;
	uint8_t buff[256+TX_METADATA_NB]; /* buffer to prepare the packet to send + metadata before SPI write burst */
	uint32_t part_int; /* integer part for PLL register value calculation */
	uint32_t part_frac; /* fractional part for PLL register value calculation */
	uint16_t fsk_dr_div; /* divider to configure for target datarate */
	int transfer_size = 0; /* data to transfer from host to TX databuffer */
	int payload_offset = 0; /* start of the payload content in the databuffer */
	uint8_t pow_index = 0; /* 4-bit value to set the firmware TX power */
	uint8_t target_mix_gain = 0; /* used to select the proper I/Q offset correction */
	uint32_t count_trig; /* timestamp value in trigger mode corrected for TX start delay */
	
	/* check if the concentrator is running */
	if (lgw_is_started == false) {
		DEBUG_MSG("ERROR: CONCENTRATOR IS NOT RUNNING, START IT BEFORE SENDING\n");
		return LGW_HAL_ERROR;
	}
	
	/* check input range (segfault prevention) */
	if (pkt_data.rf_chain >= LGW_RF_CHAIN_NB) {
		DEBUG_MSG("ERROR: INVALID RF_CHAIN TO SEND PACKETS\n");
		return LGW_HAL_ERROR;
	}
	
	/* check input variables */
	if (rf_tx_enable[pkt_data.rf_chain] == false) {
		DEBUG_MSG("ERROR: SELECTED RF_CHAIN IS DISABLED FOR TX ON SELECTED BOARD\n");
		return LGW_HAL_ERROR;
	}
	if (rf_enable[pkt_data.rf_chain] == false) {
		DEBUG_MSG("ERROR: SELECTED RF_CHAIN IS DISABLED\n");
		return LGW_HAL_ERROR;
	}
	if (pkt_data.freq_hz > rf_tx_upfreq[pkt_data.rf_chain]) {
		DEBUG_PRINTF("ERROR: FREQUENCY %d HIGHER THAN UPPER LIMIT %d OF RF_CHAIN %d\n", pkt_data.freq_hz, rf_tx_upfreq[pkt_data.rf_chain], pkt_data.rf_chain);
		return LGW_HAL_ERROR;
	} else if (pkt_data.freq_hz < rf_tx_lowfreq[pkt_data.rf_chain]) {
		DEBUG_PRINTF("ERROR: FREQUENCY %d LOWER THAN LOWER LIMIT %d OF RF_CHAIN %d\n", pkt_data.freq_hz, rf_tx_lowfreq[pkt_data.rf_chain], pkt_data.rf_chain);
		return LGW_HAL_ERROR;
	}
	if (!IS_TX_MODE(pkt_data.tx_mode)) {
		DEBUG_MSG("ERROR: TX_MODE NOT SUPPORTED\n");
		return LGW_HAL_ERROR;
	}
	if (pkt_data.modulation == MOD_LORA) {
		if (!IS_LORA_BW(pkt_data.bandwidth)) {
			DEBUG_MSG("ERROR: BANDWIDTH NOT SUPPORTED BY LORA TX\n");
			return LGW_HAL_ERROR;
		}
		if (!IS_LORA_STD_DR(pkt_data.datarate)) {
			DEBUG_MSG("ERROR: DATARATE NOT SUPPORTED BY LORA TX\n");
			return LGW_HAL_ERROR;
		}
		if (!IS_LORA_CR(pkt_data.coderate)) {
			DEBUG_MSG("ERROR: CODERATE NOT SUPPORTED BY LORA TX\n");
			return LGW_HAL_ERROR;
		}
		if (pkt_data.size > 255) {
			DEBUG_MSG("ERROR: PAYLOAD LENGTH TOO BIG FOR LORA TX\n");
			return LGW_HAL_ERROR;
		}
	} else if (pkt_data.modulation == MOD_FSK) {
		if((pkt_data.f_dev < 1) || (pkt_data.f_dev > 200)) {
			DEBUG_MSG("ERROR: TX FREQUENCY DEVIATION OUT OF ACCEPTABLE RANGE\n");
			return LGW_HAL_ERROR;
		}
		if(!IS_FSK_DR(pkt_data.datarate)) {
			DEBUG_MSG("ERROR: DATARATE NOT SUPPORTED BY FSK IF CHAIN\n");
			return LGW_HAL_ERROR;
		}
		if (pkt_data.size > 255) {
			DEBUG_MSG("ERROR: PAYLOAD LENGTH TOO BIG FOR FSK TX\n");
			return LGW_HAL_ERROR;
		}
	} else {
		DEBUG_MSG("ERROR: INVALID TX MODULATION\n");
		return LGW_HAL_ERROR;
	}
	
	/* interpretation of TX power */
	for (pow_index = TX_POW_LUT_SIZE-1; pow_index > 0; pow_index--) {
		if (tx_pow_table[pow_index].rf_power <= pkt_data.rf_power) {
			break;
		}
	}
	
	/* loading TX imbalance correction */
	target_mix_gain = tx_pow_table[pow_index].mix_gain;
	target_mix_gain = (target_mix_gain <  8)?  8 : target_mix_gain;
	target_mix_gain = (target_mix_gain > 15)? 15 : target_mix_gain;
	if (pkt_data.rf_chain == 0) { /* use radio A calibration table */
		lgw_reg_w(LGW_TX_OFFSET_I, cal_offset_a_i[target_mix_gain - 8]);
		lgw_reg_w(LGW_TX_OFFSET_Q, cal_offset_a_q[target_mix_gain - 8]);
	} else { /* use radio B calibration table */
		lgw_reg_w(LGW_TX_OFFSET_I, cal_offset_b_i[target_mix_gain - 8]);
		lgw_reg_w(LGW_TX_OFFSET_Q, cal_offset_b_q[target_mix_gain - 8]);
	}
	
	/* fixed metadata, useful payload and misc metadata compositing */
	transfer_size = TX_METADATA_NB + pkt_data.size; /*  */
	payload_offset = TX_METADATA_NB; /* start the payload just after the metadata */
	
	/* metadata 0 to 2, TX PLL frequency */
	#if (CFG_RADIO_1257 == 1)
	part_int = pkt_data.freq_hz / (SX125x_32MHz_FRAC << 8); /* integer part, gives the MSB */
	part_frac = ((pkt_data.freq_hz % (SX125x_32MHz_FRAC << 8)) << 8) / SX125x_32MHz_FRAC; /* fractional part, gives middle part and LSB */
	#elif (CFG_RADIO_1255 == 1)
	part_int = pkt_data.freq_hz / (SX125x_32MHz_FRAC << 7); /* integer part, gives the MSB */
	part_frac = ((pkt_data.freq_hz % (SX125x_32MHz_FRAC << 7)) << 9) / SX125x_32MHz_FRAC; /* fractional part, gives middle part and LSB */
	#endif
	
	buff[0] = 0xFF & part_int; /* Most Significant Byte */
	buff[1] = 0xFF & (part_frac >> 8); /* middle byte */
	buff[2] = 0xFF & part_frac; /* Least Significant Byte */
	
	/* metadata 3 to 6, timestamp trigger value */
	/* TX state machine must be triggered at T0 - TX_START_DELAY for packet to start being emitted at T0 */
	if (pkt_data.tx_mode == TIMESTAMPED)
	{
		count_trig = pkt_data.count_us - TX_START_DELAY;
		buff[3] = 0xFF & (count_trig >> 24);
		buff[4] = 0xFF & (count_trig >> 16);
		buff[5] = 0xFF & (count_trig >> 8);
		buff[6] = 0xFF &  count_trig;
	}
	
	/* parameters depending on modulation  */
	if (pkt_data.modulation == MOD_LORA) {
		/* metadata 7, modulation type, radio chain selection and TX power */
		buff[7] = (0x20 & (pkt_data.rf_chain << 5)) | (0x0F & pow_index); /* bit 4 is 0 -> LoRa modulation */
		
		buff[8] = 0; /* metadata 8, not used */
		
		/* metadata 9, CRC, LoRa CR & SF */
		switch (pkt_data.datarate) {
			case DR_LORA_SF7: buff[9] = 7; break;
			case DR_LORA_SF8: buff[9] = 8; break;
			case DR_LORA_SF9: buff[9] = 9; break;
			case DR_LORA_SF10: buff[9] = 10; break;
			case DR_LORA_SF11: buff[9] = 11; break;
			case DR_LORA_SF12: buff[9] = 12; break;
			default: DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT\n", pkt_data.datarate);
		}
		switch (pkt_data.coderate) {
			case CR_LORA_4_5: buff[9] |= 1 << 4; break;
			case CR_LORA_4_6: buff[9] |= 2 << 4; break;
			case CR_LORA_4_7: buff[9] |= 3 << 4; break;
			case CR_LORA_4_8: buff[9] |= 4 << 4; break;
			default: DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT\n", pkt_data.coderate);
		}
		if (pkt_data.no_crc == false) {
			buff[9] |= 0x80; /* set 'CRC enable' bit */
		} else {
			DEBUG_MSG("Info: packet will be sent without CRC\n");
		}
		
		/* metadata 10, payload size */
		buff[10] = pkt_data.size;
		
		/* metadata 11, implicit header, modulation bandwidth, PPM offset & polarity */
		switch (pkt_data.bandwidth) {
			case BW_125KHZ: buff[11] = 0; break;
			case BW_250KHZ: buff[11] = 1; break;
			case BW_500KHZ: buff[11] = 2; break;
			default: DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT\n", pkt_data.bandwidth);
		}
		if (pkt_data.no_header == true) {
			buff[11] |= 0x04; /* set 'implicit header' bit */
		}
		if (SET_PPM_ON(pkt_data.bandwidth,pkt_data.datarate)) {
			buff[11] |= 0x08; /* set 'PPM offset' bit at 1 */
		}
		if (pkt_data.invert_pol == true) {
			buff[11] |= 0x10; /* set 'TX polarity' bit at 1 */
		}
		
		/* metadata 12 & 13, LoRa preamble size */
		if (pkt_data.preamble == 0) { /* if not explicit, use recommended LoRa preamble size */
			pkt_data.preamble = STD_LORA_PREAMBLE;
		} else if (pkt_data.preamble < MIN_LORA_PREAMBLE) { /* enforce minimum preamble size */
			pkt_data.preamble = MIN_LORA_PREAMBLE;
			DEBUG_MSG("Note: preamble length adjusted to respect minimum LoRa preamble size\n");
		}
		buff[12] = 0xFF & (pkt_data.preamble >> 8);
		buff[13] = 0xFF & pkt_data.preamble;
		
		/* metadata 14 & 15, not used */
		buff[14] = 0;
		buff[15] = 0;
		
		/* MSB of RF frequency is now used in AGC firmware to implement large/narrow filtering in SX1257/55 */
		if (pkt_data.bandwidth == BW_500KHZ) {
			buff[0] |= 0x80; /* Enlarge filter for 500kHz BW */
		} else {
			buff[0] &= 0x7F;
		}
		
	} else if (pkt_data.modulation == MOD_FSK) {
		/* metadata 7, modulation type, radio chain selection and TX power */
		buff[7] = (0x20 & (pkt_data.rf_chain << 5)) | 0x10 | (0x0F & pow_index); /* bit 4 is 1 -> FSK modulation */
		
		buff[8] = 0; /* metadata 8, not used */
		
		/* metadata 9, frequency deviation */
		buff[9] = pkt_data.f_dev;
		
		/* metadata 10, payload size */
		buff[10] = pkt_data.size; 
		/* TODO: how to handle 255 bytes packets ?!? */
		
		/* metadata 11, packet mode, CRC, encoding */
		buff[11] = 0x01 | (pkt_data.no_crc?0:0x02) | (0x02 << 2); /* always in variable length packet mode, whitening, and CCITT CRC if CRC is not disabled  */
		
		/* metadata 12 & 13, FSK preamble size */
		if (pkt_data.preamble == 0) { /* if not explicit, use LoRa MAC preamble size */
			pkt_data.preamble = STD_FSK_PREAMBLE;
		} else if (pkt_data.preamble < MIN_FSK_PREAMBLE) { /* enforce minimum preamble size */
			pkt_data.preamble = MIN_FSK_PREAMBLE;
			DEBUG_MSG("Note: preamble length adjusted to respect minimum FSK preamble size\n");
		}
		buff[12] = 0xFF & (pkt_data.preamble >> 8);
		buff[13] = 0xFF & pkt_data.preamble;
		
		/* metadata 14 & 15, FSK baudrate */
		fsk_dr_div = (uint16_t)((uint32_t)LGW_XTAL_FREQU / pkt_data.datarate); /* Ok for datarate between 500bps and 250kbps */
		buff[14] = 0xFF & (fsk_dr_div >> 8);
		buff[15] = 0xFF & fsk_dr_div;
		
		/* insert payload size in the packet for variable mode */
		buff[16] = pkt_data.size;
		++transfer_size; /* one more byte to transfer to the TX modem */
		++payload_offset; /* start the payload with one more byte of offset */
		
		/* MSB of RF frequency is now used in AGC firmware to implement large/narrow filtering in SX1257/55 */
		buff[0] &= 0x7F; /* Always use narrow band for FSK (force MSB to 0) */
		
	} else {
		DEBUG_MSG("ERROR: INVALID TX MODULATION..\n");
		return LGW_HAL_ERROR;
	}
	
	/* copy payload from user struct to buffer containing metadata */
	memcpy((void *)(buff + payload_offset), (void *)(pkt_data.payload), pkt_data.size);
	
	/* reset TX command flags */
	lgw_abort_tx();
	
	/* put metadata + payload in the TX data buffer */
	lgw_reg_w(LGW_TX_DATA_BUF_ADDR, 0);
	lgw_reg_wb(LGW_TX_DATA_BUF_DATA, buff, transfer_size);
	DEBUG_ARRAY(i, transfer_size, buff);
	
	/* send data */
	switch(pkt_data.tx_mode) {
		case IMMEDIATE:
			lgw_reg_w(LGW_TX_TRIG_IMMEDIATE, 1);
			break;
			
		case TIMESTAMPED:
			lgw_reg_w(LGW_TX_TRIG_DELAYED, 1);
			break;
			
		case ON_GPS:
			lgw_reg_w(LGW_TX_TRIG_GPS, 1);
			break;
			
		default:
			DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d IN SWITCH STATEMENT\n", pkt_data.tx_mode);
			return LGW_HAL_ERROR;
	}
	
	return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_status(uint8_t select, uint8_t *code) {
	int32_t read_value;
	
	/* check input variables */
	CHECK_NULL(code);
	
	if (select == TX_STATUS) {
		lgw_reg_r(LGW_TX_STATUS, &read_value);
		if (lgw_is_started == false) {
			*code = TX_OFF;
		} else if ((read_value & 0x10) == 0) { /* bit 4 @1: TX programmed */
			*code = TX_FREE;
		} else if ((read_value & 0x60) != 0) { /* bit 5 or 6 @1: TX sequence */
			*code = TX_EMITTING;
		} else {
			*code = TX_SCHEDULED;
		}
		return LGW_HAL_SUCCESS;
		
	} else if (select == RX_STATUS) {
		*code = RX_STATUS_UNKNOWN; /* todo */
		return LGW_HAL_SUCCESS;
		
	} else {
		DEBUG_MSG("ERROR: SELECTION INVALID, NO STATUS TO RETURN\n");
		return LGW_HAL_ERROR;
	}
	
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_abort_tx(void) {
	int i;
	
	i = lgw_reg_w(LGW_TX_TRIG_ALL, 0);
	
	if (i == LGW_REG_SUCCESS) return LGW_HAL_SUCCESS;
	else return LGW_HAL_ERROR;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_get_trigcnt(uint32_t* trig_cnt_us) {
	int i;
	int32_t val;
	
	i = lgw_reg_r(LGW_TIMESTAMP, &val);
	if (i == LGW_REG_SUCCESS) {
		*trig_cnt_us = (uint32_t)val;
		return LGW_HAL_SUCCESS;
	} else {
		return LGW_HAL_ERROR;
	}
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

const char* lgw_version_info() {
	return lgw_version_string;
}


/* --- EOF ------------------------------------------------------------------ */
