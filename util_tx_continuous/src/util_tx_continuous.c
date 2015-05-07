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

#include <stdint.h>		/* C99 types */
#include <stdbool.h>	/* bool type */
#include <stdio.h>		/* printf fprintf sprintf fopen fputs */
#include <string.h>		/* memset */
#include <signal.h>		/* sigaction */
#include <unistd.h>		/* getopt access */
#include <stdlib.h>		/* exit codes */
#include <getopt.h>		/* getopt_long */

#include "loragw_hal.h"
#include "loragw_reg.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- MACROS & CONSTANTS --------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#define SX125x_VERSION          0x21
#define SX125x_32MHz_FRAC		15625	/* irreductible fraction for PLL register caculation */
#define SX125x_TX_DAC_CLK_SEL	1	/* 0:int, 1:ext */
#define SX125x_TX_DAC_GAIN		2	/* 3:0, 2:-3, 1:-6, 0:-9 dBFS (default 2) */
#define SX125x_TX_MIX_GAIN		14	/* -38 + 2*TxMixGain dB (default 14) */
#define SX125x_TX_PLL_BW		3	/* 0:75, 1:150, 2:225, 3:300 kHz (default 3) */
#define SX125x_TX_ANA_BW		0	/* 17.5 / 2*(41-TxAnaBw) MHz (default 0) */
#define SX125x_TX_DAC_BW		5	/* 24 + 8*TxDacBw Nb FIR taps (default 2) */
#define SX125x_XOSC_GM_STARTUP	13	/* (default 13) */
#define SX125x_XOSC_DISABLE		2	/* Disable of Xtal Oscillator blocks bit0:regulator, bit1:core(gm), bit2:amplifier */

#define DEFAULT_FREQ_HZ     868e6
#define DEFAULT_RADIO         0
#define DEFAULT_DIGITAL_GAIN  0
#define DEFAULT_DAC_GAIN      3
#define DEFAULT_MIXER_GAIN   14
#define DEFAULT_PA_VOLTAGE  1.5
#define DEFAULT_MODULATION "LORA"
#define DEFAULT_SF            7
#define DEFAULT_BW_KHZ      125
#define DEFAULT_BR_KBPS      50
#define DEFAULT_FDEV_KHZ     25
#define DEFAULT_BT            2

/* -------------------------------------------------------------------------- */
/* --- GLOBAL VARIABLES ----------------------------------------------------- */

/* Signal handling variables */
static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

#include "cal_tx_fw.var"
#include "util_fw.var"

/* -------------------------------------------------------------------------- */
/* --- SUBFUNCTIONS DECLARATION --------------------------------------------- */

static void sig_handler(int sigio);

void sx125x_write(uint8_t channel, uint8_t addr, uint8_t data); /* defined in loragw_hal.c */

uint8_t sx125x_read(uint8_t channel, uint8_t addr); /* defined in loragw_hal.c */

int load_firmware(uint8_t target, uint8_t *firmware, uint16_t size);  /* defined in loragw_hal.c */

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv)
{
	static struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */
	
	int i; /* loop and temporary variables */
	int x; /* return code for HAL functions */
	
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
        {0, 0, 0, 0}
    };
	unsigned int arg_u;
	float arg_f;
	char arg_s[64];
	
	/* Application parameters */
	uint32_t freq_hz = DEFAULT_FREQ_HZ;	
	bool radio_select = DEFAULT_RADIO;
	uint8_t g_dig = DEFAULT_DIGITAL_GAIN;
	uint8_t g_dac = DEFAULT_DAC_GAIN;
	uint8_t g_mix = DEFAULT_MIXER_GAIN;
	uint8_t g_pa = DEFAULT_PA_VOLTAGE;
	char mod[64] = DEFAULT_MODULATION;
	uint8_t sf = DEFAULT_SF;
	unsigned int bw_khz = DEFAULT_BW_KHZ;
	float br_kbps = DEFAULT_BR_KBPS;
	uint8_t fdev_khz = DEFAULT_FDEV_KHZ;
	uint8_t bt = DEFAULT_BT;
	
	uint8_t buff[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint32_t part_int; /* integer part for PLL register value calculation */
	uint32_t part_frac; /* fractional part for PLL register value calculation */
	uint16_t br;
	int32_t offset_i, offset_q;
	bool is_sx1255;
	int32_t cal_status, cal_status_expected;
	int32_t tx_status;
	
	/* Parse command line options */
	while( (i = getopt_long (argc, argv, "hud::f:r:", long_options, &option_index)) != -1 )
	{
		switch( i )
		{
			case 'h':
				printf( "~~~ Library version string~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n" );
				printf( " %s\n", lgw_version_info( ) );
				printf( "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n" );
				printf( " -f     <float>  Tx RF frequency in MHz [800:1000]\n");
				printf( " -r     <bool>   Radio select, 0:A 1:B\n");
				printf( " --dig  <uint>   Digital gain trim, [0:3]\n" );
				printf( "                   0:1, 1:7/8, 2:3/4, 3:1/2\n" );
				printf( " --mix  <uint>   SX1257 Tx mixer gain trim, [0:15]\n" );
				printf( "                   15 corresponds to maximum gain, 1 LSB corresponds to 2dB step\n" );
				printf( " --pa   <uint>   PA gain trim, [0:3]\n" );
				printf( " --mod  <char>   Modulation type ['LORA','FSK','CW']\n" );
				printf( " --sf   <uint>   LoRa Spreading Factor, [7:12]\n" );
				printf( " --bw   <uint>   LoRa bandwidth in kHz, [125,250,500]\n" );
				printf( " --br   <float>  FSK bitrate in kbps, [0.5:250]\n" );
				printf( " --fdev <uint>   FSK frequency deviation in kHz, [1:250]\n" );
				printf( " --bt   <uint>   FSK gaussian filter BT trim, [0:3]\n" );
				printf( "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n" );
				return EXIT_SUCCESS;
				break;
			
			case 0:
				if( strcmp(long_options[option_index].name,"dig") == 0 )
				{
					i = sscanf(optarg, "%u", &arg_u);
					if( (i != 1) || (arg_u > 3) )
					{
						printf( "ERROR: argument parsing of --dig argument. Use -h to print help\n" );
						return EXIT_FAILURE;
					}
					else
					{
						g_dig = (uint8_t)arg_u;
					}
				}
				else if( strcmp(long_options[option_index].name,"dac") == 0 )
				{
					i = sscanf(optarg, "%u", &arg_u);
					if( (i != 1) || (arg_u > 3) )
					{
						printf( "ERROR: argument parsing of --dac argument. Use -h to print help\n" );
						return EXIT_FAILURE;
					}
					else
					{
						g_dac = (uint8_t)arg_u;
					}
				}
				else if( strcmp(long_options[option_index].name,"mix") == 0 )
				{
					i = sscanf(optarg, "%u", &arg_u);
					if( (i != 1) || (arg_u > 15) )
					{
						printf( "ERROR: argument parsing of --mix argument. Use -h to print help\n" );
						return EXIT_FAILURE;
					}
					else
					{
						g_mix = (uint8_t)arg_u;
					}
				}
				else if( strcmp(long_options[option_index].name,"pa") == 0 )
				{
					i = sscanf(optarg, "%u", &arg_u);
					if( (i != 1) || (arg_u > 3) )
					{
						printf( "ERROR: argument parsing of --pa argument. Use -h to print help\n" );
						return EXIT_FAILURE;
					}
					else
					{
						g_pa = arg_u;
					}
				}
				else if( strcmp(long_options[option_index].name,"mod") == 0 )
				{
					i = sscanf(optarg, "%s", arg_s);
					if( (i != 1) || ((strcmp(arg_s,"LORA") != 0) && (strcmp(arg_s,"FSK") != 0)  && (strcmp(arg_s,"CW") != 0)) )
					{
						printf( "ERROR: argument parsing of --mod argument. Use -h to print help\n" );
						return EXIT_FAILURE;
					}
					else
					{
						sprintf(mod, "%s", arg_s);
					}
				}
				else if( strcmp(long_options[option_index].name,"sf") == 0 )
				{
					i = sscanf(optarg, "%u", &arg_u);
					if( (i != 1) || (arg_u < 7) || (arg_u > 12) )
					{
						printf( "ERROR: argument parsing of --sf argument. Use -h to print help\n" );
						return EXIT_FAILURE;
					}
					else
					{
						sf = (uint8_t)arg_u;
					}
				}
				else if( strcmp(long_options[option_index].name,"bw") == 0 )
				{
					i = sscanf(optarg, "%u", &arg_u);
					if( (i != 1) || ((arg_u != 125) && (arg_u != 250) && (arg_u != 500)) )
					{
						printf( "ERROR: argument parsing of --bw argument. Use -h to print help\n" );
						return EXIT_FAILURE;
					}
					else
					{
						bw_khz = arg_u;
					}
				}
				else if( strcmp(long_options[option_index].name,"br") == 0 )
				{
					i = sscanf(optarg, "%f", &arg_f);
					if( (i != 1) || (arg_f < 0.5) || (arg_f > 250) )
					{
						printf( "ERROR: argument parsing of --br argument. Use -h to print help\n" );
						return EXIT_FAILURE;
					}
					else
					{
						br_kbps = arg_f;
					}
				}
				else if( strcmp(long_options[option_index].name,"fdev") == 0 )
				{
					i = sscanf(optarg, "%u", &arg_u);
					if( (i != 1) || (arg_u < 1) || (arg_u > 250) )
					{
						printf( "ERROR: argument parsing of --fdev argument. Use -h to print help\n" );
						return EXIT_FAILURE;
					}
					else
					{
						fdev_khz = (uint8_t)arg_u;
					}
				}
				else if( strcmp(long_options[option_index].name,"bt") == 0 )
				{
					i = sscanf(optarg, "%u", &arg_u);
					if( (i != 1) || (arg_u > 3) )
					{
						printf( "ERROR: argument parsing of --bt argument. Use -h to print help\n" );
						return EXIT_FAILURE;
					}
					else
					{
						bt = (uint8_t)arg_u;
					}
				}
				else {
					printf( "ERROR: argument parsing options. Use -h to print help\n" );
					return EXIT_FAILURE;
				}
				break;
		
		case 'f':
			i = sscanf(optarg, "%f", &arg_f);
			if( (i != 1) || (arg_f < 1) )
			{
				printf( "ERROR: argument parsing of -f argument. Use -h to print help\n" );
				return EXIT_FAILURE;
			}
			else
			{
				freq_hz = (uint32_t)((arg_f * 1e6) + 0.5);
			}
			break;
			
		case 'r':
			i = sscanf(optarg, "%u", &arg_u);
			if( (i != 1) || (arg_u > 1) )
			{
				printf( "ERROR: argument parsing of -r argument. Use -h to print help\n" );
				return EXIT_FAILURE;
			}
			else
			{
				radio_select = (bool)arg_u;
			}
			break;
			
		default:
			printf( "ERROR: argument parsing options. Use -h to print help\n" );
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
	
	/* Decide if it is SX1257 or 55 depending on required freq*/
	if (freq_hz > 700e6) {
		is_sx1255 = false;
	} else {
		is_sx1255 = true;
	} 
	
	/* Connect to concentrator */
	x = lgw_connect();
	if (x == LGW_REG_ERROR) {
		printf("ERROR: FAIL TO CONNECT BOARD\n");
		return LGW_HAL_ERROR;
	}
	
	/* reset the registers (also shuts the radios down) */
	lgw_soft_reset();
	wait_ms(100);
	
	/* switch on and reset the radios (also starts the 32 MHz XTAL) */
	lgw_reg_w(LGW_RADIO_A_EN,1);
	lgw_reg_w(LGW_RADIO_B_EN,1);
	wait_ms(500);
	lgw_reg_w(LGW_RADIO_RST,1);
	wait_ms(1);
	lgw_reg_w(LGW_RADIO_RST,0);
	
	/* ~~~~~~ Setup SX1257 ~~~~~~ */
	
	/* Check version */
	if( sx125x_read(radio_select, 0x07) != SX125x_VERSION )
	{
		printf( "ERROR: Bad SX1257 version\n" );
		return EXIT_FAILURE;
	}
	
	sx125x_write(0, 0x10, SX125x_TX_DAC_CLK_SEL + 2);
	sx125x_write(1, 0x10, SX125x_TX_DAC_CLK_SEL + 2);
	if (is_sx1255) {
		sx125x_write(0, 0x28, SX125x_XOSC_GM_STARTUP + SX125x_XOSC_DISABLE*16);
		sx125x_write(1, 0x28, SX125x_XOSC_GM_STARTUP + SX125x_XOSC_DISABLE*16);
	}
	else {
		sx125x_write(0, 0x26, SX125x_XOSC_GM_STARTUP + SX125x_XOSC_DISABLE*16);
		sx125x_write(1, 0x26, SX125x_XOSC_GM_STARTUP + SX125x_XOSC_DISABLE*16);
	}
	
	/* Tx gain and BW */
	sx125x_write( radio_select, 0x08, g_mix + g_dac*16);
	if( bw_khz > 250 )
	{
		sx125x_write( radio_select, 0x0A, 12 + SX125x_TX_PLL_BW*32 ); /* ANA FILT BW */
		sx125x_write( radio_select, 0x0B, 4 ); /* DIG FILT BW */
	}
	else
	{
		sx125x_write( radio_select, 0x0A, 0 + SX125x_TX_PLL_BW*32 ); /* ANA FILT BW */
		sx125x_write( radio_select, 0x0B, 5 ); /* DIG FILT BW */
	}
	
	/* set TX PLL frequency */
	if (is_sx1255) {
		part_int = freq_hz / (SX125x_32MHz_FRAC << 7); /* integer part, gives the MSB */
		part_frac = ((freq_hz % (SX125x_32MHz_FRAC << 7)) << 9) / SX125x_32MHz_FRAC; /* fractional part, gives middle part and LSB */
	} else {
		part_int = freq_hz / (SX125x_32MHz_FRAC << 8); /* integer part, gives the MSB */
		part_frac = ((freq_hz % (SX125x_32MHz_FRAC << 8)) << 8) / SX125x_32MHz_FRAC; /* fractional part, gives middle part and LSB */
	}
	sx125x_write(radio_select, 0x04,0xFF & part_int); /* Most Significant Byte */
	sx125x_write(radio_select, 0x05,0xFF & (part_frac >> 8)); /* middle byte */
	sx125x_write(radio_select, 0x06,0xFF & part_frac); /* Least Significant Byte */
	
	/* Lock PLL and enable Tx */
	sx125x_write(radio_select, 0x00, 1);
	wait_ms(1);
	sx125x_write(radio_select, 0x00, 13);
	wait_ms(10);
	if ((sx125x_read(radio_select, 0x11) & 0x01) == 0) {
		printf("ERROR: SX125x Tx PLL did not lock\n");
		return -1;
	}
	
	/* ~~~~~~~~~~~~~~~~~~ */
	
	/* Minimum SX1301 setup */
	lgw_reg_w(LGW_GLOBAL_EN, 1);
	lgw_reg_w(LGW_TX_GAIN, g_dig);
	lgw_reg_w(LGW_PA_GAIN, g_pa);
	lgw_reg_w(LGW_TX_MODE, 1); // Tx continuous
	lgw_reg_w(LGW_FSK_TX_GAUSSIAN_SELECT_BT, bt);
	lgw_reg_w(LGW_GPIO_MODE,24); /* Set GPIO 3 and 4 in output to control potential Tx filter FPGA */
	lgw_reg_w(LGW_GPIO_SELECT_OUTPUT,8); /* Control GPIO with register */
	lgw_reg_w(LGW_GPIO_PIN_REG_OUT,16); /* Enable Tx */ 
	
	if( strcmp( mod, "CW" ) == 0 )
	{
		/* Enable signal generator with DC */
		lgw_reg_w( LGW_SIG_GEN_FREQ, 0 );
		lgw_reg_w( LGW_SIG_GEN_EN, 1 );
	}
	else
	{
		/* Tx DC offset calibration */
		x = load_firmware( 1, cal_tx_fw, 8192 ); /* Load firmware */
		lgw_reg_w(LGW_RADIO_SELECT, (is_sx1255 << 5) | ((radio_select+1) << 2) ); /* Configure calibration */
		lgw_reg_w(LGW_FORCE_HOST_RADIO_CTRL, 0); /* Give to MCU control of radio */
		lgw_reg_w(LGW_MCU_RST_1, 0);
		lgw_reg_w(LGW_PAGE_REG, 3); /* Calibration start condition */
		lgw_reg_w(LGW_EMERGENCY_FORCE_HOST_CTRL, 0); /* Give to MCU control of registers */
		wait_ms(300); /* Wait enough until cal is done */
		lgw_reg_w(LGW_EMERGENCY_FORCE_HOST_CTRL, 1); /* Get back control of registers */
		
		/* Get calibration results */
		lgw_reg_r( LGW_MCU_AGC_STATUS, &cal_status );
		cal_status_expected = (1<<7) | ((radio_select+1)<<5) | (1<<2) | (1<<1) | 1;
		if (cal_status != cal_status_expected)
		{
			lgw_reg_w( LGW_TX_OFFSET_I, 0 );
			lgw_reg_w( LGW_TX_OFFSET_Q, 0 );
			printf("WARNING: Tx DC Calibration failed\n");
		}
		
		/* TX metadata buffer */
		if( strcmp( mod, "LORA" ) == 0 )
		{
			buff[9] = sf;
			switch( bw_khz )
			{
				case 125: buff[11] = 0; break;
				case 250: buff[11] = 1; break;
				case 500: buff[11] = 2; break;
			}
			buff[11] |= 0x04; /* set 'implicit header' bit */
		}
		else if( strcmp( mod, "FSK" ) == 0 )
		{
			br = (uint16_t)(32e3 / br_kbps);
			buff[7] = 1 << 4;
			buff[9] = fdev_khz; /* FSK frequency deviation */
			buff[14] = 0xFF & (br >> 8); /* FSK bitrate */
			buff[15] = 0xFF & br;
		}
		buff[10] = 1; /* payload size */
		buff[12] = 0xFF & (65535 >> 8); /* preamble length */
		buff[13] = 0xFF & 65535;
			
		/* Write metadata into buffer */
		lgw_reg_w( LGW_TX_DATA_BUF_ADDR, 0 );
		lgw_reg_wb( LGW_TX_DATA_BUF_DATA, buff, 16 );
		
		/* Trig Tx and check modulation is running */
		lgw_reg_w( LGW_TX_TRIG_IMMEDIATE, 1 );
		wait_ms(5);
		lgw_reg_r( LGW_TX_STATUS, &tx_status );
		if( tx_status != 80 )
		{
			printf( "ERROR: modulation not running\n" );
			return EXIT_FAILURE;
		}
	}
	
	/* Load firmware to select radio */
	x = load_firmware( 1, util_fw, 8192 );
	lgw_reg_w(LGW_MCU_RST_1, 0);
	lgw_reg_w(LGW_RADIO_SELECT, radio_select);
	
	/* Set RF switch in Tx */
	if( radio_select )
	{
		lgw_reg_w(LGW_PA_B_EN,1);
	} else
	{
		lgw_reg_w(LGW_PA_A_EN,1);
	}

	/* Enable Tx and Tx narrowband filter for LORA 125kHz BW */
	if( ( strcmp( mod, "LORA" ) == 0 ) && ( bw_khz == 125 ) )
	{
		lgw_reg_w(LGW_GPIO_PIN_REG_OUT,24);
	}

	/* Recap all settings */
	printf( "SX1301 library version: %s\n", lgw_version_info( ) );
	if( strcmp( mod, "LORA" ) == 0 )
	{
		printf( "Modulation: LORA SF:%d BW:%d kHz\n", sf, bw_khz );
	}
	else if( strcmp( mod, "FSK" ) == 0 )
	{
		printf( "Modulation: FSK BR:%3.3f kbps FDEV:%d kHz BT:%d\n", br_kbps, fdev_khz, bt );
	}
	else if( strcmp( mod, "CW" ) == 0 )
	{
		printf( "Modulation: CW\n" );
	}
	if( radio_select )
	{
		printf("Radio: B\n");
	}
	else
	{
		printf("Radio: A\n");
	}
	printf( "Frequency: %4.3f MHz\n", freq_hz/1e6 );
	printf( "Expected chip: SX125%d\n", 7-is_sx1255*2 );
	printf( "Gains: Digital:%d DAC:%d Mixer:%d PA:%d\n", g_dig, g_dac, g_mix, g_pa );
	if( strcmp( mod, "CW" ) != 0 )
	{
		lgw_reg_r( LGW_TX_OFFSET_I, &offset_i );
		lgw_reg_r( LGW_TX_OFFSET_Q, &offset_q );
		printf( "Calibrated DC offsets: I:%d Q:%d\n", offset_i, offset_q );
	}
	
	/* waiting for user input */
	while ((quit_sig != 1) && (exit_sig != 1))
	{
		wait_ms(100);
	}
	
	/* clean up before leaving */
	lgw_stop();
	
	return 0;
}

/* -------------------------------------------------------------------------- */
/* --- SUBFUNCTIONS DEFINITION ---------------------------------------------- */

static void sig_handler( int sigio )
{
	if( sigio == SIGQUIT )
	{
		quit_sig = 1;
	}
	else if( (sigio == SIGINT) || (sigio == SIGTERM) )
	{
		exit_sig = 1;
	}
}

/* --- EOF ------------------------------------------------------------------ */
