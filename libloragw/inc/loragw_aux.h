/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
	LoRa concentrator HAL common auxiliary functions

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Sylvain Miermont
*/


#ifndef _LORAGW_AUX_H
#define _LORAGW_AUX_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include "config.h"	/* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

/**
@brief Wait for a certain time (millisecond accuracy)
@param t number of milliseconds to wait.
*/
void wait_ms(unsigned long t);

#endif

/* --- EOF ------------------------------------------------------------------ */
