/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
	Host specific functions to address the GPIO. mainly the Reset pin

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Miguel Luis
*/


#ifndef _LORAGW_GPIO_H
#define _LORAGW_GPIO_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>		/* C99 types*/

#include "config.h"	/* library configuration options (dynamically generated) */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

#define LGW_GPIO_SUCCESS	 0
#define LGW_GPIO_ERROR	-1

#define LGW_GPIO_IN	0
#define LGW_GPIO_OUT	1

#define LGW_GPIO_LOW	0
#define LGW_GPIO_HIGH	1

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

/**
@brief Reserves the given GPIO pin
@param pin pin ID to be reserved
@return status of operation (LGW_GPIO_SUCCESS/LGW_GPIO_ERROR)
*/
int lgw_gpio_export(int pin);

/**
@brief Releases the given GPIO pin reservation
@param pin pin ID to be released
@return status of operation (LGW_GPIO_SUCCESS/LGW_GPIO_ERROR)
*/
int lgw_gpio_unexport(int pin);

/**
@brief Sets the GPIO pin to the given direction
@param pin pin ID to be changed
@param dir new direction for the pin(LGW_GPIO_IN/LGW_GPIO_OUT)
@return status of operation (LGW_GPIO_SUCCESS/LGW_GPIO_ERROR)
*/
int lgw_gpio_direction(int pin, int dir);

/**
@brief Reads the given GPIO pin
@param pin pin ID to be read
@return value of the given pin ID (LGW_GPIO_LOW/LGW_GPIO_HIGH/LGW_GPIO_ERROR)
*/
int lgw_gpio_read(int pin);

/**
@brief Writes the given GPIO pin with value
@param pin pin ID to be changed
@param value new value for the pin(LGW_GPIO_LOW/LGW_GPIO_HIDH)
@return status of operation (LGW_GPIO_SUCCESS/LGW_GPIO_ERROR)
*/
int lgw_gpio_write(int pin, int value);

#endif

/* --- EOF ------------------------------------------------------------------ */
