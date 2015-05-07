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


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>		/* C99 types */
#include <stdio.h>		/* printf fprintf */
#include <stdlib.h>		/* malloc free */
#include <unistd.h>		/* lseek, close */
#include <fcntl.h>		/* open */
#include <string.h>		/* memset */

#include "loragw_aux.h"
#include "loragw_gpio.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_GPIO == 1
	#define DEBUG_MSG(str)				fprintf(stderr, str)
	#define DEBUG_PRINTF(fmt, args...)	fprintf(stderr,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
	#define CHECK_NULL(a)				if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_GPIO_ERROR;}
#else
	#define DEBUG_MSG(str)
	#define DEBUG_PRINTF(fmt, args...)
	#define CHECK_NULL(a)				if(a==NULL){return LGW_GPIO_ERROR;}
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */
#define BUFFER_MAX 255

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

/* GPIO export */
int lgw_gpio_export(int pin) {
    char buffer[BUFFER_MAX];
    ssize_t bytes_written;
    int fd;

    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd == LGW_GPIO_ERROR) {
        DEBUG_MSG("Failed to open export for writing!\n");
        return(LGW_GPIO_ERROR);
    }

    bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
    if (write(fd, buffer, bytes_written) != bytes_written) {
	DEBUG_MSG("Failed to export GPIO: write to sysfs error\n");
	close(fd);
	return(LGW_GPIO_ERROR);
    }

    close(fd);
    wait_ms( 100 );
    return(LGW_GPIO_SUCCESS);
}

/* GPIO unexport */
int lgw_gpio_unexport(int pin) {
    char buffer[BUFFER_MAX];
    ssize_t bytes_written;
    int fd;

    fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd == LGW_GPIO_ERROR) {
        DEBUG_MSG("Failed to open unexport for writing!\n");
        return(LGW_GPIO_ERROR);
    }

    bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
    if (write(fd, buffer, bytes_written) != bytes_written) {
	DEBUG_MSG("Failed to unexport GPIO: write to sysfs error\n");
	close(fd);
	return(LGW_GPIO_ERROR);
    }

    close(fd);
    return(LGW_GPIO_SUCCESS);
}

/* GPIO direction */
int lgw_gpio_direction(int pin, int dir) {
    static const char s_directions_str[]  = "in\0out";

    char path[BUFFER_MAX];
    int fd;

    snprintf(path, BUFFER_MAX, "/sys/class/gpio/gpio%d/direction", pin);
    fd = open(path, O_WRONLY);
    if (fd == LGW_GPIO_ERROR) {
        DEBUG_MSG("Failed to open gpio direction for writing!\n");
        return(LGW_GPIO_ERROR);
    }

    if (write(fd, &s_directions_str[LGW_GPIO_IN == dir ? 0 : 3], LGW_GPIO_IN == dir ? 2 : 3) == LGW_GPIO_ERROR) {
        DEBUG_MSG("Failed to set direction!\n");
        return(LGW_GPIO_ERROR);
    }

    close(fd);
    return(LGW_GPIO_SUCCESS);
}

/* GPIO read */
int lgw_gpio_read(int pin) {
    char path[BUFFER_MAX];
    char value_str[3];
    int fd;

    snprintf(path, BUFFER_MAX, "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_RDONLY);
    if (fd == LGW_GPIO_ERROR) {
        DEBUG_MSG("Failed to open gpio value for reading!\n");
        return(LGW_GPIO_ERROR);
    }

    if (read(fd, value_str, 3) == LGW_GPIO_ERROR) {
        DEBUG_MSG("Failed to read value!\n");
        return(LGW_GPIO_ERROR);
    }

    close(fd);
    return(atoi(value_str));
}

/* GPIO write */
int lgw_gpio_write(int pin, int value) {
    static const char s_values_str[] = "01";

    char path[BUFFER_MAX];
    int fd;

    snprintf(path, BUFFER_MAX, "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_WRONLY);
    if (fd == LGW_GPIO_ERROR) {
        DEBUG_MSG("Failed to open gpio value for writing!\n");
        return(LGW_GPIO_ERROR);
    }

    if (write(fd, &s_values_str[LGW_GPIO_LOW == value ? 0 : 1], 1) != 1) {
        DEBUG_MSG("Failed to write value!\n");
        return(LGW_GPIO_ERROR);
    }

    close(fd);
    return(LGW_GPIO_SUCCESS);
}


/* --- EOF ------------------------------------------------------------------ */
