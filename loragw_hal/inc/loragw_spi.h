/*
 / _____)             _              | |    
( (____  _____ ____ _| |_ _____  ____| |__  
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    ©2013 Semtech-Cycleo

Description:
    Host specific functions to address the LoRa™ gateway registers through a
    SPI interface.
    Single-byte read/write and burst read/write.
    Does not handle pagination.
    Could be used with multiple SPI ports in parallel (explicit file descriptor)
*/


#ifndef _LORAGW_SPI_H
#define _LORAGW_SPI_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types*/

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

#define LGW_SPI_SUCCESS  0
#define LGW_SPI_ERROR   -1
#define LGW_BURST_CHUNK  1024

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

/**
@brief Lora gateway SPI setup (configure I/O and peripherals)
@param spi_device pointer to SPI file descriptor to be written
@return status of register operation (LGW_SPI_SUCCESS/LGW_SPI_ERROR)
*/

int lgw_spi_open(int *spi_device);

/**
@brief Lora gateway SPI close
@param spi_device SPI file descriptor of the port to close
@return status of register operation (LGW_SPI_SUCCESS/LGW_SPI_ERROR)
*/

int lgw_spi_close(int spi_device);

/**
@brief Lora gateway SPI single-byte write
@param spi_device SPI file descriptor of the target port
@param address 7-bit register address
@param data data byte to write
@return status of register operation (LGW_SPI_SUCCESS/LGW_SPI_ERROR)
*/
int lgw_spi_w(int spi_device, uint8_t address, uint8_t data);

/**
@brief Lora gateway SPI single-byte read
@param spi_device SPI file descriptor of the target port
@param address 7-bit register address
@param data data byte to write
@return status of register operation (LGW_SPI_SUCCESS/LGW_SPI_ERROR)
*/
int lgw_spi_r(int spi_device, uint8_t address, uint8_t *data);

/**
@brief Lora gateway SPI burst (multiple-byte) write
@param spi_device SPI file descriptor of the target port
@param address 7-bit register address
@param data pointer to byte array that will be sent to the Lora gateway
@param size size of the transfer, in byte(s)
@return status of register operation (LGW_SPI_SUCCESS/LGW_SPI_ERROR)
*/
int lgw_spi_wb(int spi_device, uint8_t address, uint8_t *data, uint16_t size);

/**
@brief Lora gateway SPI burst (multiple-byte) read
@param spi_device SPI file descriptor of the target port
@param address 7-bit register address
@param data pointer to byte array that will be written from the Lora gateway
@param size size of the transfer, in byte(s)
@return status of register operation (LGW_SPI_SUCCESS/LGW_SPI_ERROR)
*/
int lgw_spi_rb(int spi_device, uint8_t address, uint8_t *data, uint16_t size);

#endif

/* --- EOF ------------------------------------------------------------------ */
