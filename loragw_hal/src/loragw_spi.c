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


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdio.h>      /* printf fprintf */
#include <unistd.h>     /* lseek, close */
#include <fcntl.h>      /* open */
#include <string.h>     /* memset */

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "loragw_spi.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#ifdef DEBUG
    #define DEBUG_MSG(str)              fprintf(stderr, str)
    #define DEBUG_PRINTF(fmt, args...)  fprintf(stderr,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)               if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_SPI_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)               if(a==NULL){return LGW_SPI_ERROR;}
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define READ_ACCESS     0x00
#define WRITE_ACCESS    0x80
#define SPI_SPEED       8000000
#define SPI_DEV_PATH    "/dev/spidev0.0"

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

/* SPI initialization and configuration */
int lgw_spi_open(int *spi_device) {
    int i,j,k,l;
    
    CHECK_NULL(spi_device);
    
    /* open SPI device */
    i = open(SPI_DEV_PATH, O_RDWR);
    if (i < 0) {
        DEBUG_MSG("SPI port fail to open\n");
        return LGW_SPI_ERROR;
    }
    
    /* setting SPI mode to 'mode 0' */
    j = SPI_MODE_0;
    k = ioctl(i, SPI_IOC_WR_MODE, &j);
    l = ioctl(i, SPI_IOC_RD_MODE, &j);
    if ((k < 0) || (l < 0)) {
        DEBUG_MSG("ERROR: SPI PORT FAIL TO SET IN MODE 0\n");
        close(i);
        return LGW_SPI_ERROR;
    }
    
    /* setting SPI max clk (in Hz) */
    j = SPI_SPEED;
    k = ioctl(i, SPI_IOC_WR_MAX_SPEED_HZ, &j);
    l = ioctl(i, SPI_IOC_RD_MAX_SPEED_HZ, &j);
    if ((k < 0) || (l < 0)) {
        DEBUG_MSG("ERROR: SPI PORT FAIL TO SET MAX SPEED\n");
        close(i);
        return LGW_SPI_ERROR;
    }
    
    /* setting SPI to MSB first */
    j = 0;
    k = ioctl(i, SPI_IOC_WR_LSB_FIRST, &j);
    l = ioctl(i, SPI_IOC_RD_LSB_FIRST, &j);
    if ((k < 0) || (l < 0)) {
        DEBUG_MSG("ERROR: SPI PORT FAIL TO SET MSB FIRST\n");
        close(i);
        return LGW_SPI_ERROR;
    }
    
    /* setting SPI to 8 bits per word */
    j = 0; 
    k = ioctl(i, SPI_IOC_WR_BITS_PER_WORD, &j);
    l = ioctl(i, SPI_IOC_RD_BITS_PER_WORD, &j);
    if ((k < 0) || (l < 0)) {
        DEBUG_MSG("ERROR: SPI PORT FAIL TO SET 8 BITS-PER-WORD\n");
        close(i);
        return LGW_SPI_ERROR;
    }
    DEBUG_MSG("Note: SPI port opened and configured ok\n");
    *spi_device = i;
    return LGW_SPI_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* SPI release */
int lgw_spi_close(int spi_device) {
    int i;
    
    i = close(spi_device);
    if (i < 0) {
        DEBUG_MSG("ERROR: SPI PORT FAILED TO CLOSE\n");
        return LGW_SPI_ERROR;
    } else {
        DEBUG_MSG("Note: SPI port closed\n");
        return LGW_SPI_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Simple write */
int lgw_spi_w(int spi_device, uint8_t address, uint8_t data) {
    uint8_t outbuf[2];
    struct spi_ioc_transfer k;
    int i;
    
    outbuf[0] = WRITE_ACCESS | (address & 0x7F);
    outbuf[1] = data;
    
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long) outbuf;
    k.len = ARRAY_SIZE(outbuf);
    k.speed_hz = SPI_SPEED;
    k.cs_change = 1;
    k.bits_per_word = 8;
    
    i = ioctl(spi_device, SPI_IOC_MESSAGE(1), &k);
    
    if (i != 2) {
        DEBUG_MSG("ERROR: SPI WRITE FAILURE\n");
        return LGW_SPI_ERROR;
    } else {
        DEBUG_MSG("Note: SPI write success\n");
        return LGW_SPI_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Simple read */
int lgw_spi_r(int spi_device, uint8_t address, uint8_t *data) {
    uint8_t outbuf[2];
    uint8_t inbuf[ARRAY_SIZE(outbuf)];
    struct spi_ioc_transfer k;
    int i;
    
    CHECK_NULL(data);
    
    outbuf[0] = READ_ACCESS | (address & 0x7F);
    outbuf[1] = 0x00;
    
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long) outbuf;
    k.rx_buf = (unsigned long) inbuf;
    k.len = ARRAY_SIZE(outbuf);
    k.cs_change = 1;
    
    i = ioctl(spi_device, SPI_IOC_MESSAGE(1), &k);
    
    if (i != 2) {
        DEBUG_MSG("ERROR: SPI READ FAILURE\n");
        return LGW_SPI_ERROR;
    } else {
        DEBUG_MSG("Note: SPI read success\n");
        *data = inbuf[1];
        return LGW_SPI_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) write */
int lgw_spi_wb(int spi_device, uint8_t address, uint8_t *data, uint16_t size) {
    uint8_t command;
    struct spi_ioc_transfer k[2];
    int byte_to_trans;
    int chunk_size;
    int offset = 0;
    int byte_transfered = 0;
    
    /* check input parameters */
    CHECK_NULL(data);
    if (size == 0) {
        DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
        return LGW_SPI_ERROR;
    }
    if (address > 0x7F) {
        DEBUG_MSG("ERROR: ADDRESS OUT OF SPI RANGE\n");
        return LGW_SPI_ERROR;
    }
    
    memset(&k, 0, sizeof(k)); /* clear k */
    command = WRITE_ACCESS | (address & 0x7F);
    k[0].tx_buf = (unsigned long) &command;
    k[0].len = 1;
    k[0].cs_change = 0;
    k[1].cs_change = 1;
    
    byte_to_trans = size;
    while (byte_to_trans > 0) {
        chunk_size = (byte_to_trans < LGW_BURST_CHUNK) ? byte_to_trans : LGW_BURST_CHUNK;
        k[1].tx_buf = (unsigned long)(data + offset);
        k[1].len = chunk_size;
        
        byte_transfered += (ioctl(spi_device, SPI_IOC_MESSAGE(2), &k) - 1 );
        DEBUG_PRINTF("BURST WRITE: to trans %d # chunk %d # transferred %d \n", byte_to_trans, chunk_size, byte_transfered);
        
        byte_to_trans -= chunk_size;
        offset += chunk_size;
    } 
    
    if (byte_transfered != size) {
        DEBUG_MSG("ERROR: SPI BURST WRITE FAILURE\n");
        return LGW_SPI_ERROR;
    } else {
        DEBUG_MSG("Note: SPI burst write success\n");
        return LGW_SPI_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) read */
int lgw_spi_rb(int spi_device, uint8_t address, uint8_t *data, uint16_t size) {
    uint8_t command;
    struct spi_ioc_transfer k[2];
    int byte_to_trans;
    int chunk_size;
    int offset = 0;
    int byte_transfered = 0;
    
    /* check input parameters */
    CHECK_NULL(data);
    if (size == 0) {
        DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
        return LGW_SPI_ERROR;
    }
    if (address > 0x7F) {
        DEBUG_MSG("ERROR: ADDRESS OUT OF SPI RANGE\n");
        return LGW_SPI_ERROR;
    }
    
    memset(&k, 0, sizeof(k)); /* clear k */
    command = READ_ACCESS | (address & 0x7F);
    k[0].tx_buf = (unsigned long) &command;
    k[0].len = 1;
    k[0].cs_change = 0;
    k[1].cs_change = 1;
    
    byte_to_trans = size;
    while (byte_to_trans > 0) {
        chunk_size = (byte_to_trans < LGW_BURST_CHUNK) ? byte_to_trans : LGW_BURST_CHUNK;
        k[1].rx_buf = (unsigned long)(data + offset);
        k[1].len = chunk_size;
        
        byte_transfered += (ioctl(spi_device, SPI_IOC_MESSAGE(2), &k) - 1 );
        DEBUG_PRINTF("BURST READ: to trans %d # chunk %d # transferred %d \n", byte_to_trans, chunk_size, byte_transfered);
        
        byte_to_trans -= chunk_size;
        offset += chunk_size;
    } 
    
    if (byte_transfered != size) {
        DEBUG_MSG("ERROR: SPI BURST READ FAILURE\n");
        return LGW_SPI_ERROR;
    } else {
        DEBUG_MSG("Note: SPI burst read success\n");
        return LGW_SPI_SUCCESS;
    }
}

/* --- EOF ------------------------------------------------------------------ */
