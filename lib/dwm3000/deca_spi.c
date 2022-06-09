/*! ----------------------------------------------------------------------------
 * @file    deca_spi.c
 * @brief   SPI access functions
 *
 * @attention
 *
 * Copyright 2015-2020 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include <driver/spi_master.h>
#include <string.h>

#include "deca_spi.h"
#include "deca_device_api.h"

spi_device_handle_t deca_spi_device; 

/****************************************************************************//**
 *
 *                              DW3000 SPI section
 *
 *******************************************************************************/
/*! ------------------------------------------------------------------------------------------------------------------
 * Function: openspi()
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(/*SPI_TypeDef* SPIx*/)
{
    return 0;
} // end openspi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: closespi()
 *
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void)
{
    return 0;
} // end closespi()




/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospiwithcrc()
 *
 * Low level abstract function to write to the SPI when SPI CRC mode is used
 * Takes two separate byte buffers for write header and write data, and a CRC8 byte which is written last
 * returns 0 for success, or -1 for error
 */
int writetospiwithcrc(
                uint16_t      headerLength,
                const uint8_t *headerBuffer,
                uint16_t      bodyLength,
                const uint8_t *bodyBuffer,
                uint8_t       crc8)
{
    uint16_t len = headerLength + bodyLength + 1;
    uint8_t tx_buf[len];
    memcpy(tx_buf, headerBuffer, headerLength);
    memcpy(tx_buf + headerLength, bodyBuffer, bodyLength);
    memcpy(tx_buf + headerLength + bodyLength, &crc8, 1);

    spi_transaction_t trans = {
        .tx_buffer = tx_buf,
        .length = len * 8
    };
    spi_device_acquire_bus(deca_spi_device, ~0);
    spi_device_polling_transmit(deca_spi_device, &trans);
    spi_device_release_bus(deca_spi_device);
    return 0;
} // end writetospiwithcrc()


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
int writetospi(uint16_t       headerLength,
               const uint8_t  *headerBuffer,
               uint16_t       bodyLength,
               const uint8_t  *bodyBuffer)
{
    uint16_t len = headerLength + bodyLength;
    uint8_t tx_buf[len];
    memcpy(tx_buf, headerBuffer, headerLength);
    memcpy(tx_buf + headerLength, bodyBuffer, bodyLength);

    spi_transaction_t trans = {
        .tx_buffer = tx_buf,
        .length = len * 8
    };
    spi_device_acquire_bus(deca_spi_device, ~0);
    spi_device_polling_transmit(deca_spi_device, &trans);
    spi_device_release_bus(deca_spi_device);
    return 0;
} // end writetospi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
//#pragma GCC optimize ("O3")
int readfromspi(uint16_t  headerLength,
                uint8_t   *headerBuffer,
                uint16_t  readLength,
                uint8_t   *readBuffer)
{
    uint8_t rx_buf[headerLength + readLength];
    memset(rx_buf, 0, headerLength + readLength);
    spi_transaction_t trans = {
        .tx_buffer = headerBuffer,
        .rx_buffer = rx_buf,
        .length = (headerLength + readLength) * 8,
        .rxlength = (headerLength + readLength) * 8,
    };
    spi_device_acquire_bus(deca_spi_device, ~0);
    spi_device_polling_transmit(deca_spi_device, &trans);
    spi_device_release_bus(deca_spi_device);
    memcpy(readBuffer, rx_buf + headerLength, readLength);
    return 0;
} // end readfromspi()

/****************************************************************************//**
 *
 *                              END OF DW1000 SPI section
 *
 *******************************************************************************/