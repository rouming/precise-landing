/*! ------------------------------------------------------------------------------------------------------------------
 * @file    hal_spi.h
 * @brief   utility to operate spi device based on Linux system
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */
#ifndef _HAL_SPI_H_
#define _HAL_SPI_H_

#include <stdint.h>

#define HAL_SPI_MAX_LENGTH           255
#define HAL_SPI_MAX_PRINT_LENGTH     HAL_SPI_MAX_LENGTH*3
#define HAL_SPI_WAIT_PERIOD          1

#define HAL_SPI_DEV0 0
#define HAL_SPI_DEV1 1


/** 
 * @brief initializes the current SPI device, default /dev/spidev0.0
 *        use HAL_SPI_Sel to set current spi device
 *        use HAL_SPI_Which to get current spi device
 *
 * @param none
 *
 * @return Error code
 */
int HAL_SPI_Init(void);

/** 
 * @brief de-initializes the current SPI device
 *
 * @param none
 *
 * @return none
 */
void HAL_SPI_DeInit(void);

/** 
 * @brief set current spi device
 *
 * @param [in] spi dev number, 0 or 1
 *
 * @return none
 */
void HAL_SPI_Sel(int dev);

/** 
 * @brief set current spi device
 *
 * @param none
 *
 * @return current spi device
 */
int HAL_SPI_Which(void);

/** 
 * @brief transmit data of length over the current SPI device
 *
 * @param [in] data: pointer to the TX data
 * @param [in] length: length of data to be transmitted
 *
 * @return Error code
 */
int HAL_SPI_Tx(uint8_t* data, uint8_t* length);

/** 
 * @brief receive data of length over the current SPI device
 *
 * @param [in] data: pointer to the RX data buffer
 * @param [in] length: length of data to be received
 *
 * @return Error code
 */
int HAL_SPI_Rx(uint8_t* data, uint8_t* length);

#endif //_HAL_SPI_H_

