/*! ------------------------------------------------------------------------------------------------------------------
 * @file    hal_uart.h
 * @brief   utility to operate uart device based on Linux system
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */
#ifndef _HAL_UART_H_
#define _HAL_UART_H_

#include <stdint.h>

#define HAL_UART_MAX_LENGTH            255
#define HAL_UART_MAX_PRINT_LENGTH      HAL_UART_MAX_LENGTH*3

/** 
 * @brief initializes the UART device
 *
 * @param none
 *
 * @return Error code
 */
int HAL_UART_Init(void);

/** 
 * @brief de-initializes the UART device
 *
 * @param none
 *
 * @return none
 */
int HAL_UART_DeInit(void);

/** 
 * @brief flush non-transmitted output data, non-read input data or both
 *
 * @param none
 *
 * @return none
 */
void HAL_UART_Flush(void);

/** 
 * @brief transmit data of length over UART
 *
 * @param [in] data: pointer to the Tx data buffer
 * @param [in] length: pointer to length of data to be transmitted
 *
 * @return Error code
 */
int HAL_UART_Tx(uint8_t* data, uint8_t* length);

/** 
 * @brief receive data of length over UART
 *
 * @param [in] data: pointer to the RX data buffer
 * @param [inout] length: pointer to length of data expected to be received. After reading, 
 *                      the actual data length being read is written to this variable. Thus,  
 *                      after thefunction returns, value *length can be smaller than when 
 *                      the function was called.  
 *
 * @return Error code
 */
int HAL_UART_Rx(uint8_t* data, uint8_t* length);

/** 
 * @brief setup the UART receive callback function. 
 *
 * @param [in] cb_func: callback function pointer. 
 *
 * @return none
 */
void HAL_UART_SetRxCb(void (*cb_func)(int));


#endif //_HAL_UART_H_

