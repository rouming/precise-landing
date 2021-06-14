/*! ------------------------------------------------------------------------------------------------------------------
 * @file    hal.h
 * @brief   hardware abstraction layer (HAL) header file
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */
#ifndef _HAL_H_
#define _HAL_H_

#include <stdio.h>
#include "hal_interface.h"

#define HAL_OK          0
#define HAL_ERR         1

/** 
 * @brief HAL_Print redefines printf
 */
void HAL_Print(const char* format, ... );

/** 
 * @brief Wait specified time in milisecond
 *
 * @param[in] msec in milisecond
 *
 * @return none
 */
void HAL_Delay(int msec);

/** 
 * @brief get the current sys time in microsecond
 *
 * @param[in] msec in milisecond
 *
 * @return current sys time, 64-bit
 */
uint64_t HAL_GetTime64(void);

/** 
 * @brief no operation
 *
 * @param none
 *
 * @return none
 */
void HAL_Nop(void);

/** 
 * @brief acquire current device number
 *
 * @param none
 *
 * @return current device number
 */
int HAL_DevNum();

#endif //_HAL_H_

