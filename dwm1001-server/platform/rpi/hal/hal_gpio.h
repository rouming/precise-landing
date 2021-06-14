/*! ------------------------------------------------------------------------------------------------------------------
 * @file    hal_gpio.h
 * @brief   utility to operate GPIOs
 *          this lib is Raspberry-Pi dependent
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */
#ifndef _HAL_GPIO_H_
#define _HAL_GPIO_H_

#include <wiringPi.h>

#define HAL_GPIO_INT_EDGE_FALLING   INT_EDGE_FALLING
#define HAL_GPIO_INT_EDGE_RISING    INT_EDGE_RISING
#define HAL_GPIO_INT_EDGE_BOTH      INT_EDGE_BOTH   
#define HAL_GPIO_INT_EDGE_SETUP     INT_EDGE_SETUP

#define HAL_GPIO_DRDY  3

/** 
 * @brief initializes the GPIO utilities
 *
 * @param none
 *
 * @return Error code
 */
int HAL_GPIO_Init(void);

/** 
 * @brief setup pin interrupt callback function on certain condition
 *
 * @param [in] pin: GPIO pin number of the Raspberry Pi external connector.
 * @param [in] edge_type: GPIO edge type to trigger interrupt
 * @param [in] cb: callback function pointer to be called when interrupt on the pin happens. 
 *
 * @return Error code
 */
int HAL_GPIO_SetupCb(int pin, int edge_type, void (*cb)(void));

/** 
 * @brief Reads the GPIO pin
 *
 * @param [in] pin: GPIO pin to be read.
 *
 * @return GPIO value
 */
int HAL_GPIO_PinRead(int pin);

#endif //_HAL_GPIO_H_


