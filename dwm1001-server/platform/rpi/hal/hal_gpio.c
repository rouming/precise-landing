/*! ------------------------------------------------------------------------------------------------------------------
 * @file    hal_gpio.c
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
#include <wiringPi.h> // this lib is Raspberry-Pi dependent
#include "hal.h"

/** 
 * @brief initializes the GPIO utilities
 *
 * @param none
 *
 * @return Error code
 */
int HAL_GPIO_Init(void)
{
   if (wiringPiSetup () < 0)
   {
      HAL_Print ("hal: *** ERROR *** GPIO: Unable to Init wiringPi...\n") ;
      return HAL_ERR;
   }
   return HAL_OK;
}

/** 
 * @brief setup pin interrupt callback function on certain condition
 *
 * @param [in] pin: GPIO pin number of the Raspberry Pi external connector.
 * @param [in] edge_type: GPIO edge type to trigger interrupt
 * @param [in] cb: callback function pointer to be called when interrupt on the pin happens. 
 *
 * @return Error code
 */
int HAL_GPIO_SetupCb(int pin, int edge_type, void (*cb)(void))
{
   HAL_GPIO_Init();
   if (wiringPiISR (pin, edge_type, cb) < 0)
   {
      HAL_Print ("hal: *** ERROR *** GPIO: Unable to setup GPIO ISR...\n") ;
      return HAL_ERR;
   }
   return HAL_OK;
}

/** 
 * @brief Reads the GPIO pin
 *
 * @param [in] pin: GPIO pin to be read.
 *
 * @return GPIO value
 */
int HAL_GPIO_PinRead(int pin)
{
   return digitalRead(pin);
}


   
   