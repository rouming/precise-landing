/*! ------------------------------------------------------------------------------------------------------------------
 * @file    hal.c
 * @brief   hardware abstraction layer (HAL) source file
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */
 
#include "hal.h"
#include "hal_log.h"
#include "hal_interface.h"
#include <time.h>
#include <stdarg.h>
#include <sys/time.h>

/** 
 * @brief Wait specified time in milisecond
 *
 * @param[in] msec in milisecond
 *
 * @return none
 */
#define DELAY_MULTIPLIER         1000     /* 1 second = 1,000 millisecond */
#define NANO_SECOND_MULTIPLIER   1000000  /*DELAY_MULTIPLIER * NANO_SECOND_MULTIPLIER = 1,000,000,000 */
void HAL_Delay(int msec)
{
    struct timespec tim;

    tim.tv_sec = msec / DELAY_MULTIPLIER;
    tim.tv_nsec = (msec - tim.tv_sec * DELAY_MULTIPLIER) * NANO_SECOND_MULTIPLIER;

    while (nanosleep(&tim, &tim) < 0);
}

/** 
 * @brief get the current sys time in microsecond
 *
 * @param[in] msec in milisecond
 *
 * @return current sys time, 64-bit
 */
uint64_t HAL_GetTime64(void)
{
   struct timeval time;
   uint64_t time_output;
   
   gettimeofday(&time,NULL);
   time_output = (uint64_t)time.tv_usec + ((uint64_t)time.tv_sec)*1000000;
   return time_output;   
}

/** 
 * @brief no operation
 *
 * @param none
 *
 * @return none
 */
void HAL_Nop(void)
{
   //no operation
}

/** 
 * @brief acquire current device number
 *
 * @param none
 *
 * @return current device number
 */
int HAL_DevNum()
{   
#if INTERFACE_NUMBER == 0
   return 0; // UART has only DEV0
#elif INTERFACE_NUMBER == 1
   return HAL_SPI_Which();
#elif INTERFACE_NUMBER == 2
   return HAL_SPI_Which();
#else
   return 0;
#endif
}

/** 
 * @brief HAL_Print redefines printf
 */
void HAL_Print(const char* format, ... )
{
   va_list args;
   va_start( args, format);
#if PRINT_LVL == 0
   HAL_Nop();
#elif PRINT_LVL == 1   
   vfprintf( HAL_Log_GetFile(), format, args );
   fflush(HAL_Log_GetFile());   
#elif PRINT_LVL == 2   
   vprintf(format, args);
#elif PRINT_LVL == 3
   vprintf(format, args);
   vfprintf( HAL_Log_GetFile(), format, args );
   fflush(HAL_Log_GetFile());   
#else
   HAL_Nop();
#endif //USE_HAL_PRINT   
   va_end( args );
}


