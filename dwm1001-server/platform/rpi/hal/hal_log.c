/*! ------------------------------------------------------------------------------------------------------------------
 * @file    hal_log.c
 * @brief   utility print to log file
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */
 
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

static char log_file_name[]="log.txt";
static FILE * fp = NULL;   

/** 
 * @brief get log file pointer. 
 *
 * @param none
 *
 * @return log file pointer
 */
FILE * HAL_Log_GetFile(void)
{  
   if(fp != NULL)
   {
      return fp;
   }
   fp = fopen (log_file_name, "w");
   
   return fp;
}

/** 
 * @brief de-initializes the log file. 
 *
 * @param none
 *
 * @return none
 */
void HAL_Log_DeInit(void)
{
   if(fp != NULL)
   {
      fclose(fp);  
      fp = NULL;
   }
}

/** 
 * @brief print log to log.txt file. 
 *
 * @param formated strings
 *
 * @return none
 */
void HAL_Log(const char* format, ... )
{
#if defined(HAL_LOG_ENABLED) && (HAL_LOG_ENABLED==1)
   va_list args;
   va_start( args, format );
   vfprintf( HAL_Log_GetFile(), format, args );
   va_end( args );
   fflush(HAL_Log_GetFile()); 
#endif  
}
