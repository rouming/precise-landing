/*! ------------------------------------------------------------------------------------------------------------------
 * @file    test_util.c
 * @brief   Decawave device configuration and control functions
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include "hal.h"
#include "hal_log.h"
#include "test_util.h"

static int total = 0;
static int total_ok = 0;
static int total_fail = 0;

static char test_report_file_name[]="test_report.txt";
static FILE * fp = NULL;   

/** 
 * @brief get test_report.txt file pointer. 
 *
 * @param none
 *
 * @return test_report.txt file pointer
 */
FILE * Test_GetReportFile(void)
{
   
   if(fp != NULL)
   {
      return fp;
   }
   fp = fopen (test_report_file_name, "w");
   
   return fp;
}

/** 
 * @brief de-initializes the test_report.txt file. 
 *
 * @param none
 *
 * @return none
 */
void Test_CloseReportFile(void)
{
   if(fp != NULL)
   {
      fclose(fp);  
      fp = NULL;
   }
}

/** 
 * @brief print report to  test_report.txt file. 
 *
 * @param formated strings
 *
 * @return none
 */
void Test_Report(const char* format, ... )
{
   va_list args;
   va_start( args, format );
   vfprintf( Test_GetReportFile(), format, args );
   va_end( args );
   fflush(Test_GetReportFile());   
}

// rv == 0 : ok
// rv == 1 : fail
int Test_Check(int rv)
{   
   if (rv == 0)
   {
      total_ok++;
      HAL_Log("OK\n"); 
   }
   else
   {
      total_fail++;
      HAL_Log("Fail\n");       
   }
   total++;

#if (PRINT_LVL == 1)
   static int test_start_counting = 0;
   if(test_start_counting == 0)
   {
      printf("Total :   OK : Fail\n");
      printf("%*d :", 5,total);
      printf("%*d :", 5,total_ok);
      printf("%*d \n", 5,total_fail);
      test_start_counting = 1;
   }
   printf("\b\r");   
   printf("%*d :", 5,total);
   printf("%*d :", 5,total_ok);
   printf("%*d \n", 5,total_fail);
#endif

   return (rv != 0);
}


int Test_CheckTxRx(int rv)
{
   HAL_Log("Check Tx&Rx:\n");
   return Test_Check(rv);
}

int Test_CheckValue(int rv)
{
   HAL_Log("Check Value:\n");
   return Test_Check(rv);
}

void Test_End(void)
{
   printf("\n");
   printf("Total = %d \n", total);   
   printf("   OK = %d \n", total_ok);   
   printf(" Fail = %d \n", total_fail); 
   HAL_Log("\n");
   HAL_Log("Total = %d \n", total);   
   HAL_Log("   OK = %d \n", total_ok);   
   HAL_Log(" Fail = %d \n", total_fail); 
   
   Test_CloseReportFile();
}

