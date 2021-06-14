/*! ------------------------------------------------------------------------------------------------------------------
 * @file    test_util.h
 * @brief   Decawave device configuration and control functions
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#ifndef _TEST_UTIL_H_
#define _TEST_UTIL_H_

#include <stdio.h>
#include <stdlib.h>

int Test_Check(int rv);

int Test_CheckTxRx(int rv);

int Test_CheckValue(int rv);

void Test_End(void);

FILE * Test_GetReportFile(void);

void Test_CloseReportFile(void);

void Test_Report(const char* format, ... );

#endif //_RV_UTIL_H_
