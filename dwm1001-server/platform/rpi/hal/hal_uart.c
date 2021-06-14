/*! ------------------------------------------------------------------------------------------------------------------
 * @file    hal_uart.c
 * @brief   utility to operate uart device based on Linux system
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */
#include <stdbool.h>
#include <string.h>
#include <unistd.h>			//Used for UART (close)
#include <fcntl.h>			//Used for UART (open)
#include <termios.h>		   //Used for UART
#include <stdio.h>
#include <stdint.h>
#include <sys/signal.h>
#include "hal_uart.h"
#include "hal_log.h"
#include "hal.h"

static void HAL_UART_RxCb(int status);

static bool uart_initialized = false;
static int uart0_filestream = -1;
static void (*rx_cb)(int status) = NULL;

/** 
 * @brief initializes the UART device
 *
 * @param none
 *
 * @return Error code
 */
int HAL_UART_Init(void)
{	
   if(uart_initialized)
   {
      return HAL_OK;
   }

	HAL_Log("hal:     UART: Init start.\n");
   
   // SETUP UART    
   uart0_filestream = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
   
   // open() return the new file descriptor, or -1 if an error occurred.
   // More detail in  http://man7.org/linux/man-pages/man2/open.2.html
   if (uart0_filestream == -1)
   {
      //ERROR - CAN'T OPEN SERIAL PORT
      HAL_Log("hal: *** ERROR *** UART: Unable to open UART. \n");
      HAL_Log("hal: *** ERROR *** UART: Ensure it is not in use by another application\n");
      return HAL_ERR;
   }
   
   //More info about sigaction in: http://pubs.opengroup.org/onlinepubs/7908799/xsh/sigaction.html
   struct sigaction saio;
   saio.sa_handler = HAL_UART_RxCb;
   saio.sa_flags = 0;
   saio.sa_restorer = NULL; 
   sigaction(SIGIO,&saio,NULL);

   fcntl(uart0_filestream, F_SETFL, FNDELAY|FASYNC);
   fcntl(uart0_filestream, F_SETOWN, getpid());
   
   //CONFIGURE THE UART
   struct termios options;   
   tcgetattr(uart0_filestream, &options);
   options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
   options.c_iflag = IGNPAR;
   options.c_oflag = 0;
   options.c_lflag = 0;
   tcflush(uart0_filestream, TCIFLUSH);
   tcsetattr(uart0_filestream, TCSANOW, &options);
      
   HAL_Delay(2); //required to make flush work, for some reason
   tcflush(uart0_filestream, TCIOFLUSH);
  
   uart_initialized = true;
   
	HAL_Log("hal:     UART: Init done.\n");
   return HAL_OK;
}

/** 
 * @brief de-initializes the UART device
 *
 * @param none
 *
 * @return none
 */
int HAL_UART_DeInit(){   
   // CLOSE UART 
   if (uart0_filestream != -1)
   {
      HAL_Log("hal:     UART: DeInit.\n");
      close(uart0_filestream);
      uart0_filestream = -1;
   }   
   uart_initialized = false;
   
   return HAL_OK;
}

/** 
 * @brief flush non-transmitted output data, non-read input data or both
 *
 * @param none
 *
 * @return none
 */
void HAL_UART_Flush(void)
{
   tcflush(uart0_filestream, TCIOFLUSH);
}  

/** 
 * @brief transmit data of length over UART
 *
 * @param [in] data: pointer to the Tx data buffer
 * @param [in] length: length of data to be received
 *
 * @return Error code
 */
int HAL_UART_Tx(uint8_t* data, uint8_t* length)
{
   int errno = 0;
   uint8_t i;
   uint8_t tx_length = *length;
   uint16_t str_len = 0;
	char print_str[HAL_UART_MAX_PRINT_LENGTH];
     
   if(!uart_initialized)
   {
      HAL_Log("hal: *** ERROR *** UART: not initialized. \n");
      return HAL_ERR;
   }
   if(tx_length == 0)
   {
      HAL_Log("hal: *** ERROR *** UART: Tx length is 0. \n");
      return HAL_ERR;
   }   
   if(tx_length > HAL_UART_MAX_LENGTH)
   {
      HAL_Log("hal: *** ERROR *** UART: Tx length %d exceeds max %d \n", tx_length, HAL_UART_MAX_LENGTH);
      return HAL_ERR;
   }    
   
   HAL_UART_Flush();

   errno = write(uart0_filestream, data, tx_length);		//Filestream, data pointer, number of bytes to write
   if (errno < 0)
   {
      HAL_Log("hal: *** ERROR *** UART: %s err_no: %d\n", __func__, errno);
      return HAL_ERR;
   }
   
   errno = snprintf(print_str, HAL_UART_MAX_PRINT_LENGTH, "hal:     UART: Tx %d bytes: 0x", tx_length);   
   str_len = (errno >= 0)? strlen(print_str):0;
   for(i = 0; i <tx_length; i++){
      errno = snprintf(print_str+str_len, HAL_UART_MAX_PRINT_LENGTH-str_len, "%02x", data[i]);
      str_len += (errno >= 0)? 2:0;
   }
   errno = snprintf(print_str+str_len, HAL_UART_MAX_PRINT_LENGTH-str_len, "\n");
   str_len += (errno >= 0)? 1:0;
   HAL_Log("%s", print_str); 
   
   return HAL_OK;
}
  
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
int HAL_UART_Rx(uint8_t* data, uint8_t* length)
{
   int errno = 0;
	uint16_t i;
   uint16_t str_len = 0;
	char print_str[HAL_UART_MAX_PRINT_LENGTH];
   
   if(!uart_initialized)
   {
      HAL_Log("hal: *** ERROR *** UART: not initialized.\n");
      return HAL_ERR;
   }   
   if(*length == 0)
   {
      return HAL_OK;
   } 
   
   // Read up to 255 characters from the port if they are there
   // read(Filestream, buffer to store in, number of bytes to read (max));
   int rx_length = read(uart0_filestream, (void*)data, *length);
   if (rx_length < 0)
   {
      //An error occured (will occur if there are no bytes)
      HAL_Log("hal: *** ERROR *** UART: %s err_no: %d\n", __func__, rx_length);
      *length = 0;
      return HAL_ERR;
   }
   else if (rx_length == 0)
   {
      //No data waiting
      *length = 0;
   }
   else
   {
      //Bytes received      
      errno = snprintf(print_str, HAL_UART_MAX_PRINT_LENGTH, "hal:     UART: Rx %d bytes: 0x", rx_length);   
      str_len = (errno >= 0)? strlen(print_str):0;
      for(i = 0; i <rx_length; i++){
         errno = snprintf(print_str+str_len, HAL_UART_MAX_PRINT_LENGTH-str_len, "%02x", data[i]);
         str_len += (errno >= 0)? 2:0;
      }
      errno = snprintf(print_str+str_len, HAL_UART_MAX_PRINT_LENGTH-str_len, "\n");
      str_len += (errno >= 0)? 1:0;
      HAL_Log("%s", print_str); 
      
      *length = rx_length;
   }
   return HAL_OK;
}

/** 
 * @brief setup the UART receive callback function. 
 *
 * @param [in] cb_func: callback function pointer. 
 *
 * @return none
 */
void HAL_UART_SetRxCb(void (*cb_func)(int))
{
   rx_cb = cb_func;
}

/** 
 * @brief UART receive callback function. This function will be called when 
 *        there is incoming data available. 
 *
 * @param [in] status: signal status. 
 *
 * @return none
 */
static void HAL_UART_RxCb(int status)
{
   if(rx_cb != NULL)
   {
      (*rx_cb)(status);
   }
}



