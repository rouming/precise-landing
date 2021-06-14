/*! ------------------------------------------------------------------------------------------------------------------
 * @file    hal_spi.c
 * @brief   utility to operate spi device based on Linux system
 *
 * @attention
 *
 * Copyright 2017 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */
#include <stdbool.h>
#include <fcntl.h>   // for open
#include <unistd.h>  // for close
#include <string.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h> 
#include "hal_spi.h"
#include "hal_log.h"
#include "hal.h"

#define HAL_SPI_BITS                 8
#define HAL_SPI_SPEED                2000000
#define HAL_SPI_DELAY                0

static const char *device0 = "/dev/spidev0.0";
static const char *device1 = "/dev/spidev0.1";
static struct spi_ioc_transfer tr_data;
static unsigned char spi_tx; 
static unsigned char spi_rx;

static int curr_dev = 0;
static int curr_dev_fd = 0;
static int spi_dev0_fd = 0;
static int spi_dev1_fd = 0;

/** 
 * @brief initializes the current SPI device, default /dev/spidev0.0
 *        use HAL_SPI_Sel to set current spi device
 *        use HAL_SPI_Which to get current spi device
 *
 * @param none
 *
 * @return Error code
 */
int HAL_SPI_Init(void)
{	      
	int ret = 0;   
   uint8_t mode = 0;
   uint8_t bits = HAL_SPI_BITS;
   uint32_t speed = HAL_SPI_SPEED;
   uint16_t delay = HAL_SPI_DELAY;
   int fd;
   const char *device_str;   
   
   device_str = curr_dev ? device1 : device0;
   
	fd = open(device_str, O_RDWR);
	if (fd < 0){
		HAL_Log("hal: *** ERROR *** SPI%d: Can't open device.\n", HAL_SPI_Which()); 
      return fd;
   }
   
   curr_dev_fd = fd;
   if(curr_dev == 0){
      spi_dev0_fd = fd;
   }else{      
      spi_dev1_fd = fd;
   }
	/*
	 * spi mode
	 */
	ret = ioctl(curr_dev_fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1){
		HAL_Log("hal: *** ERROR *** SPI%d: can't set spi mode.\n", HAL_SPI_Which());
      return ret;
   }
 
	ret = ioctl(curr_dev_fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1){
		HAL_Log("hal: *** ERROR *** SPI%d: can't get spi mode.\n", HAL_SPI_Which());
      return ret;
   }

	/*
	 * bits per word
	 */
	ret = ioctl(curr_dev_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1){
		HAL_Log("hal: *** ERROR *** SPI%d: can't set bits per word.\n", HAL_SPI_Which());
      return ret;
   }
 
	ret = ioctl(curr_dev_fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1){
		HAL_Log("hal: *** ERROR *** SPI%d: can't get bits per word. \n", HAL_SPI_Which());
      return ret;
   }
 
	/*
	 * max speed hz
	 */
	ret = ioctl(curr_dev_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1){
		HAL_Log("hal: *** ERROR *** SPI%d: can't set max speed hz.\n", HAL_SPI_Which());
      return ret;
   }
 
	ret = ioctl(curr_dev_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1){
		HAL_Log("hal: *** ERROR *** SPI%d: can't get max speed hz. \n", HAL_SPI_Which());
      return ret;
   }
 
	tr_data.tx_buf = (unsigned long)&spi_tx;
	tr_data.rx_buf = (unsigned long)&spi_rx;
	tr_data.len = 1;
	tr_data.delay_usecs = delay;
	tr_data.speed_hz = speed;
	tr_data.bits_per_word = bits;
	tr_data.cs_change = 0;

	HAL_Log("hal:     SPI%d: spi mode: %d\n", HAL_SPI_Which(), mode);
	HAL_Log("hal:     SPI%d: bits per word: %d\n", HAL_SPI_Which(), bits);
	HAL_Log("hal:     SPI%d: max speed: %d Hz (%d KHz)\n", HAL_SPI_Which(), speed, speed/1000);
   
   return curr_dev_fd;
}

/** 
 * @brief de-initializes the current SPI device
 *
 * @param none
 *
 * @return none
 */
void HAL_SPI_DeInit(void)
{
   close(curr_dev_fd);   
}


/** 
 * @brief set current spi device
 *
 * @param [in] spi dev number, 0 or 1
 *
 * @return none
 */
void HAL_SPI_Sel(int dev)
{
   const char *device_str;
   curr_dev = dev;
   device_str = dev ? device1 : device0;
   curr_dev_fd = dev==0 ? spi_dev0_fd : spi_dev1_fd;
   HAL_Log("hal:     SPI%d: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Device select: %s\n", HAL_SPI_Which(), device_str);
}

/** 
 * @brief acquire current spi device
 *
 * @param none
 *
 * @return current spi device
 */
int HAL_SPI_Which(void)
{
   return curr_dev;
}

/** 
 * @brief transmit data of length over the current SPI device
 *
 * @param [in] data: pointer to the TX data
 * @param [in] length: length of data to be transmitted
 *
 * @return Error code
 */
int HAL_SPI_Tx(uint8_t* tx_data, uint8_t* length)
{
	uint16_t i;   
   uint8_t rx_data[HAL_SPI_MAX_LENGTH];
   uint8_t tx_length = *length;
   uint16_t str_len = 0;
   int errno; 
	char print_str[HAL_SPI_MAX_PRINT_LENGTH];
   
   if(tx_length == 0){
      return HAL_OK;
   } 
   if(tx_length > HAL_SPI_MAX_LENGTH){
      HAL_Log("hal: *** ERROR *** SPI%d: Tx length exceeds the limit: %d\n", HAL_SPI_Which(), (uint16_t)HAL_SPI_MAX_LENGTH);
      return HAL_ERR;
   }      
   
   errno = snprintf(print_str, HAL_SPI_MAX_PRINT_LENGTH, "hal:     SPI: Tx %d bytes: 0x", tx_length);   
   str_len = (errno >= 0)? strlen(print_str):0;
   for(i = 0; i <tx_length; i++){
      errno = snprintf(print_str+str_len, HAL_SPI_MAX_PRINT_LENGTH-str_len, "%02x", tx_data[i]);
      str_len += (errno >= 0)? 2:0;
   }
   errno = snprintf(print_str+str_len, HAL_SPI_MAX_PRINT_LENGTH-str_len, "\n");
   str_len += (errno >= 0)? 1:0;
   HAL_Log("%s", print_str); 
   
	tr_data.tx_buf = (unsigned long)tx_data;
	tr_data.rx_buf = (unsigned long)rx_data;
	tr_data.len = tx_length;
      
	errno = ioctl(curr_dev_fd, SPI_IOC_MESSAGE(1), &tr_data);
	if (errno == -1){
		HAL_Log("hal: *** ERROR *** SPI%d: Error in %s\n", HAL_SPI_Which(), __func__);
      return HAL_ERR;
   }
   
   //set back values to protect   
	tr_data.tx_buf = (unsigned long)&spi_tx;
	tr_data.rx_buf = (unsigned long)&spi_rx;
   
   return HAL_OK;
}

/** 
 * @brief receive data of length over the current SPI device
 *
 * @param [in] data: pointer to the RX data buffer
 * @param [in] length: length of data to be received
 *
 * @return Error code
 */
int HAL_SPI_Rx(uint8_t* rx_data, uint8_t* length)
{
	uint16_t i;
   uint8_t tx_data[HAL_SPI_MAX_LENGTH];
   uint8_t rx_length = *length;
   uint16_t str_len = 0;
   int errno; 
	char print_str[HAL_SPI_MAX_PRINT_LENGTH];
   
   if(rx_length == 0){
      return HAL_OK;
   } 
   
   if(rx_length > HAL_SPI_MAX_LENGTH){
      HAL_Log("hal: *** ERROR *** SPI%d: Rx length exceeds the limit: %d.\n", HAL_SPI_Which(), (uint16_t)HAL_SPI_MAX_LENGTH);
      return HAL_ERR;
   }
   
   memset(tx_data, 0xFF, rx_length);//dont care. can be any byte not only 0xFF
   
	tr_data.tx_buf = (unsigned long)tx_data;
	tr_data.rx_buf = (unsigned long)rx_data;
	tr_data.len = rx_length;
   
	errno = ioctl(curr_dev_fd, SPI_IOC_MESSAGE(1), &tr_data);
	if (errno == -1){
		HAL_Log("hal: *** ERROR *** SPI%d: Error in %s\n", HAL_SPI_Which(), __func__);
      return HAL_ERR;
   }
   
   //set back values to protect   
	tr_data.tx_buf = (unsigned long)&spi_tx;
	tr_data.rx_buf = (unsigned long)&spi_rx;
   
   errno = snprintf(print_str, HAL_SPI_MAX_PRINT_LENGTH, "hal:     SPI: Rx %d bytes: 0x", rx_length);   
   str_len = (errno >= 0)? strlen(print_str):0;
   for(i = 0; i <rx_length; i++){
      errno = snprintf(print_str+str_len, HAL_SPI_MAX_PRINT_LENGTH-str_len, "%02x", rx_data[i]);
      str_len += (errno >= 0)? 2:0;
   }
   errno = snprintf(print_str+str_len, HAL_SPI_MAX_PRINT_LENGTH-str_len, "\n");
   str_len += (errno >= 0)? 1:0;
   HAL_Log("%s", print_str); 
   
   return HAL_OK;
}





