/*!
 *    @file  hal_cfg.h
 *   @brief  The config file for hal layer
 *
 *  @author  Dale.J (dj), Dale.J@zoho.com
 *
 *  @internal
 *       Created:  05/17/2018
 *      Revision:  none
 *  Organization:  Druid Tech
 *     Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */

#ifndef __HAL_CFG_H__
#define __HAL_CFG_H__

#include "nrf_gpio.h"
/* Defines -------------------------------------------------------------------*/
#define SW_VERSION        1

// FLASH
#define HAL_CFG_FLS_NSS   
#define HAL_CFG_FLS_MISO  
#define HAL_CFG_FLS_MOSI  
#define HAL_CFG_FLS_SCK   

// KX022
#define HAL_CFG_ACC_INT   
#define HAL_CFG_ACC_NSS   
#define HAL_CFG_ACC_SCK   
#define HAL_CFG_ACC_MISO  
#define HAL_CFG_ACC_MOSI 

// I2C
#define HAL_CFG_I2C_SCL   
#define HAL_CFG_I2C_SDA   

// HALL
#define HAL_CFG_HALL_IRQ  

// GPS
#define HAL_CFG_GPS_PWR    NRF_GPIO_PIN_MAP(1,00)
#define HAL_CFG_GPS_BKP    NRF_GPIO_PIN_MAP(0,16)
#define HAL_CFG_GPS_RXD    NRF_GPIO_PIN_MAP(0,11)
#define HAL_CFG_GPS_TXD    NRF_GPIO_PIN_MAP(0,13)

// BATTERY
#define HAL_CFG_BATTERY    

// LED
#define HAL_CFG_LED_GREEN  

// CELL
#define HAL_CFG_CELL_PWR   NRF_GPIO_PIN_MAP(0,30)
#define HAL_CFG_CELL_RI    NRF_GPIO_PIN_MAP(1,11)
#define HAL_CFG_CELL_DTR   NRF_GPIO_PIN_MAP(1,15)
#define HAL_CFG_CELL_TXD   NRF_GPIO_PIN_MAP(0,26)
#define HAL_CFG_CELL_RXD   NRF_GPIO_PIN_MAP(0,4)
#define HAL_CFG_CELL_STA   NRF_GPIO_PIN_MAP(0,6)
#define HAL_CFG_CELL_KEY   NRF_GPIO_PIN_MAP(0,8)

#endif // #ifndef __HAL_CFG_H__
