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



// LED
#define HAL_CFG_LED_GREEN  

// CELL
#define HAL_CFG_CELL_TXD   NRF_GPIO_PIN_MAP(0,9)
#define HAL_CFG_CELL_RXD   NRF_GPIO_PIN_MAP(0,10)

#endif // #ifndef __HAL_CFG_H__
