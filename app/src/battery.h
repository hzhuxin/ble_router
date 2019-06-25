/*!
 *    @file  BATT.h
 *   @brief  Battery module
 *
 *  @author  Dale.J (dj), Dale.J@zoho.com
 *
 *  @internal
 *       Created:  04/ 7/2016
 *      Revision:  none
 *  Organization:  Druid Tech
 *     Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */

#ifndef __BATT_H__
#define __BATT_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Defines -------------------------------------------------------------------*/
#define BATT_SHUT_VOLT   3600

/* Global functions ----------------------------------------------------------*/
bool BATT_HardwareTest(uint16_t *pVoltage);
bool BATT_UpdateVoltage(void);
uint16_t BATT_GetVoltage(void);

#endif // #ifndef __BATT_H__

