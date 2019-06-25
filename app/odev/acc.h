/*!
 *    @file  ACC.h
 *   @brief  Accelerometer
 *
 *  @author  Dale.J (dj), Dale.J@zoho.com
 *
 *  @internal
 *       Created:  04/ 8/2016
 *      Revision:  none
 *  Organization:  Druid Tech
 *     Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */

#ifndef __ACC_H__
#define __ACC_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#include "kx022_reg.h"
// #include "xl362_reg.h"

/* Defines -------------------------------------------------------------------*/

/* Typedefs ------------------------------------------------------------------*/
typedef void (*ACC_Callback_t) (void);

/* Function prototypes -------------------------------------------------------*/
bool ACC_HardwareTest(void);
bool ACC_EnableSample(ACC_Callback_t cb);
bool ACC_EnableShake(ACC_Callback_t cb);
bool ACC_ReadStatus(uint8_t *pStatus);
bool ACC_ReadFIFO(int16_t* buf, int32_t* cnt);
bool ACC_ReadData(int16_t* buf);
bool ACC_ClearFIFO(void);
bool ACC_Init(void);
bool ACC_Deinit(void);
bool ACC_Lock(void);
bool ACC_Unlock(void);
bool ACC_HardwareReset(void);
bool ACC_SoftwareReset(void);

#endif // #ifndef __ACC_H__
