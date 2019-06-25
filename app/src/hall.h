/*!
 *    @file  Hall.h
 *   @brief  The hall mode
 *
 *  @author  Dale.J (dj), Dale.J@zoho.com
 *
 *  @internal
 *       Created:  05/ 5/2016
 *      Revision:  none
 *  Organization:  Druid Tech
 *     Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */

#ifndef __HALL_H__
#define __HALL_H__
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Typedefs ------------------------------------------------------------------*/
typedef void (*HALL_Callback_t)(bool is_released);

/* Function prototypes -------------------------------------------------------*/
void HALL_Init(HALL_Callback_t cb);

#endif // #ifndef __HALL_H__
