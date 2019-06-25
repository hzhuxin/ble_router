/*!
 *    @file  acc.c
 *   @brief  Accelerometer Sensor based on XL362 or KX022
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


/* Includes ------------------------------------------------------------------*/
#if (DEV_TYPE == 2 && HW_VERSION == 3) || (DEV_TYPE == 101 && HW_VERSION == 4)
#include "KX022/kx022.c"
#else
#include "XL362/xl362.c"
#endif

/* Defines -------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Global functions ----------------------------------------------------------*/

