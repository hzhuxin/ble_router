/**
 * @brief The temperature sensor abstract layer
 * 
 * @file temp.h
 * @date 2018-07-12
 * @author Dengjian
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

#ifndef __HAL_TEMP_H__
#define __HAL_TEMP_H__

/* Includes ------------------------------------------------------------------*/
#include "hal.h"

/* Typedefs ------------------------------------------------------------------*/

/* Defines -------------------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/

/* Global functions ----------------------------------------------------------*/

hal_err_t hal_temp_update(void);
float hal_temp_get(void); // Get temperature in degree(float)
hal_err_t hal_temp_get_int(void); // Get temperature in 0.1 degree(int)

#endif // #ifndef __TEMP_H__

