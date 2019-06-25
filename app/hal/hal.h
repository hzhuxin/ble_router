/**
 * @brief The hardware abstract layer
 * 
 * @file hal.h
 * @date 2018-06-22
 * @author Dengjian
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

#ifndef __HAL_H__
#define __HAL_H__

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <limits.h>
#include <stdbool.h>

/* Typedefs ------------------------------------------------------------------*/

typedef enum hal_err {
    HAL_ERR_OK = 0,

    HAL_ERR_NOT_SUPPORT = INT_MIN,
    HAL_ERR_NOT_INITIALIZED,
    HAL_ERR_ALREADY_INITIALIZED,
    HAL_ERR_BUSY,
    HAL_ERR_FAIL,
    HAL_ERR_DATA,
    HAL_ERR_PARAM,
    HAL_ERR_MEMORY,
    HAL_ERR_DENIAL,
    HAL_ERR_TIMEOUT,
    HAL_ERR_UNLIKELY,
} hal_err_t;


/* Global variables ----------------------------------------------------------*/

/* Defines -------------------------------------------------------------------*/

#endif // #ifndef __HAL_H__

