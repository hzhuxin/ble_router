/*!
 *    @file  bmx.h
 *   @brief  The bmx module
 *
 *  @author  Dale.J (dj), Dale.J@zoho.com
 *
 *  @internal
 *       Created:  01/29/2018
 *      Revision:  none
 *  Organization:  Druid Tech
 *     Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */

#ifndef __BMX_H__
#define __BMX_H__

/* Includes ------------------------------------------------------------------*/
#include "hal.h"
#include "hal_i2c.h"

/* Defines -------------------------------------------------------------------*/

/* Typedefs ------------------------------------------------------------------*/
struct dev_bmx_ops;
typedef struct dev_bmx {
    const struct dev_bmx_ops* ops;
    hal_i2c_t *i2c;
    bool initiated;
    void* lock;
} dev_bmx_t;

typedef struct dev_bmx_ops {
    hal_err_t (*init) (dev_bmx_t *bmx);
    hal_err_t (*deinit) (dev_bmx_t *bmx);

    hal_err_t (*lock) (dev_bmx_t *bmx);
    hal_err_t (*unlock) (dev_bmx_t *bmx);

    hal_err_t (*update) (dev_bmx_t* bmx);
    hal_err_t (*read) (dev_bmx_t* bmx, int16_t* temperature, uint8_t* humidity, uint16_t* pressure);
} dev_bmx_ops_t;

/* Function prototypes -------------------------------------------------------*/
dev_bmx_t* dev_bmp280_get_instance(hal_i2c_t* i2c);
dev_bmx_t* dev_bme280_get_instance(hal_i2c_t* i2c);

#endif // #ifndef __BMX_H__

