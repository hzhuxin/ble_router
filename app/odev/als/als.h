/*!
 *    @file  als.h
 *   @brief  Ambient light sensor based on ISL290xx
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

#ifndef __ALS_H__
#define __ALS_H__

/* Includes ------------------------------------------------------------------*/
#include "hal.h"
#include "hal_i2c.h"

/* Defines -------------------------------------------------------------------*/

/* Typedefs ------------------------------------------------------------------*/
struct dev_als_ops;
typedef struct dev_als {
    const struct dev_als_ops* ops;
    hal_i2c_t *i2c;
    bool initiated;
    void* lock;
} dev_als_t;

typedef struct dev_als_ops {
    hal_err_t (*init) (dev_als_t *als);
    hal_err_t (*deinit) (dev_als_t *als);

    hal_err_t (*lock) (dev_als_t *als);
    hal_err_t (*unlock) (dev_als_t *als);

    hal_err_t (*update) (dev_als_t* als);
    hal_err_t (*read) (dev_als_t* als, uint32_t* light);
} dev_als_ops_t;

/* Function prototypes -------------------------------------------------------*/
dev_als_t* dev_isl29035_get_instance(hal_i2c_t* i2c);

#endif // #ifndef __ALS_H__

