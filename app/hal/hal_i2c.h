/**
 * @brief The i2c abstract layer
 * 
 * @file i2c.h
 * @date 2018-06-22
 * @author Dengjian
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

#ifndef __I2C_H__
#define __I2C_H__

/* Includes ------------------------------------------------------------------*/
#include "hal.h"
#include "hal_pin.h"

/* Typedefs ------------------------------------------------------------------*/
typedef enum hal_i2c_freq {
    HAL_I2C_FREQ_100K = 0,
    HAL_I2C_FREQ_400K,
    HAL_I2C_FREQ_3P4M,  // 3.4MHz
    HAL_I2C_FREQ_MAX,
} hal_i2c_freq_t;

typedef struct hal_i2c_cfg {
  hal_i2c_freq_t freq;
  hal_pin_t scl_pin;
  hal_pin_t sda_pin;
} hal_i2c_cfg_t;

struct hal_i2c_ops;
typedef struct hal_i2c {
  void* inst;
  void* lock;
  void* priv;
  const struct hal_i2c_ops* ops;
} hal_i2c_t;

typedef struct hal_i2c_ops {
    hal_err_t (*init) (hal_i2c_t *obj, hal_i2c_cfg_t* cfg);
    hal_err_t (*deinit) (hal_i2c_t *obj);

    hal_err_t (*open) (hal_i2c_t *obj);
    hal_err_t (*close) (hal_i2c_t *obj);

    hal_err_t (*lock) (hal_i2c_t *obj);
    hal_err_t (*unlock) (hal_i2c_t *obj);

    hal_err_t (*read) (hal_i2c_t *obj, int dev_addr, void* buf, int len);
    hal_err_t (*write) (hal_i2c_t *obj, int dev_addr, void* buf, int len);

    hal_err_t (*read_reg) (hal_i2c_t *obj, int dev_addr, int reg_addr, int reg_addr_len, void* buf, int len);
    hal_err_t (*write_reg) (hal_i2c_t *obj, int dev_addr, int reg_addr, int reg_addr_len, void* buf, int len);
} hal_i2c_ops_t;

/* Global variables ----------------------------------------------------------*/

/* Defines -------------------------------------------------------------------*/

hal_i2c_t* hal_i2c_get_instance(int id);

#endif // #ifndef __I2C_H__

