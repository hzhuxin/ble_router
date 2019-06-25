/*!
 *    @file  als.c
 *   @brief  The als module based on bme280
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


/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "als/als.h"
#include "als/isl29035.h"

#include "nrf_log.h"
/* Defines -------------------------------------------------------------------*/
#define DEV_ADDR            (0x44)

/* Typedefs ------------------------------------------------------------------*/

/* alsate variables ---------------------------------------------------------*/
static const int LOCK_TIMEOUT = pdMS_TO_TICKS(2000);
static dev_als_t _als = { 0 };
static bool _valid = false;
static uint32_t _light = 0;

/* Private functions ---------------------------------------------------------*/

static bool als_read(uint32_t *light) {
  dev_als_t* als = &_als;
  uint16_t sample = 0;
  uint8_t val = CMDII_RES_16BIT | CMDII_RNG_4K;

  if(als->i2c->ops->write_reg(als->i2c, DEV_ADDR, ALS_REG_CMDII, 1, &val, sizeof(val)) <= 0) {
    return false;
  }

  val = CMDI_ALS_ONCE;
  if(als->i2c->ops->write_reg(als->i2c, DEV_ADDR, ALS_REG_CMDI, 1, &val, sizeof(val)) <= 0) {
    return false;
  }

  vTaskDelay(pdMS_TO_TICKS(SAMPLE_TIME_16BIT));
  if(als->i2c->ops->read_reg(als->i2c,  DEV_ADDR, ALS_REG_DATAL, 1, (uint8_t*)&sample, sizeof(sample)) <= 0) {
    return false;
  }

  if(light) {
    *light = SAMPLE_TO_LUX(4000, 16, sample);
  }

  return true;
}

/**
 * @brief Init als
 *
 * @return HAL_ERR_OK if success
 */
 static hal_err_t init (dev_als_t* als) {

  if(!als | !als->lock) {
    return HAL_ERR_PARAM;
  }

  if(als->initiated) {
    return HAL_ERR_ALREADY_INITIALIZED;
  }

  als->initiated = true;

  return HAL_ERR_OK;
}

/**
 * @brief Deinit als
 *
 * @return HAL_ERR_OK if success
 */
 static hal_err_t deinit (dev_als_t* als) {

  if(!als | !als->lock) {
    return HAL_ERR_PARAM;
  }

  if(!als->initiated) {
    return HAL_ERR_NOT_INITIALIZED;
  }

  als->initiated = false;
  return 0;
}

/**
 * @brief Lock the instance for operation
 *
 * @param als
 *
 * @return HAL_ERR_OK if success
 */
static hal_err_t lock(dev_als_t *als) {

  if(!als || !als->lock) {
    return HAL_ERR_PARAM;
  }

  if(!xSemaphoreTake(als->lock, LOCK_TIMEOUT)) {
    return HAL_ERR_BUSY;
  }

  return HAL_ERR_OK;
}

/**
 * @brief Unlock the instance
 *
 * @param als
 *
 * @return HAL_ERR_OK if success
 */
static hal_err_t unlock(dev_als_t *als) {

  if(!als || !als->lock) {
    return HAL_ERR_PARAM;
  }

  if(!xSemaphoreGive(als->lock)) {
    return HAL_ERR_UNLIKELY;
  }

  return HAL_ERR_OK;
}

/**
 * @brief Write bytes
 *
 * @param als
 *
 * @return HAL_ERR_OK if success
 */
static hal_err_t update(dev_als_t* als) {

  if(!als) {
    return HAL_ERR_PARAM;
  }

  if(!als->initiated) {
    return HAL_ERR_NOT_INITIALIZED;
  }

  _valid = als_read(&_light);

  if(!_valid) {
    return HAL_ERR_DATA;
  }

  return HAL_ERR_OK;
}

/**
 * @brief Read data
 *
 * @param als
 * @param temperature in 0.1 degree Celsius
 * @param humidity in 1%
 * @param pressure in hPa
 *
 * @return
 */
static hal_err_t read (dev_als_t* als, uint32_t* light) {

  if(!als) {
    return HAL_ERR_PARAM;
  }

  if(!als->initiated) {
    return HAL_ERR_NOT_INITIALIZED;
  }

  if(light) {
    *light = _light;
  }

  if(!_valid) {
    return HAL_ERR_DATA;
  }

  return HAL_ERR_OK;
}

/* Global functions ----------------------------------------------------------*/

/**
 * @brief Get an instance
 *
 * @param als pointer to the als to use
 *
 * @return pointer to the instance if success
 */
dev_als_t* dev_isl29035_get_instance(hal_i2c_t* i2c) {
    dev_als_t* als = &_als;
    static const dev_als_ops_t ops = {
        .init = init,
        .deinit = deinit,
        .lock = lock,
        .unlock = unlock,
        .update = update,
        .read = read,
    };

    if(!i2c && !als->lock) {
      return NULL;
    }

    if(!als->lock) {
      als->lock = xSemaphoreCreateBinary();
    }

    if(!als->lock) {
      return NULL;
    }

    if(i2c) {
      als->i2c= i2c;
      als->ops = &ops;
    }

    xSemaphoreGive(als->lock);

    return als;
}


