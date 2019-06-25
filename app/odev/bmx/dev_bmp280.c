/*!
 *    @file  bmx.c
 *   @brief  The bmx module based on bmp280
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
#include "bmx.h"
#include "bmx/BMP280_driver/bmp280.h"
#include "nrf_log.h"

/* Defines -------------------------------------------------------------------*/

/* Typedefs ------------------------------------------------------------------*/

/* bmxate variables ---------------------------------------------------------*/
static const int LOCK_TIMEOUT = pdMS_TO_TICKS(2000);
static dev_bmx_t _bmx = { 0 };
static bool _valid = false;
static struct bmp280_dev _dev;
static struct bmp280_config _cfg;
static struct bmp280_uncomp_data _data;

/* Private functions ---------------------------------------------------------*/

static void user_delay_ms(uint32_t period) {
  vTaskDelay(pdMS_TO_TICKS(period));
}

static int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
  dev_bmx_t* bmx = &_bmx;
  if(!bmx->i2c) {
    return -1;
  }

  int ret = bmx->i2c->ops->read_reg(bmx->i2c, dev_id, reg_addr, sizeof(reg_addr), reg_data, len);

  return ret == len ? BMP280_OK : BMP280_E_COMM_FAIL;
}

static int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
  dev_bmx_t* bmx = &_bmx;
  if(!bmx->i2c) {
    return -1;
  }

  int ret = bmx->i2c->ops->write_reg(bmx->i2c, dev_id, reg_addr, sizeof(reg_addr), reg_data, len);

  return ret == len ? BMP280_OK : BMP280_E_COMM_FAIL;
}

static hal_err_t bmx280_init(void) {
  struct bmp280_dev* dev = &_dev;
  dev->dev_id = BMP280_I2C_ADDR_PRIM;
  dev->intf = BMP280_I2C_INTF;
  dev->read = user_i2c_read;
  dev->write = user_i2c_write;
  dev->delay_ms = user_delay_ms;
  if(bmp280_init(dev) != BMP280_OK) {
    return HAL_ERR_DENIAL;
  }

  struct bmp280_config *cfg = &_cfg;
  if(bmp280_get_config(cfg, dev) != BMP280_OK) {
    return HAL_ERR_DENIAL;
  }
  cfg->os_pres = BMP280_OS_8X;
  cfg->os_temp = BMP280_OS_2X;
  cfg->filter = BMP280_FILTER_OFF;

  if(bmp280_set_config(cfg, dev) != BMP280_OK) {
    return HAL_ERR_DENIAL;
  }

  if(bmp280_set_power_mode(BMP280_SLEEP_MODE, dev) != BMP280_OK) {
    return HAL_ERR_DENIAL;
  }

  return HAL_ERR_OK;
}

/**
 * @brief Init bmx
 *
 * @return HAL_ERR_OK if success
 */
 static hal_err_t init (dev_bmx_t* bmx) {

  if(!bmx | !bmx->lock) {
    return HAL_ERR_PARAM;
  }

  if(bmx->initiated) {
    return HAL_ERR_ALREADY_INITIALIZED;
  }

  if(bmx280_init() != HAL_ERR_OK) {
    return HAL_ERR_DENIAL;
  }

  bmx->initiated = true;

  return HAL_ERR_OK;
}

/**
 * @brief Deinit bmx
 *
 * @return HAL_ERR_OK if success
 */
 static hal_err_t deinit (dev_bmx_t* bmx) {
  struct bmp280_dev* dev = &_dev;

  if(!bmx) {
    return HAL_ERR_PARAM;
  }

  if(!bmx->initiated) {
    return HAL_ERR_NOT_INITIALIZED;
  }

  bmp280_set_power_mode(BMP280_SLEEP_MODE, dev);
  bmx->initiated = false;
  return 0;
}

/**
 * @brief Lock the instance for operation
 *
 * @param bmx
 *
 * @return HAL_ERR_OK if success
 */
static hal_err_t lock(dev_bmx_t *bmx) {

  if(!bmx || !bmx->lock) {
    return HAL_ERR_PARAM;
  }

  if(!xSemaphoreTake(bmx->lock, LOCK_TIMEOUT)) {
    return HAL_ERR_BUSY;
  }

  return HAL_ERR_OK;
}

/**
 * @brief Unlock the instance
 *
 * @param bmx
 *
 * @return HAL_ERR_OK if success
 */
static hal_err_t unlock(dev_bmx_t *bmx) {

  if(!bmx || !bmx->lock) {
    return HAL_ERR_PARAM;
  }

  if(!xSemaphoreGive(bmx->lock)) {
    return HAL_ERR_UNLIKELY;
  }

  return HAL_ERR_OK;
}

/**
 * @brief Write bytes
 *
 * @param bmx
 *
 * @return HAL_ERR_OK if success
 */
static hal_err_t update(dev_bmx_t* bmx) {
  struct bmp280_dev* dev = &_dev;

  if(!bmx) {
    return HAL_ERR_PARAM;
  }

  if(!bmx->initiated) {
    return HAL_ERR_NOT_INITIALIZED;
  }

  if(bmp280_set_power_mode(BMP280_FORCED_MODE, dev) != BMP280_OK) {
    return HAL_ERR_DENIAL;
  }

  dev->delay_ms(bmp280_compute_meas_time(dev));
  _valid = (bmp280_get_uncomp_data(&_data, dev) == BMP280_OK);
  bmp280_set_power_mode(BMP280_SLEEP_MODE, dev);

  if(!_valid) {
    return HAL_ERR_DATA;
  }

  return HAL_ERR_OK;
}

/**
 * @brief Read data
 *
 * @param bmx
 * @param temperature in 0.1 degree Celsius
 * @param humidity in 1%
 * @param pressure in hPa
 *
 * @return
 */
static hal_err_t read (dev_bmx_t* bmx, int16_t* temperature, uint8_t* humidity, uint16_t* pressure) {
  struct bmp280_dev* dev = &_dev;

  if(!bmx) {
    return HAL_ERR_PARAM;
  }

  if(!bmx->initiated) {
    return HAL_ERR_NOT_INITIALIZED;
  }

  if(temperature) {
    *temperature = bmp280_comp_temp_32bit(_data.uncomp_temp, dev) / 10;
  }
  if(humidity) {
    *humidity = 0;
  }
  if(pressure) {
    *pressure = bmp280_comp_pres_32bit(_data.uncomp_press, dev) / 100;
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
 * @param bmx pointer to the bmx to use
 *
 * @return pointer to the instance if success
 */
dev_bmx_t* dev_bmp280_get_instance(hal_i2c_t* i2c) {
    dev_bmx_t* bmx = &_bmx;
    static const dev_bmx_ops_t ops = {
        .init = init,
        .deinit = deinit,
        .lock = lock,
        .unlock = unlock,
        .update = update,
        .read = read,
    };

    if(!i2c && !bmx->lock) {
        return NULL;
    }

    if(!bmx->lock) {
      bmx->lock = xSemaphoreCreateBinary();
    }

    if(!bmx->lock) {
        return NULL;
    }

    if(i2c) {
      bmx->i2c= i2c;
      bmx->ops = &ops;
    }
    xSemaphoreGive(bmx->lock);

    return bmx;
}


