/*!
 *    @file  bmx.c
 *   @brief  The bmx module based on bme280
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
#include "bmx/BME280_driver/bme280.h"

/* Defines -------------------------------------------------------------------*/

/* Typedefs ------------------------------------------------------------------*/

/* bmxate variables ---------------------------------------------------------*/
static const int LOCK_TIMEOUT = pdMS_TO_TICKS(2000);
static dev_bmx_t _bmx = { 0 };
static bool _valid = false;
static struct bme280_dev _dev;
static struct bme280_data _data;

/* Private functions ---------------------------------------------------------*/

static void user_delay_ms(uint32_t period) {
  vTaskDelay(pdMS_TO_TICKS(period + 2));
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
  struct bme280_dev* dev = &_dev;
  dev->dev_id = BME280_I2C_ADDR_PRIM;
  dev->intf = BME280_I2C_INTF;
  dev->read = user_i2c_read;
  dev->write = user_i2c_write;
  dev->delay_ms = user_delay_ms;
  if(bme280_init(dev) != BME280_OK) {
    return HAL_ERR_DENIAL;
  }

  uint8_t settings_sel;
  dev->settings.osr_h = BME280_OVERSAMPLING_1X;
  dev->settings.osr_p = BME280_OVERSAMPLING_8X;
  dev->settings.osr_t = BME280_OVERSAMPLING_4X;
  dev->settings.filter = BME280_FILTER_COEFF_OFF;
  settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

  if(bme280_set_sensor_settings(settings_sel, dev) != BME280_OK) {
    return HAL_ERR_DENIAL;
  }

  if(bme280_set_sensor_mode(BME280_SLEEP_MODE, dev) != BME280_OK) {
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
  struct bme280_dev* dev = &_dev;

  if(!bmx) {
    return HAL_ERR_PARAM;
  }

  if(!bmx->initiated) {
    return HAL_ERR_NOT_INITIALIZED;
  }

  bme280_set_sensor_mode(BME280_SLEEP_MODE, dev);
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
  struct bme280_dev* dev = &_dev;

  if(!bmx) {
    return HAL_ERR_PARAM;
  }

  if(!bmx->initiated) {
    return HAL_ERR_NOT_INITIALIZED;
  }

  if(bme280_set_sensor_mode(BME280_FORCED_MODE, dev) != BME280_OK) {
    return HAL_ERR_DENIAL;
  }
  dev->delay_ms(10);

  _valid = (bme280_get_sensor_data(BME280_ALL, &_data, dev) == BME280_OK);
  bme280_set_sensor_mode(BME280_SLEEP_MODE, dev);

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

  if(!bmx) {
    return HAL_ERR_PARAM;
  }

  if(!bmx->initiated) {
    return HAL_ERR_NOT_INITIALIZED;
  }

  if(temperature) {
    *temperature = _data.temperature / 10;
  }
  if(humidity) {
    *humidity = _data.humidity >> 10;
  }
  if(pressure) {
    *pressure = _data.pressure / 10000;
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
dev_bmx_t* dev_bme280_get_instance(hal_i2c_t* i2c) {
    dev_bmx_t* bmx = &_bmx;
    static const dev_bmx_ops_t ops = {
        .init = init,
        .deinit = deinit,
        .lock = lock,
        .unlock = unlock,
        .update = update,
        .read = read,
    };

    if(!i2c) {
        return NULL;
    }

    if(!bmx->lock) {
      bmx->lock = xSemaphoreCreateBinary();
    }

    if(!bmx->lock) {
        return NULL;
    }

    xSemaphoreTake(bmx->lock, 0);
    {
      bmx->i2c= i2c;
      bmx->ops = &ops;
    }
    xSemaphoreGive(bmx->lock);

    return bmx;
}


