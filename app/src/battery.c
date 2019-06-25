/*!
 *    @file  BATT.c
 *   @brief  Battery module
 *
 *  @author  Dale.J (dj), Dale.J@zoho.com
 *
 *  @internal
 *       Created:  04/ 7/2016
 *      Revision:  none
 *  Organization:  Druid Tech
 *     Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "battery.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "debug.h"
#include "hal_rtc.h"
#include "hal_adc.h"

/* Defines -------------------------------------------------------------------*/
#define BATT_UPDATE_INTERVAL      (60 * 10)
#define LOCK_TIMEOUT              (pdMS_TO_TICKS(20))
DBG_SET_LEVEL(DBG_LEVEL_D);

/* Typedefs ------------------------------------------------------------------*/
typedef struct {
  uint16_t buf[5];
  uint16_t cnt;
} Filter_t;

/* Private variables ---------------------------------------------------------*/
static SemaphoreHandle_t _lock = NULL;
static TickType_t updateTime = 0;
/* static uint16_t buf[20] = { 0 }; */
/* static bool finished = false; */
static uint16_t voltage = 0;
static Filter_t filter = { 0 };

/* Private functions ---------------------------------------------------------*/
static uint16_t UpdateFilter(Filter_t* filter, uint16_t data) {
  int capacity = sizeof(filter->buf) /sizeof(filter->buf[0]);
  if(filter->cnt > capacity) {
    filter->cnt = 0;
  }
  if(filter->cnt == capacity) {
    memcpy(filter->buf, &filter->buf[1], sizeof(filter->buf) - sizeof(filter->buf[0]));
    filter->buf[capacity - 1] = data;
  } else {
    filter->buf[filter->cnt++] = data;
  }
  if(filter->cnt >= 3) {
    int16_t min = 0x7fff;
    int16_t max = 0;
    int32_t sum = 0;
    for(int i = 0; i < filter->cnt; i++) {
      sum += filter->buf[i];
      if(min > filter->buf[i]) {
        min = filter->buf[i];
      }
      if(max < filter->buf[i]) {
        max = filter->buf[i];
      }
    }
    sum -= min + max;
    sum /= filter->cnt - 2;
    return sum;
  } else {
    return filter->buf[filter->cnt - 1];
  }
}

static bool lock(void) {
  if(!_lock) {
    static StaticSemaphore_t lock_memory = { 0 };
    _lock = xSemaphoreCreateRecursiveMutexStatic(&lock_memory);
    if(!_lock) {
      return false;
    } else {
      return true;
    }
  }

  return xSemaphoreTakeRecursive(_lock, LOCK_TIMEOUT);
}

static bool unlock(void) {
  return xSemaphoreGiveRecursive(_lock);
}

// Init ADC
static void AdcInit(void)
{
  // static bool initiated = false;
  // if(!initiated) {
  //   initiated = true;
    hal_adc_cfg_t adc_cfg = {
      .resolution = 12,
      .oversample = 32,
    };
    hal_adc_channel_t ch_cfg[] = {
      {
        .pin_p = 4,
        .pin_n = HAL_PIN_NOT_USED,
        .gain = 1 / 6.0,
        .acqtime = 40,
        .reference = HAL_ADC_REFERENCE_INT,
      }
    };
    hal_adc_init(&adc_cfg, ch_cfg, sizeof(ch_cfg) / sizeof(*ch_cfg));
  // }
}
// Deinit ADC
static void AdcDeinit(void) {
  hal_adc_deinit();
}
// Sample ADC
static bool AdcSample(void) {
  AdcInit();

  hal_adc_update();
  int16_t v = 0;
  hal_adc_get_sample(0, &v);
  voltage = UpdateFilter(&filter, (v * 3600UL * 3) >> 12);

  AdcDeinit();
  return true;
}
/* Global functions ----------------------------------------------------------*/
/**
 * @brief Test the hardware connection
 *
 * @param pVoltage
 *
 * @return true if successTT_ReadVoltage()
 */
bool BATT_HardwareTest(uint16_t *pVoltage) {
  if(!lock()) {
    return false;
  }

  bool ret = false;
  if(AdcSample()) {
    if(voltage > 3000 && voltage < 4300) {
      if(pVoltage != NULL) {
        *pVoltage = voltage;
      }
      ret = true;
    }
  }

  unlock();
  return ret;
}

/**
 * @brief Update the battery voltage
 *
 * @return true if success
 */
bool BATT_UpdateVoltage(void) {
  if(!lock()) {
    return false;
  }

  bool ret = false;
  updateTime = hal_rtc_get_time();
  if(AdcSample()) {
    ret = true;
  }
  DBG_D("Voltage updated: %d mV\r\n", voltage);

  unlock();
  return ret;
}

/**
 * @brief Get the battery voltage in mV
 *
 * @return the battery voltage
 */
uint16_t BATT_GetVoltage(void) {
  if(!lock()) {
    return 0;
  }

  uint32_t time = hal_rtc_get_time();
  uint32_t delta = (time >= updateTime) ? time - updateTime : updateTime - time;
  if(delta > BATT_UPDATE_INTERVAL) {
    BATT_UpdateVoltage();
  }

  unlock();
  return voltage;
}

