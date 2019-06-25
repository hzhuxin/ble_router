/**
 * @brief The temperature sensor abstract layer
 *
 * @file temp.c
 * @date 2018-07-12
 * @author Dengjian
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */


/* Includes ------------------------------------------------------------------*/
#include "nrf_temp.h"
#include "hal_rtc.h"
#ifdef SOFTDEVICE_PRESENT
#include "ble_gap.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"
#endif

/* Defines -------------------------------------------------------------------*/

/* Typedefs ------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static int32_t _temperature = 0;
static bool _valid = false;

/* Private functions ---------------------------------------------------------*/

/* Global functions ----------------------------------------------------------*/

/**
 * @brief Update temperature
 *
 * @return HAL_ERR_SUCCESS if success
 */
hal_err_t hal_temp_update(void) {
#ifdef SOFTDEVICE_PRESENT
  _valid = sd_temp_get(&_temperature) == NRF_SUCCESS;
#else
  nrf_temp_init();

  NRF_TEMP->TASKS_START = 1;
  while (NRF_TEMP->EVENTS_DATARDY == 0) {
  }
  NRF_TEMP->EVENTS_DATARDY = 0;

  _temperature = nrf_temp_read();
  NRF_TEMP->TASKS_STOP = 1;
#endif

  return HAL_ERR_OK;
}

/**
 * @brief Get temperature in degree (float)
 *
 * @return the temperature if success
 */
float hal_temp_get(void) {
  if(_valid) {
    return _temperature * 0.25;
  } else {
    return 0;
  }
}

/**
 * @brief Get temperature in 0.1 degree (int)
 *
 * @return the temperature if success
 */
int hal_temp_get_int(void) {
  if(_valid) {
    return _temperature * 10 / 4;
  } else {
    return 0;
  }
}
