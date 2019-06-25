/**
 * @brief The watchdog abstract layer
 *
 * @file wdt.c
 * @date 2018-06-23
 * @author Dengjian
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "nrfx_wdt.h"
#include "hal_wdt.h"

/* Defines -------------------------------------------------------------------*/

/* Typedefs ------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static void dummy_callback(void) {
}

/* Global functions ----------------------------------------------------------*/

/**
 * @brief Watchdog init
 *
 * @param cfg
 * @param cb
 *
 * @return HAL_ERR_OK if success
 */
hal_err_t hal_wdt_init(hal_wdt_cfg_t* cfg, hal_wdt_cb_t cb) {

  nrfx_wdt_config_t nrf_cfg = NRFX_WDT_DEAFULT_CONFIG;
  nrf_cfg.reload_value = cfg->timeout;
  if(cfg->stop_when_sleep && cfg->stop_when_debug) {
    nrf_cfg.behaviour = NRF_WDT_BEHAVIOUR_PAUSE_SLEEP_HALT;
  } else if(!cfg->stop_when_sleep && !cfg->stop_when_debug) {
    nrf_cfg.behaviour = NRF_WDT_BEHAVIOUR_RUN_SLEEP_HALT;
  } else if(cfg->stop_when_sleep) {
    nrf_cfg.behaviour = NRF_WDT_BEHAVIOUR_RUN_HALT;
  } else if(cfg->stop_when_debug) {
    nrf_cfg.behaviour = NRF_WDT_BEHAVIOUR_RUN_SLEEP;
  }

  if(nrfx_wdt_init(&nrf_cfg, cb ? cb : dummy_callback) != NRF_SUCCESS) {
    return HAL_ERR_UNLIKELY;
  }
  nrfx_wdt_channel_id channel;
  if(nrfx_wdt_channel_alloc(&channel) != NRF_SUCCESS) {
    return HAL_ERR_UNLIKELY;
  }
  nrfx_wdt_enable();
  nrfx_wdt_feed();

  return HAL_ERR_OK;
}

/**
 * @brief Watchdog feed
 *
 * @return HAL_ERR_OK
 */
hal_err_t hal_wdt_feed(void) {
  nrfx_wdt_feed();

  return HAL_ERR_OK;
}


