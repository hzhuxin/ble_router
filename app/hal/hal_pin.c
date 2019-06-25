/**
 * @brief The pin abstract layer
 *
 * @file pin.c
 * @date 2018-06-22
 * @author Dengjian
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "hal.h"
#include "hal_pin.h"
#include "nrf_gpio.h"
#include "nrfx_gpiote.h"
#include "nrf_log.h"

/* Defines -------------------------------------------------------------------*/

/* Typedefs ------------------------------------------------------------------*/
typedef struct pin_evt_cb {
  hal_pin_t pin;
  hal_pin_evt_cb_t cb;
} pin_evt_cb_t;

/* Private variables ---------------------------------------------------------*/
static pin_evt_cb_t evt_cb_tab[GPIOTE_CH_NUM] = { 0 };

/* Private functions ---------------------------------------------------------*/
static bool set_evt_cb(hal_pin_t pin, hal_pin_evt_cb_t cb) {
  pin_evt_cb_t* p;

  for(p = evt_cb_tab; p < evt_cb_tab + GPIOTE_CH_NUM; p++) {
    if(p->pin == pin) {
      p->cb = cb;
      return true;
    }
  }

  for(p = evt_cb_tab; p < evt_cb_tab + GPIOTE_CH_NUM; p++) {
    if(p->cb == NULL) {
      p->pin = pin;
      p->cb = cb;
      return true;
    }
  }

  return false;
}

static hal_pin_evt_cb_t get_evt_cb(hal_pin_t pin) {
  pin_evt_cb_t* p;

  for(p = evt_cb_tab; p < evt_cb_tab + GPIOTE_CH_NUM; p++) {
    if(p->pin == pin) {
      return p->cb;
    }
  }

  return NULL;
}

static void pin_evt_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  hal_pin_evt_t evt = HAL_PIN_EVT_MAX;
  hal_pin_evt_cb_t cb = NULL;

  switch(action) {
    case NRF_GPIOTE_POLARITY_LOTOHI:
      evt = HAL_PIN_EVT_RISING;
      break;
    case NRF_GPIOTE_POLARITY_HITOLO:
      evt = HAL_PIN_EVT_FALLING;
      break;
    case NRF_GPIOTE_POLARITY_TOGGLE:
      evt = HAL_PIN_EVT_TOGGLE;
      break;
    default:
      break;
  }

  cb = get_evt_cb(pin);
  if(cb) {
    cb(pin, evt);
  }
}

/* Global variables ----------------------------------------------------------*/

/* Global functions ----------------------------------------------------------*/

hal_err_t hal_pin_set_mode(hal_pin_t pin, hal_pin_mode_t mode) {
  nrf_gpio_pin_dir_t dir = NRF_GPIO_PIN_DIR_INPUT;
  nrf_gpio_pin_input_t input = NRF_GPIO_PIN_INPUT_DISCONNECT;
  nrf_gpio_pin_pull_t pull = NRF_GPIO_PIN_NOPULL;
  nrf_gpio_pin_drive_t drive = NRF_GPIO_PIN_S0S1;
  nrf_gpio_pin_sense_t sense = NRF_GPIO_PIN_NOSENSE;

  switch(mode) {
    case HAL_PIN_MODE_DEFAULT:
    case HAL_PIN_MODE_DISCONNECTED:
      break;
    case HAL_PIN_MODE_IN:
      input = NRF_GPIO_PIN_INPUT_CONNECT;
      break;
    case HAL_PIN_MODE_IN_PULL_UP:
      input = NRF_GPIO_PIN_INPUT_CONNECT;
      pull = NRF_GPIO_PIN_PULLUP;
      break;
    case HAL_PIN_MODE_IN_PULL_DOWN:
      input = NRF_GPIO_PIN_INPUT_CONNECT;
      pull = NRF_GPIO_PIN_PULLDOWN;
      break;
    case HAL_PIN_MODE_OUT:
      dir = NRF_GPIO_PIN_DIR_OUTPUT;
      break;
    case HAL_PIN_MODE_OUT_OPEN_DRAIN:
      dir = NRF_GPIO_PIN_DIR_OUTPUT;
      drive = NRF_GPIO_PIN_S0D1;
      break;
    case HAL_PIN_MODE_OUT_OPEN_SOURCE:
      dir = NRF_GPIO_PIN_DIR_OUTPUT;
      drive = NRF_GPIO_PIN_D0S1;
      break;
    case HAL_PIN_MODE_IRQ:
      input = NRF_GPIO_PIN_INPUT_CONNECT;
      break;
    case HAL_PIN_MODE_IRQ_PULL_UP:
      input = NRF_GPIO_PIN_INPUT_CONNECT;
      pull = NRF_GPIO_PIN_PULLUP;
      break;
    case HAL_PIN_MODE_IRQ_PULL_DOWN:
      input = NRF_GPIO_PIN_INPUT_CONNECT;
      pull = NRF_GPIO_PIN_PULLDOWN;
      break;
    default:
      break;
  }

  nrf_gpio_cfg(pin, dir, input, pull, drive, sense);
  return HAL_ERR_OK;
}

hal_err_t hal_pin_set_mode_ex(hal_pin_t pin, hal_pin_mode_t mode, hal_pin_mode_ex_t ex, hal_pin_lvl_t lvl) {
  nrf_gpio_pin_dir_t dir = NRF_GPIO_PIN_DIR_INPUT;
  nrf_gpio_pin_input_t input = NRF_GPIO_PIN_INPUT_DISCONNECT;
  nrf_gpio_pin_pull_t pull = NRF_GPIO_PIN_NOPULL;
  nrf_gpio_pin_drive_t drive = NRF_GPIO_PIN_S0S1;
  nrf_gpio_pin_sense_t sense = NRF_GPIO_PIN_NOSENSE;

  switch(mode) {
    case HAL_PIN_MODE_DEFAULT:
    case HAL_PIN_MODE_DISCONNECTED:
      break;
    case HAL_PIN_MODE_IN:
      input = NRF_GPIO_PIN_INPUT_CONNECT;
      break;
    case HAL_PIN_MODE_IN_PULL_UP:
      input = NRF_GPIO_PIN_INPUT_CONNECT;
      pull = NRF_GPIO_PIN_PULLUP;
      break;
    case HAL_PIN_MODE_IN_PULL_DOWN:
      input = NRF_GPIO_PIN_INPUT_CONNECT;
      pull = NRF_GPIO_PIN_PULLDOWN;
      break;
    case HAL_PIN_MODE_OUT:
      dir = NRF_GPIO_PIN_DIR_OUTPUT;
      drive = (ex == HAL_PIN_MODE_EX_HIGH) ? NRF_GPIO_PIN_H0H1 : NRF_GPIO_PIN_S0S1;
      break;
    case HAL_PIN_MODE_OUT_OPEN_DRAIN:
      dir = NRF_GPIO_PIN_DIR_OUTPUT;
      drive = (ex == HAL_PIN_MODE_EX_HIGH) ? NRF_GPIO_PIN_H0D1 : NRF_GPIO_PIN_S0D1;
      break;
    case HAL_PIN_MODE_OUT_OPEN_SOURCE:
      dir = NRF_GPIO_PIN_DIR_OUTPUT;
      drive = (ex == HAL_PIN_MODE_EX_HIGH) ? NRF_GPIO_PIN_D0H1 : NRF_GPIO_PIN_D0S1;
      break;
    case HAL_PIN_MODE_IRQ:
      input = NRF_GPIO_PIN_INPUT_CONNECT;
      break;
    case HAL_PIN_MODE_IRQ_PULL_UP:
      input = NRF_GPIO_PIN_INPUT_CONNECT;
      pull = NRF_GPIO_PIN_PULLUP;
      break;
    case HAL_PIN_MODE_IRQ_PULL_DOWN:
      input = NRF_GPIO_PIN_INPUT_CONNECT;
      pull = NRF_GPIO_PIN_PULLDOWN;
      break;
    default:
      break;
  }

  nrf_gpio_cfg(pin, dir, input, pull, drive, sense);
  nrf_gpio_pin_write(pin, lvl);
  return HAL_ERR_OK;
}

hal_err_t hal_pin_set_mode_irq(hal_pin_t pin, hal_pin_mode_t mode, hal_pin_evt_t evt, hal_pin_evt_cb_t cb) {
  if(!nrfx_gpiote_is_init()) {
    nrfx_gpiote_init();
  }

  nrfx_gpiote_in_config_t cfg = {
    .sense = NRF_GPIO_PIN_NOSENSE,
    .pull = NRF_GPIO_PIN_NOPULL,
    .is_watcher = false,
    .hi_accuracy = false,
  };

  if(mode == HAL_PIN_MODE_IRQ_PULL_UP) {
    cfg.pull = NRF_GPIO_PIN_PULLUP;
  } else if(mode == HAL_PIN_MODE_IRQ_PULL_DOWN) {
    cfg.pull = NRF_GPIO_PIN_PULLDOWN;
  } else if(mode == HAL_PIN_MODE_IRQ) {
    cfg.pull = NRF_GPIO_PIN_NOPULL;
  } else {
    return HAL_ERR_PARAM;
  }

  if(evt == HAL_PIN_EVT_RISING) {
    cfg.sense = NRF_GPIOTE_POLARITY_LOTOHI;
  } else if(evt == HAL_PIN_EVT_FALLING) {
    cfg.sense = NRF_GPIOTE_POLARITY_HITOLO;
  } else if(evt == HAL_PIN_EVT_TOGGLE) {
    cfg.sense = NRF_GPIOTE_POLARITY_TOGGLE;
  } else {
    return HAL_ERR_PARAM;
  }

  if(nrfx_gpiote_in_init(pin, &cfg, pin_evt_handler) != NRF_SUCCESS) {
    return HAL_ERR_DENIAL;
  }

  if(!set_evt_cb(pin, cb)) {
    return HAL_ERR_MEMORY;
  }
  return HAL_ERR_OK;
}

hal_err_t hal_pin_irq_enable(hal_pin_t pin) {
  nrfx_gpiote_in_event_enable(pin, true);
  return HAL_ERR_OK;
}

hal_err_t hal_pin_irq_disable(hal_pin_t pin) {
  nrfx_gpiote_in_event_enable(pin, false);
  return HAL_ERR_OK;
}

hal_err_t hal_pin_write(hal_pin_t pin, hal_pin_lvl_t lvl) {
  nrf_gpio_pin_write(pin, lvl);
  return HAL_ERR_OK;
}

hal_err_t hal_pin_toggle(hal_pin_t pin) {
  nrf_gpio_pin_toggle(pin);
  return HAL_ERR_OK;
}

hal_pin_lvl_t hal_pin_read(hal_pin_t pin) {
  return nrf_gpio_pin_read(pin);
}

