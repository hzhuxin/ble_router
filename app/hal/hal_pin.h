/**
 * @brief The pin abstract layer
 * 
 * @file pin.h
 * @date 2018-06-22
 * @author Dengjian
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

#ifndef __PIN_H__
#define __PIN_H__

/* Includes ------------------------------------------------------------------*/
#include "hal.h"

/* Defines -------------------------------------------------------------------*/
#define HAL_PIN_NOT_USED      (-1)
#define HAL_PIN_ADC_VDD       (-2)
#define HAL_PIN_ADC_REF       (-3)
#define HAL_PIN_ADC_TEMP      (-4)
#define HAL_PIN_ADC_VDDHDIV5  (-5)

/* Typedefs ------------------------------------------------------------------*/

typedef int hal_pin_t;

typedef enum hal_pin_mode {
    HAL_PIN_MODE_DEFAULT = 0,
    HAL_PIN_MODE_DISCONNECTED = 1,
    HAL_PIN_MODE_IN,
    HAL_PIN_MODE_IN_PULL_UP,
    HAL_PIN_MODE_IN_PULL_DOWN,
    HAL_PIN_MODE_OUT,
    HAL_PIN_MODE_OUT_OPEN_DRAIN,
    HAL_PIN_MODE_OUT_OPEN_SOURCE,
    HAL_PIN_MODE_IRQ,
    HAL_PIN_MODE_IRQ_PULL_UP,
    HAL_PIN_MODE_IRQ_PULL_DOWN,
    HAL_PIN_MODE_MAX,
} hal_pin_mode_t;

typedef enum hal_pin_mode_ex {
    HAL_PIN_MODE_EX_NORMAL = 0,
    HAL_PIN_MODE_EX_HIGH,
    HAL_PIN_MODE_EX_LOW,
    HAL_PIN_MODE_EX_MAX,
} hal_pin_mode_ex_t;

typedef enum hal_pin_evt {
    HAL_PIN_EVT_RISING = 0,
    HAL_PIN_EVT_FALLING,
    HAL_PIN_EVT_TOGGLE,
    HAL_PIN_EVT_MAX,
} hal_pin_evt_t;

typedef enum hal_pin_lvl {
    HAL_PIN_LVL_LOW = 0,
    HAL_PIN_LVL_HIGH,
    HAL_PIN_LVL_MAX,
} hal_pin_lvl_t;

typedef void (*hal_pin_evt_cb_t) (hal_pin_t pin, hal_pin_evt_t evt);

/* Global variables ----------------------------------------------------------*/


/* Function prototypes -------------------------------------------------------*/

hal_err_t hal_pin_set_mode(hal_pin_t pin, hal_pin_mode_t mode);
hal_err_t hal_pin_set_mode_ex(hal_pin_t pin, hal_pin_mode_t mode, hal_pin_mode_ex_t ex, hal_pin_lvl_t lvl);
hal_err_t hal_pin_set_mode_irq(hal_pin_t pin, hal_pin_mode_t mode, hal_pin_evt_t evt, hal_pin_evt_cb_t cb);
hal_err_t hal_pin_irq_enable(hal_pin_t pin);
hal_err_t hal_pin_irq_disable(hal_pin_t pin);
hal_err_t hal_pin_write(hal_pin_t pin, hal_pin_lvl_t lvl);
hal_err_t hal_pin_toggle(hal_pin_t pin);
hal_pin_lvl_t hal_pin_read(hal_pin_t pin);

#endif // #ifndef __PIN_H__

