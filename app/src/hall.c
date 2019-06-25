/*!
 *    @file  hall.c
 *   @brief  The hall mode
 *
 *  @author  Dale.J (dj), Dale.J@zoho.com
 *
 *  @internal
 *       Created:  05/ 5/2016
 *      Revision:  none
 *  Organization:  Druid Tech
 *     Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "timers.h"
/* #include "batt.h" */
/* #include "led.h" */
#include "hall.h"
#include "hal_pin.h"
#include "hal_cfg.h"

/* Includes ------------------------------------------------------------------*/
#define TIMER_OPS_TIMEOUT       2

/* Global variables ----------------------------------------------------------*/
static HALL_Callback_t _callback = NULL;
static TimerHandle_t _timer = NULL;
static int _counter = 0;

/* Private functions ---------------------------------------------------------*/

bool is_hall_active(void) {
  return hal_pin_read(HAL_CFG_HALL_IRQ) == HAL_PIN_LVL_LOW;
}

static void hall_callback(hal_pin_t pin, hal_pin_evt_t evt) {
  xTimerStartFromISR(_timer, NULL);
  _counter = 0;
}

static void timer_callback(TimerHandle_t xTimer) {
  if(_counter++ >= 20) {
    xTimerStop(_timer, TIMER_OPS_TIMEOUT);
    _callback(false);
  } else if(!is_hall_active()) {
    xTimerStop(_timer, TIMER_OPS_TIMEOUT);
    if(_counter > 12) {
      _callback(true);
    }
  }
}
    /* // Battery vlotage is critically low */
    /* if(BATT_GetVoltage() < BATT_SHUT_VOLT) { */
    /*     LED_R_On(); */
    /*     while(is_hall_active()) { */
    /*       vTaskDelay(pdMS_TO_TICKS(200)); */
    /*     } */
    /*     LED_R_Off(); */
    /*     continue; */
    /* } */

/* Global functions ----------------------------------------------------------*/

/**
 * @brief init the hall module
 *
 * @param pvParameters
 */
void HALL_Init(HALL_Callback_t callback) {
  if(!callback) {
    _callback = callback;
  }

  static StaticTimer_t timer_memory = { 0 };
  _timer = xTimerCreateStatic("hall_timer", pdMS_TO_TICKS(250), pdTRUE, NULL, timer_callback, &timer_memory);

  // Init irq
  hal_pin_set_mode_irq(HAL_CFG_HALL_IRQ, HAL_PIN_MODE_IRQ, HAL_PIN_EVT_FALLING, hall_callback);
  hal_pin_irq_enable(HAL_CFG_HALL_IRQ);
}

