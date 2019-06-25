/**
 * @brief The rtc abstract layer
 *
 * @file rtc.c
 * @date 2018-06-22
 * @author Dengjian
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */


/* Includes ------------------------------------------------------------------*/
#include "nrfx_rtc.h"
#include "hal_rtc.h"

/* Defines -------------------------------------------------------------------*/
#define RTC_INST_IDX             2

/* Typedefs ------------------------------------------------------------------*/
typedef struct rtc_backup {
  uint32_t time;
  uint32_t check;
} rtc_backup_t;

/* Private variables ---------------------------------------------------------*/
#ifdef NRF52840_XXAA
static rtc_backup_t* _backup = (rtc_backup_t*) 0x2003fff8;
#else
static rtc_backup_t* _backup = (rtc_backup_t*) 0x2000fff8;
#endif

static nrfx_rtc_t _rtc = NRFX_RTC_INSTANCE(RTC_INST_IDX);
static volatile uint32_t _base = 0; // Base time in seconds
static volatile uint32_t _overflow = 0; // Number of overflows

/* Private functions ---------------------------------------------------------*/
static void rtc_callback(nrfx_rtc_int_type_t evt) {
  if(evt == NRFX_RTC_INT_OVERFLOW) {
    _overflow++;
  }
}

/* Global functions ----------------------------------------------------------*/

/**
 * @brief Init rtc
 *
 * @return HAL_ERR_SUCCESS if success
 */
hal_err_t hal_rtc_init(void) {
  nrfx_rtc_config_t cfg = NRFX_RTC_DEFAULT_CONFIG;
  cfg.prescaler = RTC_FREQ_TO_PRESCALER(8);

  if(nrfx_rtc_init(&_rtc, &cfg, rtc_callback) != NRFX_SUCCESS) {
    return HAL_ERR_DENIAL;
  }

  // Recover from nonvolatile ram
  if(_backup->time == ~_backup->check) {
    _base = _backup->time;
  }

  nrfx_rtc_overflow_enable(&_rtc, true);
  nrfx_rtc_enable(&_rtc);

  return HAL_ERR_OK;
}

/**
 * @brief Set time
 *
 * @param time
 *
 * @return HAL_ERR_OK if success
 */
hal_err_t hal_rtc_set_time(uint32_t time) {
  _base = time - hal_rtc_get_uptime();

  return HAL_ERR_OK;
}

/**
 * @brief Get the timestamp at the last reboot
 *
 * @return timestamp in seconds
 */
uint32_t hal_rtc_get_boot_time(void) {
  return _base;
}

/**
 * @brief Get time
 *
 * @return time in seconds
 */
uint32_t hal_rtc_get_time(void) {
  uint32_t uptime = 0;
  uint32_t base = _base;
  do {
    uptime = hal_rtc_get_uptime();
    base = _base;
  } while(base != _base);

  // Backup in nonvolatile ram
  _backup->time = base + uptime;
  _backup->check = ~_backup->time;

  return base + uptime;
}

/**
 * @brief Get time from the last reboot
 *
 * @return uptime in seconds
 */
uint32_t hal_rtc_get_uptime(void) {
  uint32_t counter = 0;
  uint32_t overflow = _overflow;
  do {
    counter = nrfx_rtc_counter_get(&_rtc);
    overflow = _overflow;
  } while(overflow != _overflow);

  return (overflow << 21) + (counter >> 3);
}
