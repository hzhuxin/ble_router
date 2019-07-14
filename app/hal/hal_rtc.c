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
#include "nrf_fstorage.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
/* Defines -------------------------------------------------------------------*/
#define RTC_INST_IDX             2
#define RTC_MEMORY_START          (0xA0000)
#define RTC_MEMORY_SIZE          (4096)

DBG_SET_LEVEL(DBG_LEVEL_I);
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
static nrf_fstorage_t _fstorage;
/* Private functions ---------------------------------------------------------*/
static void rtc_callback(nrfx_rtc_int_type_t evt) {
  if(evt == NRFX_RTC_INT_OVERFLOW) {
    _overflow++;
  }
}

/* Global functions ----------------------------------------------------------*/
static uint8_t fstorage_done_flag = 0;
static xSemaphoreHandle sema = NULL;
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
  xSemaphoreGiveFromISR(sema, NULL);
}

void rtc_memory_init(void)
{
  if(!sema)
  {
    sema = xSemaphoreCreateBinary();
    xSemaphoreTake(sema,0);
  }
  extern nrf_fstorage_api_t nrf_fstorage_sd;
  nrf_fstorage_t *fs= &_fstorage;
  fs->evt_handler = fstorage_evt_handler;
  fs->start_addr = RTC_MEMORY_START;
  fs->end_addr   = fs->start_addr + RTC_MEMORY_SIZE; // 4k
  nrf_fstorage_init(fs, &nrf_fstorage_sd, fstorage_evt_handler);

}
int32_t rtc_read_memory(void)
{
  int32_t utc;
  nrf_fstorage_t *fs= &_fstorage;
  nrf_fstorage_read(fs, RTC_MEMORY_START, &utc, 4);
  return utc;
}

hal_err_t hal_rtc_save(void)
{
  uint32_t utc = hal_rtc_get_time();
  ret_code_t ret;
  nrf_fstorage_t *fs= &_fstorage;
  xSemaphoreTake(sema,0);
  ret = nrf_fstorage_erase(fs, RTC_MEMORY_START, 1, NULL);
  if(ret != NRF_SUCCESS)
  {
    DBG_E("Save RTC of erase failed, return %d",ret);
    return -ret;
  }
  if(!xSemaphoreTake(sema,pdMS_TO_TICKS(5000)))
  {
    DBG_E("Save RTC of erase timeout 5S");
    return -1;
  }

  xSemaphoreTake(sema,0);
  ret = nrf_fstorage_write(fs, RTC_MEMORY_START, &utc, 4, NULL);
  if(ret != NRF_SUCCESS)
  {
    DBG_E("Save RTC of write failed return %d",ret);
    return -ret;
  }
  if(!xSemaphoreTake(sema,pdMS_TO_TICKS(5000)))
  {
    DBG_E("Save RTC of write timeout 5S");
    return -2;
  }
  uint32_t temp;
  ret = nrf_fstorage_read(fs, RTC_MEMORY_START, &temp, 4);
  if(ret != NRF_SUCCESS)
  {
    DBG_E("Save RTC of read failed return %d",ret);
    return -ret;
  }
  if(utc != temp)
  {
    DBG_E("Save RTC failed, write<%d>, read<%d>",utc, temp);
    return -3;
  }
  DBG_I("Save RTC<%d> OK",utc);
  return 0;
}
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

  int32_t utc;
  rtc_memory_init();
  utc = rtc_read_memory();
  DBG_I("--------Hal RTC read %d(0x%x)------\r\n",utc,utc);
  if(utc > 0)
  {
    hal_rtc_set_time(utc);
  }
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
  // if(time == hal_rtc_get_time())
  //   return HAL_ERR_OK;

  _base = time - hal_rtc_get_uptime();
  hal_rtc_save();
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
