/**
 * @brief The rtc abstract layer
 * 
 * @file rtc.h
 * @date 2018-06-22
 * @author Dengjian
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

#ifndef __RTC_H__
#define __RTC_H__

/* Includes ------------------------------------------------------------------*/
#include "hal.h"

/* Typedefs ------------------------------------------------------------------*/
typedef void (*hal_rtc_alarm_cb_t) (void);

/* Defines -------------------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/

/* Global functions ----------------------------------------------------------*/

hal_err_t hal_rtc_init(void);
hal_err_t hal_rtc_set_time(uint32_t time);
uint32_t hal_rtc_get_time(void);
uint32_t hal_rtc_get_uptime(void);  // Time from the last reboot, in seconds
uint32_t hal_rtc_get_boot_time(void);  // Timestamp at the last reboot, in seconds
hal_err_t hal_rtc_set_alarm(uint32_t timestamp, hal_rtc_alarm_cb_t cb);  // Absolute timestamp in seconds
hal_err_t hal_rtc_save(void);
#endif // #ifndef __RTC_H__

