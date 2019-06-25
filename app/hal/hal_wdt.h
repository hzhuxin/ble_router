/**
 * @brief The watchdog abstract layer
 * 
 * @file wdt.h
 * @date 2018-06-23
 * @author Dengjian
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

#ifndef __WDT_H__
#define __WDT_H__

/* Includes ------------------------------------------------------------------*/
#include "hal.h"

/* Typedefs ------------------------------------------------------------------*/
typedef struct hal_wdt_cfg {
  int32_t timeout;
  int32_t stop_when_debug: 1;
  int32_t stop_when_sleep: 1;
  int32_t stop_when_deepsleep: 1;
} hal_wdt_cfg_t;

typedef void (*hal_wdt_cb_t) (void);

/* Defines -------------------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/

/* Global functions ----------------------------------------------------------*/

hal_err_t hal_wdt_init(hal_wdt_cfg_t* cfg, hal_wdt_cb_t cb);
hal_err_t hal_wdt_feed(void);

#endif // #ifndef __WDT_H__

