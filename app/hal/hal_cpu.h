/**
 * @brief The cpu abstract layer
 * 
 * @file cpu.h
 * @date 2018-06-22
 * @author Dengjian
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

#ifndef __CPU_H__
#define __CPU_H__

/* Includes ------------------------------------------------------------------*/
#include "hal.h"

/* Typedefs ------------------------------------------------------------------*/
typedef enum hal_cpu_reset_reason {
  HAL_CPU_RESET_REASON_UDF = (1 << 0),  // Undefined reset
  HAL_CPU_RESET_REASON_BOR = (1 << 1),  // Brownout reset
  HAL_CPU_RESET_REASON_PIN = (1 << 2),  // Pin reset
  HAL_CPU_RESET_REASON_POR = (1 << 3),  // Power-on reset
  HAL_CPU_RESET_REASON_SFT = (1 << 4),  // Software reset
  HAL_CPU_RESET_REASON_WDT = (1 << 5),  // Watchdog reset
  HAL_CPU_RESET_REASON_WUR = (1 << 7),  // Wake up reset
  HAL_CPU_RESET_REASON_MAX = INT_MAX,
} hal_cpu_reset_reason_t;

typedef enum hal_cpu_voltage {
  HAL_CPU_VOLTAGE_1V8 = 0,
  HAL_CPU_VOLTAGE_2V1,
  HAL_CPU_VOLTAGE_2V4,
  HAL_CPU_VOLTAGE_2V7,
  HAL_CPU_VOLTAGE_3V0,
  HAL_CPU_VOLTAGE_3V3,
  HAL_CPU_VOLTAGE_MAX,
} hal_cpu_voltage_t;

typedef enum hal_cpu_voltage_regulator { // INNER regulator
  HAL_CPU_VOLTAGE_REGULATOR_LDO = 0,
  HAL_CPU_VOLTAGE_REGULATOR_DCDC,
  HAL_CPU_VOLTAGE_REGULATOR_MAX,
} hal_cpu_voltage_regulator_t;

/* Defines -------------------------------------------------------------------*/
#define HAL_CPU_MAC_LEN         BLE_GAP_ADDR_LEN
#define HAL_CPU_ID_LEN          BLE_GAP_ADDR_LEN

/* Global variables ----------------------------------------------------------*/

/* Global functions ----------------------------------------------------------*/

void hal_cpu_reset(void);
uint8_t* hal_cpu_get_id(void);
uint8_t* hal_cpu_get_mac(void);
char* hal_cpu_get_id_str(void);
char* hal_cpu_get_mac_str(void);
hal_cpu_reset_reason_t hal_cpu_get_reset_reason(void);
hal_err_t hal_cpu_set_voltage_mode(hal_cpu_voltage_regulator_t r, hal_cpu_voltage_t v);

#endif // #ifndef __CPU_H__

