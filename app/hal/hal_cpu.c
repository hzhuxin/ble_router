/**
 * @brief The cpu abstract layer
 *
 * @file cpu.c
 * @date 2018-06-22
 * @author Dengjian
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "nrfx_power.h"
#include "nrfx_wdt.h"
#include "nrfx_log.h"
#include "hal_cpu.h"
#ifdef SOFTDEVICE_PRESENT
#include "ble_gap.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"
#endif

/* Defines -------------------------------------------------------------------*/

/* Typedefs ------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static hal_cpu_reset_reason_t _reset_reason = HAL_CPU_RESET_REASON_MAX;

/* Private functions ---------------------------------------------------------*/

/* Global functions ----------------------------------------------------------*/

/**
 * @brief Software reset
 */
void hal_cpu_reset(void) {
#ifdef SOFTDEVICE_PRESENT
  sd_nvic_SystemReset();
#else
  NVIC_SystemReset();
#endif
}

/**
 * @brief Get cpu mac
 *
 * @return pointer to cpu mac
 */
uint8_t* hal_cpu_get_mac(void) {
  static uint8_t mac[HAL_CPU_MAC_LEN] = { 0 };
#ifdef SOFTDEVICE_PRESENT
  if(mac[0] == 0) {
    uint8_t sd_enabled = false;
    if(sd_softdevice_is_enabled(&sd_enabled) == NRF_SUCCESS && sd_enabled) {
      ble_gap_addr_t gap_addr;
      if(sd_ble_gap_addr_get(&gap_addr) == NRF_SUCCESS) {
        for(int i = 0; i < sizeof(mac); i++) {
          mac[sizeof(mac) - 1 - i] = gap_addr.addr[i];
        }
      }
    }
  }
#else
  #error "not supported for now"
#endif
  return mac;
}

/**
 * @brief Get cpu mac string
 *
 * @return cpu mac string
 */
char* hal_cpu_get_mac_str(void) {
  static char mac_str[HAL_CPU_MAC_LEN * 3] = { 0 };

  if(mac_str[0] == 0) {
    uint8_t* mac = hal_cpu_get_mac();
    if(mac[0] != 0 || mac[HAL_CPU_MAC_LEN - 1] != 0) {
      for(int i = 0; i < HAL_CPU_MAC_LEN; i++) {
        sprintf(&mac_str[i * 3], "%02x:", mac[i]);
      }
      mac_str[sizeof(mac_str) - 1] = '\0';
    }
  }

  return mac_str;
}

/**
 * @brief Get cpu id
 *
 * @return pointer to cpu id
 */
uint8_t* hal_cpu_get_id(void) {
  return hal_cpu_get_mac();
}

/**
 * @brief Get cpu id string
 *
 * @return cpu id string
 */
char* hal_cpu_get_id_str(void) {
  static char id_str[HAL_CPU_MAC_LEN * 2 + 1] = { 0 };

  if(id_str[0] == 0) {
    uint8_t* id = hal_cpu_get_id();
    if(id[0] != 0 || id[HAL_CPU_ID_LEN - 1] != 0) {
      for(int i = 0; i < HAL_CPU_ID_LEN; i++) {
        sprintf(&id_str[i * 2], "%02x", id[i]);
      }
      id_str[sizeof(id_str) - 1] = '\0';
    }
  }

  return id_str;
}

/**
 * @brief Get reset reason
 *
 * @return reset reason
 */
hal_cpu_reset_reason_t hal_cpu_get_reset_reason(void) {
  if(_reset_reason >= HAL_CPU_RESET_REASON_MAX) {
    uint32_t rr = 0;
#ifdef SOFTDEVICE_PRESENT
    uint8_t sd_enabled = false;
    if(sd_softdevice_is_enabled(&sd_enabled) == NRF_SUCCESS && sd_enabled) {
      // Read reason
      sd_power_reset_reason_get(&rr);
      // Clear reason
      sd_power_reset_reason_clr(rr);
    } else {
      // Read reason
      rr = NRF_POWER->RESETREAS;
      // Clear reason
      NRF_POWER->RESETREAS = rr;
    }
#else
    // Read reason
    rr = NRF_POWER->RESETREAS;
    // Clear reason
    NRF_POWER->RESETREAS = rr;
#endif

    NRF_LOG_INFO("NRF Reset reason: 0x%x", rr);

    // Bugfix for chip
    if(rr & POWER_RESETREAS_RESETPIN_Msk) {
      rr = POWER_RESETREAS_RESETPIN_Msk;
    }
    // Covert reason
    if(rr == 0) {
      _reset_reason = HAL_CPU_RESET_REASON_POR | HAL_CPU_RESET_REASON_BOR;
    } else {
      _reset_reason = 0;
      if(rr & NRF_POWER_RESETREAS_RESETPIN_MASK) {
        _reset_reason |= HAL_CPU_RESET_REASON_PIN;
      }
      if(rr & NRF_POWER_RESETREAS_DOG_MASK) {
        _reset_reason |= HAL_CPU_RESET_REASON_WDT;
      }
      if(rr & NRF_POWER_RESETREAS_SREQ_MASK) {
        _reset_reason |= HAL_CPU_RESET_REASON_SFT;
      }
      if(rr & NRF_POWER_RESETREAS_LOCKUP_MASK) {
        _reset_reason |= HAL_CPU_RESET_REASON_UDF;
      }
      if(rr & NRF_POWER_RESETREAS_OFF_MASK) {
        _reset_reason |= HAL_CPU_RESET_REASON_WUR;
      }
    }
    NRF_LOG_INFO("Reset reason: 0x%x", _reset_reason);
  }

  return _reset_reason;
}

hal_err_t hal_cpu_set_voltage_mode(hal_cpu_voltage_regulator_t regulator, hal_cpu_voltage_t voltage) {

#define NORMAL_VOLTAGE_MODE 0
#define HIGH_VOLTAGE_MODE 1

  if(regulator >= HAL_CPU_VOLTAGE_REGULATOR_MAX || voltage >= HAL_CPU_VOLTAGE_MAX) {
    return HAL_ERR_PARAM;
  }

  uint8_t mode = NORMAL_VOLTAGE_MODE;

#if defined(NRF52840_XXAA) // 52840 ONLY
  mode = NRF_POWER->MAINREGSTATUS;
  if(mode == HIGH_VOLTAGE_MODE) {
    if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) ==
        (UICR_REGOUT0_VOUT_DEFAULT << UICR_REGOUT0_VOUT_Pos)) {
      NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
      while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

      NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
                          (voltage << UICR_REGOUT0_VOUT_Pos);

      NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
      while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

      // System reset is needed to update UICR registers.
      NVIC_SystemReset();
    }

  if(regulator == HAL_CPU_VOLTAGE_REGULATOR_DCDC) { //enable DCDC
    NRF_POWER->DCDCEN0 = 0x01;
  } else {
    NRF_POWER->DCDCEN0 = 0x00;
  }
} else {  // normal voltage mode in 52840
      // nothing to do
}
#endif
  // Boath for 52840 and 52832 in normal voltage mode
  UNUSED_PARAMETER(mode);
  if(regulator == HAL_CPU_VOLTAGE_REGULATOR_DCDC) {
      NRF_POWER->DCDCEN = 0x01;
  } else {
      NRF_POWER->DCDCEN = 0x00;
  }

#undef NORMAL_VOLTAGE_MODE
#undef HIGHT_VOLTAGE_MODE

  return HAL_ERR_OK;
}

