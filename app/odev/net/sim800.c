/*!
 *    @file  SIM800.c
 *   @brief  CELL module based on SIM800c
 *
 *  @author  Dale.J (dj), Dale.J@zoho.com
 *
 *  @internal
 *       Created:  04/10/2016
 *      Revision:  none
 *  Organization:  Druid Tech
 *     Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */


/* Includes ------------------------------------------------------------------*/
#include "cell.c"
#include "hal_cfg.h"
#include "hal_pin.h"

/* Defines -------------------------------------------------------------------*/
#define MAC_TIMEOUT   pdMS_TO_TICKS(1000 * 5)

/* Private variables ---------------------------------------------------------*/
static char MAC[CELLULAR_MAC_LENGTH + 1] = {0};

/* Global functions ----------------------------------------------------------*/

#if 0
/**
 * @brief Get the status pin
 *
 * @return the status
 */
bool CELL_GetStatusPin(void) {
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = GSM_STA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GSM_STA_GPIO_Port, &GPIO_InitStruct);

  bool ret = HAL_GPIO_ReadPin(GSM_STA_GPIO_Port, GSM_STA_Pin);

  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GSM_STA_GPIO_Port, &GPIO_InitStruct);

  return ret;
}

/**
 * @brief Read MAC
 *
 * @param pMAC
 *
 * @return true if success
 */
bool CELL_ReadMAC(uint8_t* pMAC) {
  DBG_D("CELL: Reading mac...\r\n");

  bool succeed = false;
  TickType_t start = xTaskGetTickCount();
  while(!Util_timeout(start, xTaskGetTickCount(), MAC_TIMEOUT)) {
    if(cellular_sim800_bt_mac(modem, (char*)MAC, sizeof(MAC)) == 0) {
      succeed = true;
      if(pMAC != NULL) {
        memcpy(pMAC, MAC, sizeof(MAC));
      }
      break;
    } else {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  if(succeed) {
    DBG_I("CELL: read mac succeed: %s\r\n", MAC);
  } else {
    DBG_E("CELL: read mac failed\r\n");
  }

  return succeed;
}

/**
 * @brief Get the MAC string
 *
 * @return pointer to MAC
 */
static char* CELL_GetMac(void) {
  if(MAC[0] != '\0' || CELL_ReadMAC(NULL)) {
    return MAC;
  } else {
    return NULL;
  }
}
#endif

/**
 * @brief Start tx test at the max power, which is 900Mhz, power level 5
 *
 * @return true if success
 */
bool CELL_TxStart(void) {
  DBG_D("CELL: Starting tx...\r\n");

  bool succeed = (modem->ops->command(modem, "AT+CTBURST=1, 1, 124, 5", 1) != NULL);

  if(succeed) {
    DBG_I("CELL: Start tx succeed\r\n");
  } else {
    DBG_W("CELL: Start tx failed\r\n");
  }

  return succeed;
}

/**
 * @brief Stop tx test
 *
 * @return true if success
 */
bool CELL_TxStop(void) {
  DBG_D("CELL: Stopping tx...\r\n");

  bool succeed = (modem->ops->command(modem, "AT+CTBURST=0, 1, 124, 5", 1) != NULL);

  if(succeed) {
    DBG_I("CELL: Stop tx succeed\r\n");
  } else {
    DBG_W("CELL: Stop tx failed\r\n");
  }

  return succeed;
}

/**
 * @brief Power on
 *
 * @return true if success
 */
bool CELL_PowerOn(void) {
  DBG_D("CELL: Powering on\r\n");
  // Power on
  hal_pin_set_mode_ex(HAL_CFG_CELL_PWR, HAL_PIN_MODE_OUT, HAL_PIN_MODE_EX_NORMAL, 1);
  vTaskDelay(pdMS_TO_TICKS(800));
  // Power key paulse
  hal_pin_set_mode_ex(HAL_CFG_CELL_KEY, HAL_PIN_MODE_OUT_OPEN_DRAIN, HAL_PIN_MODE_EX_NORMAL, 0);
  vTaskDelay(pdMS_TO_TICKS(1200));
  // Release power key
  hal_pin_set_mode(HAL_CFG_CELL_KEY, HAL_PIN_MODE_DEFAULT);
  vTaskDelay(pdMS_TO_TICKS(2000));

  // Variables init
  MAC[0] = 0;

  DBG_I("CELL: Power on succeed\r\n");
  return true;
}

/**
 * @brief Power off
 *
 * @return true if success
 */
bool CELL_PowerOff(void) {
  DBG_D("CELL: Powering off\r\n");
  // Power off
  hal_pin_set_mode_ex(HAL_CFG_CELL_PWR, HAL_PIN_MODE_OUT, HAL_PIN_MODE_EX_NORMAL, 0);
  vTaskDelay(pdMS_TO_TICKS(200));

  DBG_I("CELL: Power off succeed\r\n");
  return true;
}
