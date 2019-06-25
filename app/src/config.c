/*!
 *    @file  Config.c
 *   @brief  The config module
 *
 *  @author  Dale.J (dj), Dale.J@zoho.com
 *
 *  @internal
 *       Created:  04/29/2016
 *      Revision:  none
 *  Organization:  Druid Tech
 *     Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "config.h"
#include "debug.h"
#include "util.h"
#include "w25q.h"

/* Defines -------------------------------------------------------------------*/
DBG_SET_LEVEL(DBG_LEVEL_D);

#define CFG_SECTOR_NUM    2
#define CFG_SECTOR_ADDR   (CFG_SECTOR_NUM * W25Q_SECTOR_SIZE)

/* Private variables ---------------------------------------------------------*/
static const CFG_TypeDef dftCfg = {
  .checksum = 0xff,
  .version = 0x01,
  .length = sizeof(CFG_TypeDef),

  .envMode = 1,
  .bhvMode = 1,
  .gpsMode = 1,
  .netMode = 1,
  .smsMode = 0,

  .envIntv = 600,
  .bhvIntv = 600,
  .gpsIntv = 3600,
  .netIntv = 7200,
  .smsIntv = 86400,

  .powerOffMode = 0,
  .powerOffTime = 0,
  .powerOnTime = 0,

  .warningMode = 0,
  .resetMode = 0,
};

static CFG_TypeDef xCfg;
static CFG_TypeDef *pxCfg = NULL;

/* Global functions ----------------------------------------------------------*/
/**
 * @brief Load config form flash
 *
 * @return true if necessary
 */
bool CFG_Init(void) {
  int len = 0;
  // Read from flash
  if(!W25Q_Lock()) {
    return false;
  }
  len = W25Q_Read(CFG_SECTOR_ADDR, sizeof(CFG_TypeDef), (uint8_t*)&xCfg);
  W25Q_Unlock();
  if(len != sizeof(CFG_TypeDef)) {
    DBG_E("Read config from flash failed\r\n");
  }
  // Restore default if check failed
  bool valid = false;
  if(xCfg.length == sizeof(CFG_TypeDef)) {
    if(util_checksum((uint8_t*)&xCfg, 1, xCfg.length - 1) == xCfg.checksum) {
      valid = true;
    }
  }
  if(!valid) {
    DBG_W("Check config failed\r\n");
    memcpy(&xCfg, &dftCfg, dftCfg.length);
    xCfg.checksum = util_checksum((uint8_t*)&xCfg, 1, xCfg.length - 1);
  }
  pxCfg = &xCfg;
  // Return
  return true;
}

/**
 * @brief Get config
 *
 * @return true if success
 */
CFG_TypeDef* CFG_Get(void) {
  if(pxCfg == NULL) {
    CFG_Init();
  }
  return pxCfg;
}

/**
 * @brief Check if setting is changed
 *
 * @param pCfg
 *
 * @return true if changed
 */
bool CFG_isChanged(CFG_TypeDef *pCfg) {
  // Calculate checksum
  pCfg->checksum = util_checksum((uint8_t*)pCfg, 1, sizeof(CFG_TypeDef) - 1);
  // Compare the new config
  return memcmp(&xCfg, pCfg, sizeof(CFG_TypeDef));
}

/**
 * @brief Strore config if necessary
 *
 * @param pCfg
 *
 * @return true if success
 */
bool CFG_Set(CFG_TypeDef *pCfg) {
  // Check
  if(!CFG_isChanged(pCfg)) {
    return true;
  }
  // Copy
  memcpy(&xCfg, pCfg, sizeof(CFG_TypeDef));
  // Save
  if(!W25Q_Lock()) {
    return false;
  }
  if(W25Q_Write(CFG_SECTOR_ADDR, sizeof(CFG_TypeDef), (uint8_t*)&xCfg) != sizeof(CFG_TypeDef)) {
    DBG_E("Write config to flash failed\r\n");
  } else {
    DBG_I("Write config to flash succeed (%d, %d, %d, %d)\r\n",
          pCfg->envMode == 1 ? pCfg->envIntv : 0,
          pCfg->bhvMode == 1 ? pCfg->bhvIntv : 0,
          pCfg->gpsMode == 1 ? pCfg->gpsIntv : 0,
          pCfg->netMode == 1 ? pCfg->netIntv : 0
        );
  }
  W25Q_Unlock();
  return true;
}

/**
 * @brief Strore config
 *
 * @param pCfg
 *
 * @return true if success
 */
bool CFG_Save(void) {
  // Calculate checksum
  pxCfg->checksum = util_checksum((uint8_t*)pxCfg, 1, sizeof(CFG_TypeDef) - 1);
  // Save
  if(!W25Q_Lock()) {
    return false;
  }
  W25Q_Write(CFG_SECTOR_ADDR, sizeof(CFG_TypeDef), (uint8_t*)pxCfg);
  W25Q_Unlock();
  return true;
}

/**
 * @brief Erase config
 *
 * @return true if success
 */
bool CFG_Erase(void) {
  if(!W25Q_Lock()) {
    return false;
  }
  W25Q_Erase(CFG_SECTOR_ADDR, W25Q_SECTOR_SIZE);
  W25Q_Unlock();
  return true;
}
