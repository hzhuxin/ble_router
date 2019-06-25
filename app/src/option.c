/*!
 *    @file  Option.c
 *   @brief  The option module
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
#include "option.h"
#include "debug.h"
#include "util.h"
#include "w25q.h"
#include "app_cfg.h"

/* Defines -------------------------------------------------------------------*/
DBG_SET_LEVEL(DBG_LEVEL_D);

#define OPT_SECTOR_NUM    6
#define OPT_SECTOR_ADDR   (OPT_SECTOR_NUM * W25Q_SECTOR_SIZE)

/* Private variables ---------------------------------------------------------*/
static const OPT_TypeDef dftOpt = {
  .checksum = 0xff,
  .version = 0x01,
  .length = sizeof(OPT_TypeDef),

  .envThreshold = 3650,
  .bhvThreshold = 3650,
  .gpsThreshold = 3720,
  .netThreshold = 3720,

  .serverHosts = {
    { DEFAULT_HOST },
  },
  .serverPorts = {
    DEFAULT_PORT,
  },

  .smsReceivers = {
    { 0 }
  },
  .smsOperators = {
    { 0 }
  },
};

static OPT_TypeDef xOpt = { 0 };
static OPT_TypeDef *pxOpt= NULL;

/* Global functions ----------------------------------------------------------*/
/**
 * @brief Load config form flash
 *
 * @return true if necessary
 */
bool OPT_Init(void) {
  int len = 0;
  // Read from flash
  if(!W25Q_Lock()) {
    return false;
  }
  len = W25Q_Read(OPT_SECTOR_ADDR, sizeof(OPT_TypeDef), (uint8_t*)&xOpt);
  W25Q_Unlock();
  if(len != sizeof(OPT_TypeDef)) {
    DBG_E("Read option from flash failed\r\n");
  }
  // Restore default if check failed
  bool valid = false;
  if(xOpt.length == sizeof(OPT_TypeDef)) {
    if(util_checksum((uint8_t*)&xOpt, 1, xOpt.length - 1) == xOpt.checksum) {
      valid = true;
    }
  }
  if(!valid) {
    DBG_W("Check option failed\r\n");
    memcpy(&xOpt, &dftOpt, dftOpt.length);
    xOpt.checksum = util_checksum((uint8_t*)&xOpt, 1, xOpt.length - 1);
  }
  pxOpt = &xOpt;
  // Return
  return true;
}

/**
 * @brief Get config
 *
 * @return true if success
 */
OPT_TypeDef* OPT_Get(void) {
  if(pxOpt == NULL) {
    OPT_Init();
  }
  return pxOpt;
}

/**
 * @brief Check if option is changed
 *
 * @param pOpt
 *
 * @return true if changed
 */
bool OPT_isChanged(OPT_TypeDef *pOpt) {
  // Calculate checksum
  pOpt->checksum = util_checksum((uint8_t*)pOpt, 1, sizeof(OPT_TypeDef) - 1);
  // Compare the new option
  return memcmp(&xOpt, pOpt, sizeof(OPT_TypeDef));
}

/**
 * @brief Strore config if necessary
 *
 * @param pOpt
 *
 * @return true if success
 */
bool OPT_Set(OPT_TypeDef *pOpt) {
  // Check
  if(!OPT_isChanged(pOpt)) {
    return true;
  }
  // Copy
  memcpy(&xOpt, pOpt, sizeof(OPT_TypeDef));
  // Save
  if(!W25Q_Lock()) {
    return false;
  }
  if(W25Q_Write(OPT_SECTOR_ADDR, sizeof(OPT_TypeDef), (uint8_t*)&xOpt) != sizeof(OPT_TypeDef)) {
    DBG_E("Write opion to flash failed\r\n");
  } else {
    DBG_I("Write opion to flash succeed (%d, %d, %d, %d, %s, %d)\r\n",
        pOpt->envThreshold,
        pOpt->bhvThreshold,
        pOpt->gpsThreshold,
        pOpt->netThreshold,
        pOpt->serverHosts[0],
        pOpt->serverPorts[0]
        );
  }
  W25Q_Unlock();
  return true;
}

/**
 * @brief Strore config
 *
 * @param pOpt
 *
 * @return true if success
 */
bool OPT_Save(void) {
  // Calculate checksum
  pxOpt->checksum = util_checksum((uint8_t*)pxOpt, 1, sizeof(OPT_TypeDef) - 1);
  // Save
  if(!W25Q_Lock()) {
    return false;
  }
  W25Q_Write(OPT_SECTOR_ADDR, sizeof(OPT_TypeDef), (uint8_t*)pxOpt);
  W25Q_Unlock();
  return true;
}

/**
 * @brief Erase config
 *
 * @return true if success
 */
bool OPT_Erase(void) {
  if(!W25Q_Lock()) {
    return false;
  }
  W25Q_Erase(OPT_SECTOR_ADDR, W25Q_SECTOR_SIZE);
  W25Q_Unlock();
  return true;
}
