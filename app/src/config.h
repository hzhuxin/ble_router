/*!
 *    @file  Config.h
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

#ifndef __CONFIG_H__
#define __CONFIG_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Typedefs ------------------------------------------------------------------*/
enum {
  CFG_SAMP_MODE_PAUSE = 0,
  CFG_SAMP_MODE_NORMAL = 1,
  CFG_SAMP_MODE_OFF = 2,
};

typedef struct {
  uint32_t version;
  char token[32];
  char host[64];
  uint32_t port;
  uint32_t force;
} OAD_CfgDef;

typedef struct {
  uint8_t checksum;
  uint8_t version;
  uint16_t length;

  uint8_t  resetMode;
  uint8_t  warningMode;
  uint8_t  powerOffMode;

  uint8_t  envMode;
  uint8_t  bhvMode;
  uint8_t  gpsMode;
  uint8_t  netMode;
  uint8_t  smsMode;

  uint32_t envIntv;
  uint32_t bhvIntv;
  uint32_t gpsIntv;
  uint32_t netIntv;
  uint32_t smsIntv;

  uint32_t powerOffTime;
  uint32_t powerOnTime;
} CFG_TypeDef;

/* Function prototypes -------------------------------------------------------*/
bool CFG_Init(void);
bool CFG_Save(void);
bool CFG_Erase(void);
CFG_TypeDef* CFG_Get(void);
bool CFG_Set(CFG_TypeDef *pCfg);
bool CFG_isChanged(CFG_TypeDef *pCfg);

#endif // #ifndef __CONFIG_H__
