/*!
 *    @file  Option.h
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

#ifndef __OPTION_H__
#define __OPTION_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Typedefs ------------------------------------------------------------------*/
typedef struct {
  uint8_t  checksum;
  uint8_t  version;
  uint16_t length;

  uint16_t envThreshold;
  uint16_t bhvThreshold;
  uint16_t gpsThreshold;
  uint16_t netThreshold;

  uint8_t  serverHosts[3][64];
  uint32_t serverPorts[3];

  uint8_t  smsReceivers[6][24];
  uint8_t  smsOperators[6][24];
} OPT_TypeDef;

/* Function prototypes -------------------------------------------------------*/
bool OPT_Init(void);
bool OPT_Save(void);
bool OPT_Erase(void);
OPT_TypeDef* OPT_Get(void);
bool OPT_Set(OPT_TypeDef *pOPT);
bool OPT_isChanged(OPT_TypeDef *pOpt);

#endif // #ifndef __OPTION_H__
