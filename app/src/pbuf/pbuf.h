/*!
 *    @file  pbuf.h
 *   @brief  The pbuf module
 *
 *  @author  Dale.J (dj), Dale.J@zoho.com
 *
 *  @internal
 *       Created:  04/18/2016
 *      Revision:  none
 *  Organization:  Druid Tech
 *     Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */

#ifndef __PBUF_H__
#define __PBUF_H__

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "Download.pb-c.h"
#include "cell.h"
/* Defines -------------------------------------------------------------------*/
struct PB_Package;

#define PB_MAX_PKG_SIZE     768
#define PB_HEADER_SIZE      12
#define PB_MAX_PDU_SIZE     (PB_MAX_PKG_SIZE) - (PB_HEADER_SIZE)
#define PB_MAX_SMS_SIZE     160 / 4 * 3 - (PB_HEADER_SIZE)

#define PB_BT_CONN_ID       1000

/* Typedefs ------------------------------------------------------------------*/
typedef struct {
  uint16_t version;
  uint16_t manufacture;
  uint16_t type;
  uint16_t len;
  uint16_t dummy;
  uint16_t crc;
  uint8_t data[PB_MAX_PDU_SIZE];
} PB_PackageTypeDef;

typedef struct {
  uint16_t protocol;
  uint16_t dummy;
  uint32_t type;
  uint32_t hver;
  uint8_t uid[12];
  uint32_t sn;
  uint16_t crc;
  uint16_t chk;
} PB_InfoTypeDef;

enum {
  PB_ERR_OK = 0,
  PB_ERR_CONN = -1,
  PB_ERR_TIMEOUT = -2,
  PB_ERR_PACKAGE = -3,
  PB_ERR_PARAM = -4,
  PB_ERR_DATA = -5,
  PB_ERR_SMS = -6,
};

/* Function prototypes -------------------------------------------------------*/
ProtobufCAllocator* PB_Allocator(void);
int PB_Receive(int connid, int timeout);
int PB_Command(int connid, int timeout, PB_PackageTypeDef* req);
int PB_Register(int connid, int timeout, int32_t status, int32_t voltage, CellData_t *cell, int32_t* pSN);
int PB_Setting(int connid, int timeout, char* oadToken);
int PB_Param(int connid, int timeout);
int PB_Gps(int connid, int timeout);
int PB_Env(int connid, int timeout);
int PB_Bhv(int connid, int timeout);
int PB_Cellular(int connid, int timeout);
int PB_Status(int connid, int timeout);
int PB_SMS(int rssi);
int PB_Debug(int connid, int timeout, char* msg);
int PB_App(int connid, int timeout, int32_t* linkType, int32_t* workMode) ;
int PB_Confirm(int connid, int timeout);
bool PB_Factory(int timeout, int32_t status, int32_t voltage);
bool PB_Info(int connid, int sn);
int PB_Download(int connid, int timeout, char* token, uint32_t offset, uint32_t length, Protocol__DownloadRsp** presp);
int PB_Log(int connid, int timeout, bool *more);
#endif // #ifndef __PBUF_H__
