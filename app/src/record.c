/*!
 *    @file  Record.c
 *   @brief  The record module
 *
 *  @author  Dale.J (dj), Dale.J@zoho.com
 *
 *  @internal
 *       Created:  05/ 2/2016
 *      Revision:  none
 *  Organization:  Druid Tech
 *     Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */


/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "partition.h"
#include "record.h"
#include "gps.h"
#include "cell.h"
/* #include "hw.h" */
#include "debug.h"
/* #include "loader.h" */
#include "behavior.h"
#include "w25q.h"
#include "hal_cfg.h"

/* Defines -------------------------------------------------------------------*/
#ifdef LOG_ENABLE
#define ENV_BASE_SECOTR   10
#define ENV_NUM_SECTORS   80

#define GPS_BASE_SECOTR   (ENV_BASE_SECOTR + ENV_NUM_SECTORS) // TODO: fixme
#define GPS_NUM_SECTORS   100

#define ACT_BASE_SECOTR   (GPS_BASE_SECOTR + GPS_NUM_SECTORS)
#define ACT_NUM_SECTORS   100

#define NET_BASE_SECOTR   (ACT_BASE_SECOTR + ACT_NUM_SECTORS)
#define NET_NUM_SECTORS   10

#define LOG_BASE_SECOTR   (NET_BASE_SECOTR + NET_NUM_SECTORS)
#define LOG_NUM_SECTORS   200

#define ERR_BASE_SECOTR   (LOG_BASE_SECOTR + LOG_NUM_SECTORS)
#define ERR_NUM_SECTORS   10
#else
#define ENV_BASE_SECOTR   10
#define ENV_NUM_SECTORS   75

#define GPS_BASE_SECOTR   (ENV_BASE_SECOTR + ENV_NUM_SECTORS)
#define GPS_NUM_SECTORS   100

#define ACT_BASE_SECOTR   (GPS_BASE_SECOTR + GPS_NUM_SECTORS)
#define ACT_NUM_SECTORS   305

#define NET_BASE_SECOTR   (ACT_BASE_SECOTR + ACT_NUM_SECTORS)
#define NET_NUM_SECTORS   10

#define ERR_BASE_SECOTR   (NET_BASE_SECOTR + NET_NUM_SECTORS)
#define ERR_NUM_SECTORS   10
#endif

/* Private variables ---------------------------------------------------------*/
static Part_Index_t envIdx = {
  .read = 0,
  .write = 0,
  .count = 0,
  .sequence = 0xffffffff,
};

static Part_Info_t envPart = {
  .base = ENV_BASE_SECOTR * W25Q_SECTOR_SIZE,
  .itemSize = sizeof(Record_EnvData),
  .capacity = (ENV_NUM_SECTORS - 1) * W25Q_SECTOR_SIZE / sizeof(Record_EnvData),
  .index = &envIdx,
  .lock = NULL,
};

static Part_Index_t gpsIdx = {
  .read = 0,
  .write = 0,
  .count = 0,
  .sequence = 0xffffffff,
};

static Part_Info_t gpsPart = {
  .base = GPS_BASE_SECOTR * W25Q_SECTOR_SIZE,
  .itemSize = sizeof(Record_GpsData),
  .capacity = (GPS_NUM_SECTORS - 1) * W25Q_SECTOR_SIZE / sizeof(Record_EnvData),
  .index = &gpsIdx,
  .lock = NULL,
};

static Part_Index_t actIdx = {
  .read = 0,
  .write = 0,
  .count = 0,
  .sequence = 0xffffffff,
};

static Part_Info_t actPart = {
  .base = ACT_BASE_SECOTR * W25Q_SECTOR_SIZE,
  .itemSize = sizeof(Record_ActData),
  .capacity = (ACT_NUM_SECTORS - 1) * W25Q_SECTOR_SIZE / sizeof(Record_ActData),
  .index = &actIdx,
  .lock = NULL,
};

static Part_Index_t netIdx = {
  .read = 0,
  .write = 0,
  .count = 0,
  .sequence = 0xffffffff,
};

static Part_Info_t netPart = {
  .base = NET_BASE_SECOTR * W25Q_SECTOR_SIZE,
  .itemSize = sizeof(CellData_t),
  .capacity = (NET_NUM_SECTORS - 1) * W25Q_SECTOR_SIZE / sizeof(CellData_t),
  .index = &netIdx,
  .lock = NULL,
};

#ifdef LOG_ENABLE
static Part_Index_t logIdx = {
  .read = 0,
  .write = 0,
  .count = 0,
  .sequence = 0xffffffff,
};

static Part_Info_t logPart = {
  .base = LOG_BASE_SECOTR * W25Q_SECTOR_SIZE,
  .itemSize = sizeof(char),
  .capacity = (LOG_NUM_SECTORS - 1) * W25Q_SECTOR_SIZE / sizeof(char),
  .index = &logIdx,
  .lock = NULL,
};
#endif

static Part_Index_t errIdx = {
  .read = 0,
  .write = 0,
  .count = 0,
  .sequence = 0xffffffff,
};

static Part_Info_t errPart = {
  .base = ERR_BASE_SECOTR * W25Q_SECTOR_SIZE,
  .itemSize = sizeof(Record_ErrData),
  .capacity = (ERR_NUM_SECTORS - 1) * W25Q_SECTOR_SIZE / sizeof(Record_ErrData),
  .index = &errIdx,
  .lock = NULL,
};

static Part_Info_t* partTable[] = {
#ifdef LOG_ENABLE
  &envPart, &gpsPart, &actPart, &netPart, &logPart, &errPart,
#else
  &envPart, &gpsPart, &actPart, &netPart, NULL, &errPart,
#endif
};

/* Global functions ----------------------------------------------------------*/

/**
 * @brief Acquire env sample
 *
 * @param pEnv
 * @param error
 * @param timeout in seconds
 *
 * @return true if success
 */
bool Record_Sample(Record_EnvData *pEnv, uint32_t* error, uint32_t timeout) {
  // Clean record
  memset(pEnv, 0, sizeof(Record_EnvData));

  /* // Read sensor */
  /* ALS_Read(&pEnv->luminance); */
  /* pEnv->voltage = BATT_GetVoltage(); */
  /* BMX280_Read(&pEnv->pressure, &pEnv->temperature, &pEnv->humidity); */
  /* // GPS fix */
  /* GPS_FixData fix; */
  bool fixed = false;
  /* TickType_t duration = 0; */
  /* if(pEnv->temperature > -300) { */
  /*   GPS_PowerOn(); */
  /*   TickType_t start = RTC_GetSeconds(); */
  /*   fixed = GPS_Fix(&fix, pdMS_TO_TICKS(1000 * timeout), pdMS_TO_TICKS(1000 * 90)); */
  /*   duration = RTC_GetSeconds() - start; */
  /*   GPS_PowerOff(); */
  /* } */

  /* pEnv->fixtime = duration > UINT8_MAX ? UINT8_MAX : duration; */
  /* if(fixed) { */
  /*   RTC_Time_t *pt = (RTC_Time_t*)&fix; */
  /*   RTC_MakeTime(pt); */

  /*   pEnv->time = pt->time; */
  /*   pEnv->longitude = fix.lon;          // 10e-7 deg */
  /*   pEnv->latitude = fix.lat;           // 10e-7 deg */
  /*   pEnv->altidude = fix.height / 10;   // 0.01m */

  /*   pEnv->dimention = fix.fixType; */
  /*   pEnv->heading = fix.heading / 10000;// 0.1 deg */
  /*   pEnv->hAccuracy = fix.hAcc / 100;   // 0.1m */
  /*   pEnv->vAccuracy = fix.vAcc / 100;   // 0.1m */
  /*   pEnv->usedStar = fix.uStar; */
  /*   pEnv->viewStar = fix.vStar; */
  /*   pEnv->speed = fix.gSpeed / 100;     // 0.1m/s */
  /* } else { */
  /*   pEnv->latitude = 2000000000; */
  /*   pEnv->longitude = 2000000000; */
  /* } */

  return fixed;
}

/**
 * @brief Write an error record
 *
 * @param time
 * @param type
 * @param data
 *
 * @return true if success
 */
bool Record_Error(uint32_t time, uint32_t type, uint32_t data) {
  Record_ErrData err;
  err.time = time;
  err.version = SW_VERSION;
  err.type = type;
  err.data = data;

  return Record_Write(RECORD_TYPE_ERR, (uint8_t*)&err);
}

/**
 * @brief Get the size of a data item
 *
 * @param type
 *
 * @return size in byte
 */
uint32_t Record_Size(uint32_t type) {
  uint16_t ret = 0;

  switch(type) {
    case RECORD_TYPE_ENV:
      ret = sizeof(Record_EnvData);
      break;
    case RECORD_TYPE_GPS:
      ret = sizeof(Record_GpsData);
      break;
    case RECORD_TYPE_ACT:
      ret = sizeof(Record_ActData);
      break;
    case RECORD_TYPE_NET:
      ret = sizeof(CellData_t);
      break;
    case RECORD_TYPE_LOG:
      ret = sizeof(char);
      break;
    case RECORD_TYPE_ERR:
      ret = sizeof(Record_ErrData);
      break;
    default:
      break;
  }
  return ret;
}

/**
 * @brief Count record
 *
 * @param type
 *
 * @return number of items
 */
uint32_t Record_Count(uint32_t type) {
  return Part_Count(partTable[type]);
}

//uint8_t buf[1024];
//void Record_Browse(uint32_t type) {
//  const Part_Info_t *part = partTable[type];
//  uint32_t address = part->base;
//  uint32_t batch = sizeof(buf) / part->itemSize * part->itemSize;

//  if(!W25Q_Lock()) {
//    return;
//  }

//  for(int i = 0; i < 4; i++) {
//    W25Q_Read(address, sizeof(buf), buf);
//    address += sizeof(buf);
//  }
//
//  for(int i = 0; i < 160; i++) {
//    W25Q_Read(address, batch, buf);
//    address += batch;
//  }
//  W25Q_Unlock();
//}

/**
 * @brief Read record
 *
 * @param type
 * @param pOut
 * @param outBufSize
 *
 * @return number of items read
 */
uint32_t Record_Read(uint32_t type, uint8_t* pOut, uint32_t outBufSize) {
  return Part_Read(partTable[type], pOut, outBufSize);
}

/**
 * @brief Get record
 *
 * @param type
 * @param offset
 * @param pOut
 *
 * @return true if success
 */
bool Record_Get(uint32_t type, uint32_t offset, uint8_t* pOut) {
  return Part_Get(partTable[type], offset, pOut);
}

/**
 * @brief Peek record
 *
 * @param type
 * @param pOut
 * @param outBufSize
 *
 * @return number of items read
 */
uint32_t Record_Peek(uint32_t type, uint8_t* pOut, uint32_t outBufSize) {
  return Part_Peek(partTable[type], pOut, outBufSize);
}

/**
 * @brief Write a record
 *
 * @param type
 * @param pData
 *
 * @return true if success
 */
bool Record_Write(uint32_t type, uint8_t* pData) {
  return Part_Write(partTable[type], pData);
}

/**
 * @brief Write records
 *
 * @param type
 * @param pData
 * @param nbItems
 *
 * @return true if success
 */
uint32_t Record_Writes(uint32_t type, uint8_t* pData, uint32_t nbItems) {
  return Part_Writes(partTable[type], pData, nbItems);
}

/**
 * @brief Delete record
 *
 * @param type
 * @param nbItems
 *
 * @return true if success
 */
bool Record_Delete(uint32_t type, uint32_t nbItems) {
  return Part_Delete(partTable[type], nbItems);
}

/**
 * @brief Erase all records
 *
 * @return true if success
 */
bool Record_Erase(void) {
  for(int type = 0; type < RECORD_TYPE_MAX; type++) {
    Part_Erase(partTable[type]);
  }
  return true;
}
