/*!
 *    @file  Record.h
 *   @brief  The record module
 *
 *  @author  Dale.J (dj), Dale.J@zoho.com
 *
 *  @internal
 *       Created:  04/20/2016
 *      Revision:  none
 *  Organization:  Druid Tech
 *     Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */

#ifndef __RECORD_H__
#define __RECORD_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Typedefs ------------------------------------------------------------------*/
// Runtime error type
typedef enum {
  RT_ERR_BATTERY = 0,
  RT_ERR_CHIP = 1,
  RT_ERR_LOW_G = 2,
  RT_ERR_HIGH_G = 3,
  RT_ERR_ENV_FULL = 4,
  RT_ERR_ACT_FULL = 5,
  RT_ERR_ENV_OVERRUN = 6,
  RT_ERR_ACT_OVERRUN = 7,
  RT_ERR_LOW_P = 8,
  RT_ERR_HIGH_P = 9,
  RT_ERR_AUX_FULL = 10,
  RT_ERR_AUX_OVERRUN = 11,
  RT_ERR_CELL_SMS = 12,
  RT_ERR_CELL_CSQ = 13,
  RT_ERR_CELL_CREG = 14,
  RT_ERR_CELL_CGATT = 15,
  RT_ERR_CELL_APN = 16,
  RT_ERR_CELL_CIICR = 17,
  RT_ERR_CELL_CIPSTART = 18,
  RT_ERR_CELL_OTHER = 19,
  RT_ERR_CELL_COPS = 20,
  RT_ERR_CELL_COMMAND = 21,
  RT_ERR_CELL_HW = 22,
  RT_ERR_RESET = 128,
  RT_ERR_HEAP = 129,
  RT_ERR_STACK = 130,
  RT_ERR_ASSERT = 131,
} Record_RuntimeError;

enum {
  RECORD_TYPE_ENV = 0,
  RECORD_TYPE_GPS,
  RECORD_TYPE_ACT,
  RECORD_TYPE_NET,
  RECORD_TYPE_LOG,
  RECORD_TYPE_ERR,
  RECORD_TYPE_SN,
  RECORD_TYPE_MAX,
};

// Env data item (40B)
typedef struct {
  uint32_t time;
  int32_t  longitude;
  int32_t  latitude;
  int32_t  altidude;
  uint16_t hAccuracy;
  uint16_t vAccuracy;
  uint16_t speed;
  uint16_t heading;
  uint8_t  dimention;
  uint8_t  viewStar;
  uint8_t  usedStar;
  uint8_t  rssi;
  uint8_t  fixtime;
} Record_GpsData;
// Env data item (16B)
typedef struct {
  uint32_t time;
  uint16_t voltage; // mV
  int16_t  temperature; // 0.1℃
  uint32_t light; // lx
  uint16_t pressure; // hpa
  uint8_t  power; // %
  uint8_t  humidity; // %
} Record_EnvData;
// Runtime error item (16B)
typedef struct {
 uint32_t type;
 uint32_t data;
 uint32_t time;
 uint32_t version;
} Record_ErrData;
// Accel data item (8B)
typedef struct {
  uint16_t x;
  uint16_t y;
  uint16_t z;
} Record_AccelData;
// Active data item (40B)
struct Behavior_Record;
typedef struct Behavior_Record Record_ActData;
// Sensor data item (24B)
typedef struct {
  uint32_t time;
  uint16_t voltage;
  int16_t  temperature;
  uint8_t  humidity;
  uint8_t  dummy1;
  uint16_t pressure;
  uint32_t luminance;
  int16_t  acceleration_x;
  int16_t  acceleration_y;
  int16_t  acceleration_z;
  uint16_t dummy2;
} Record_SensorData;

/* Function prototypes -------------------------------------------------------*/
bool Record_Erase(void);
bool Record_Sample(Record_EnvData *pEnv, uint32_t* error, uint32_t timeout);
bool Record_Error(uint32_t time, uint32_t type, uint32_t data);
uint32_t Record_Count(uint32_t type);
bool Record_Get(uint32_t type, uint32_t offset, uint8_t* pOut);
uint32_t Record_Read(uint32_t type, uint8_t* pOut, uint32_t outBufSize);
uint32_t Record_Peek(uint32_t type, uint8_t* pOut, uint32_t outBufSize);
bool Record_Write(uint32_t type, uint8_t* pData);
uint32_t Record_Writes(uint32_t type, uint8_t* pData, uint32_t nbItems);
bool Record_Delete(uint32_t type, uint32_t nbItems);
uint32_t Record_Size(uint32_t type);
#endif // #ifndef __RECORD_H__
