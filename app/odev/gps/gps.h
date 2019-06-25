/*!
 * @file  GPS.h
 * @brief  GPS module
 *
 * @author  Dale.J (dj), Dale.J@zoho.com
 *
 * @internal
 *      Created:  04/13/2016
 *      Revision:  none
 * Organization:  Druid Tech
 *      Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */

#ifndef __GPS_H__
#define __GPS_H__

/* Includes ------------------------------------------------------------------ */
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

/* Typedefs ------------------------------------------------------------------ */
enum GPS_FixType {
  GPS_FIX_TYPE_NONE = 0,
  GPS_FIX_TYPE_2D = 1,
  GPS_FIX_TYPE_3D = 2,
};

enum GPS_ErrorType {
  GPS_ERR_OK = 0,
  GPS_ERR_HW = 1,
  GPS_ERR_PKG = 2,
  GPS_ERR_PENDING = 3,
};

typedef struct {
  // Timing
  uint32_t time;    // ms
  uint16_t year;    // YYYY
  uint8_t month;    // MM
  uint8_t day;      // DD
  uint8_t hour;     // HH
  uint8_t minute;   // MM
  uint8_t second;   // SS
  uint8_t tValid;   // time valid

  // Location
  int32_t  lon;     // 1e-7 deg
  int32_t  lat;     // 1e-7 deg
  int32_t  height;  // mm
  uint32_t hAcc;    // mm
  uint32_t vAcc;    // mm
  uint32_t hdop;    // * 10000
  uint32_t vdop;    // * 10000
  int32_t  gSpeed;  // mm/s
  int32_t  heading; // 1e-5 deg
  uint8_t  uStar;   // satellites used
  uint8_t  vStar;   // satellites in view
  uint8_t  nStar;   // satellites found
  uint8_t  fixType; // 0 - No fix; 1 - 2D fix; 2 - 3D Fix
  uint8_t  lValid;  // location valid
  uint8_t  signal;  // average signal strength
  uint8_t  dummy[2];
}GPS_FixData;

/* Function prototypes ------------------------------------------------------- */
void GPS_PowerOn(void);
void GPS_PowerOff(void);
void GPS_BackPowerOn(void);
void GPS_BackPowerOff(void);
bool GPS_HardwareTest(void);
bool GPS_Fix(GPS_FixData* pFix, TickType_t fix_timeout, TickType_t accuracy_timeout);
void GPS_SetParam(uint32_t hAcc, uint32_t vAcc);
char* GPS_gets(TickType_t timeout);
void GPS_SetLog(bool en);

bool GPS_Open(void);
bool GPS_GetSNR(uint8_t *snr, uint8_t svid);
bool GPS_Close(void);

#endif // #ifndef __GPS_H__
