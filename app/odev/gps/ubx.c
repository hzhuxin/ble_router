/*!
 *    @file  ubx.c
 *   @brief  GPS module based on UBLOX7Q
 *
 *  @author  Dale.J (dj), Dale.J@zoho.com
 *
 *  @internal
 *       Created:  04/13/2016
 *      Revision:  none
 *  Organization:  Druid Tech
 *     Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */

/* Includes ------------------------------------------------------------------ */
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "gps.h"
#include "debug.h"
#include "util.h"
#include "hal_uart.h"
#include "hal_pin.h"
#include "hal_cfg.h"
#include "hal_rtc.h"
#include "minmea.h"

/* Defines ------------------------------------------------------------------- */
DBG_SET_LEVEL(DBG_LEVEL_D);

#define SNR_TIMEOUT         pdMS_TO_TICKS(1000 * 10)
#define CMD_TIMEOUT         pdMS_TO_TICKS(1200)
#define MAX_PKG_SIZE        100
#define PVT_PKG_SIZE        92
#define SETTING_PKG_SIZE    10
#define MINMEA_MAX_LENGTH   100

/* Typedefs ------------------------------------------------------------------ */
typedef struct {
  uint16_t hdr;
  uint16_t id;
  uint16_t len;
  uint8_t  data[2]; // there are far more than two bytes
}UBX_CommandTypeDef;

typedef struct {
  uint32_t iTOW;    // ms
  uint16_t year;
  uint8_t  month;
  uint8_t  day;
  uint8_t  hour;
  uint8_t  min;
  uint8_t  sec;
  uint8_t  valid;   // Bit0 - DateValidFlag, Bit1 - TimeValidFlag, Bit2 - TimeFullyResoloved
  uint32_t tAcc;    // ns
  int32_t  nano;    // ns
  uint8_t  fixType; // 0 - NoFix, 1 - DeadReckoningOnly, 2 - 2DFix, 3 - 3DFix, 4 - GNSS&DeadReckonigCombined, 5 - TimeOnlyFix
  uint8_t  flags;   // Bit0-ValidFix, Bit1-DifferentialCorrectionApplied, Bit432-PowerSaveModeState(1~Enable, 2~Acquisition, 3~Tracking, 4~PowerOptimizedTracking, 5~Inactive
  uint8_t  reserved1;
  uint8_t  numSV;
  int32_t  lon;     // 1e-7 deg
  int32_t  lat;     // 1e-7 deg
  int32_t  height;  // mm
  int32_t  hMSL;    // mm
  uint32_t hAcc;    // mm
  uint32_t vAcc;    // mm
  int32_t  velN;    // mm/s
  int32_t  velE;    // mm/s
  int32_t  velD;    // mm/s
  int32_t  gSpeed;  // mm/s
  int32_t  heading; // 1e-5 deg
  uint32_t sAcc;    // mm/s
  uint32_t headingAcc; // deg
  uint16_t pDOP;
  uint16_t reserved2;
  uint32_t reserved3;
}PVT_CommandTypeDef;

typedef enum {
  GPS_MODE_UNKNOWN = 0,
  GPS_MODE_NMEA,
  GPS_MODE_UBX,
} GPS_Mode;

/* Private variables --------------------------------------------------------- */
// UART port
static GPS_Mode mode = GPS_MODE_UNKNOWN;
static hal_uart_t* uart = NULL;
// Receive buffer
static uint8_t xfer_buf[MAX_PKG_SIZE];
// Port config command
static uint8_t settingUBX[] = {
  0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00,
  0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25,
  0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xA0, 0xA9
};
static uint8_t settingNMEA[] = {
  0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00,
  0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25,
  0x00, 0x00, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xA1, 0xAF
};

/* Pull PVT command */
static uint8_t pvtCmd[] = {
  0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19
};

// Accuracy threshold
static uint32_t hAccuracyThreshold = 15000;
static uint32_t vAccuracyThreshold = 15000;
static bool logEnable = false;

/* Private functions --------------------------------------------------------- */
// Check package
static bool CheckPKG (uint8_t* data, uint32_t len) {
  UBX_CommandTypeDef *pkg = (UBX_CommandTypeDef*)data;

  if(len < 8 || len > MAX_PKG_SIZE) {
    return false;
  }

  if(len != pkg->len + 8) {
    return false;
  }

  if(pkg->hdr != 0x62B5) {
    return false;
  }

  uint8_t ca = 0;
  uint8_t cb = 0;
  for(int i = 2; i < len - 2; i++) {
    ca += data[i];
    cb += ca;
  }

  return (ca == data[len - 2]) && (cb == data[len - 1]);
}

// Parse PVT package
static bool ParsePVT(uint8_t *data, GPS_FixData* pFix) {
  UBX_CommandTypeDef* pkt = (UBX_CommandTypeDef*)data;
  PVT_CommandTypeDef* pvt = (PVT_CommandTypeDef*)pkt->data;

  // Init
  pFix->tValid = false;
  pFix->lValid = false;

  // Timing
  if((pvt->valid & 0x03) == 0x03) {
    pFix->time = pvt->iTOW;
    pFix->year = pvt->year;
    pFix->month = pvt->month;
    pFix->day = pvt->day;
    pFix->hour = pvt->hour;
    pFix->minute = pvt->min;
    pFix->second = pvt->sec;
    pFix->tValid = true;
    // Deal with leap seconds
    if(pFix->second == 60) {
      pFix->second--;
      pFix->time--;
    }
  }

  // Location
  if((pvt->flags&0x01) == 0x01) {
    if(pvt->fixType >= 2 && pvt->fixType <= 4) {
      pFix->gSpeed = pvt->gSpeed;
      pFix->heading = pvt->heading;
      pFix->height = pvt->hMSL;
      pFix->vAcc = pvt->vAcc;
      pFix->lon = pvt->lon;
      pFix->lat = pvt->lat;
      pFix->hAcc = pvt->hAcc;
      pFix->uStar = pvt->numSV;
      pFix->vStar = pvt->numSV;
      pFix->fixType = (pvt->fixType != 0x02) ? GPS_FIX_TYPE_3D : GPS_FIX_TYPE_2D;
      pFix->lValid = true;
    }
  }

  return pFix->lValid || pFix->tValid;
}

// Try to get a fix
static uint32_t getFix (GPS_FixData* pFix) {
  if(!uart) {
    return GPS_ERR_HW;
  }
  if(mode != GPS_MODE_UBX) {
    // Setting command
    vTaskDelay(pdMS_TO_TICKS(500));
    uart->ops->write(uart, settingUBX, sizeof(settingUBX));
    vTaskDelay(500);
    uart->ops->clr_rx_data(uart);
    mode = GPS_MODE_UBX;
  }

  memset(pFix, 0, sizeof(GPS_FixData));
  if(uart->ops->write(uart, pvtCmd, sizeof(pvtCmd)) == sizeof(pvtCmd)) {
    int len = uart->ops->read(uart, xfer_buf, PVT_PKG_SIZE);
#ifdef LOG_ENABLE
    if(logEnable && len) {
      char* buf = pvPortMalloc((len + 1)* 2);
      int i = 0;
      while(buf && i < len) {
        sprintf(&buf[i * 2], "%02X", xfer_buf[i]);
        i++;
      }
      DBG_L("[GPS] %s\n", buf);
      vPortFree(buf);
    }
#endif
    if(len == PVT_PKG_SIZE) {
      if(CheckPKG(xfer_buf, PVT_PKG_SIZE)) {
        if(ParsePVT(xfer_buf, pFix)) {
          return GPS_ERR_OK;
        } else {
          return GPS_ERR_PENDING;
        }
      } else {
        return GPS_ERR_PKG;
      }
    }
  }
  return GPS_ERR_HW;
}

// input a string
char* GPS_gets(TickType_t timeout) {
  if(!uart) {
    return NULL;
  }
  if(mode != GPS_MODE_NMEA) {
    vTaskDelay(500);
    uart->ops->write(uart, settingNMEA, sizeof(settingNMEA));
    vTaskDelay(500);
    uart->ops->clr_rx_data(uart);
    mode = GPS_MODE_NMEA;
  }

  int i = 0;
  TickType_t start = xTaskGetTickCount();
  while(!util_timeout(start, xTaskGetTickCount(), timeout) && i < MINMEA_MAX_LENGTH - 1) {
    char c = 0;
    if(uart->ops->read(uart, &c, 1) == 1) {
      if(c == '\n') {
        if(i != 0) {
          xfer_buf[i] = '\0';
          return (char*)xfer_buf;
        }
      } else if(c != '\r') {
        xfer_buf[i++] = c;
      }
    }
  }

  return NULL;
}

/* Global functions ----------------------------------------------------------*/

/**
 * @brief Power on the gps module
 */
void GPS_PowerOn(void) {
  // Mode
  mode = GPS_MODE_UNKNOWN;
  // Power control
  DBG_D("GPS: Back power on\r\n");
  hal_pin_set_mode_ex(HAL_CFG_GPS_BKP, HAL_PIN_MODE_OUT, HAL_PIN_MODE_EX_HIGH, HAL_PIN_LVL_HIGH);
  DBG_D("GPS: power on\r\n");
  hal_pin_set_mode_ex(HAL_CFG_GPS_PWR, HAL_PIN_MODE_OUT, HAL_PIN_MODE_EX_HIGH, HAL_PIN_LVL_HIGH);
  // UART port
  uart = hal_uart_get_instance(0);
  if(!uart) {
    return;
  }
  const hal_uart_cfg_t cfg = {
    .baudrate = HAL_UART_BAUDRATE_9600,
    .parity = HAL_UART_PARITY_NONE,
    .tx_mode = HAL_UART_TX_MODE_NOCOPY,
    .rx_mode = HAL_UART_RX_MODE_BUFFERED,
    .rx_buf_size = 512,
    .rx_timeout_ms = CMD_TIMEOUT,
    .tx_timeout_ms = CMD_TIMEOUT,
    .tx_pin = HAL_CFG_GPS_TXD,
    .rx_pin = HAL_CFG_GPS_RXD,
  };
  uart->ops->init(uart, &cfg);
  uart->ops->set_rx_enable(uart, true);
}

/**
 * @brief Power off the gps module
 */
void GPS_PowerOff(void) {
  DBG_D("GPS: Power off\r\n");
  // Power control
  hal_pin_set_mode_ex(HAL_CFG_GPS_PWR, HAL_PIN_MODE_OUT, HAL_PIN_MODE_EX_NORMAL, HAL_PIN_LVL_LOW);
  vTaskDelay(10);
  // UART port
  if(uart) {
    uart->ops->deinit(uart);
    uart = NULL;
  }
}


/**
 * @brief Turn on the backup power
 */
void GPS_BackPowerOn(void) {
  DBG_D("GPS: Back power on\r\n");
  hal_pin_set_mode_ex(HAL_CFG_GPS_BKP, HAL_PIN_MODE_OUT, HAL_PIN_MODE_EX_NORMAL, HAL_PIN_LVL_HIGH);
}

/**
 * @brief Turn off the backup power
 */
void GPS_BackPowerOff(void) {
  DBG_D("GPS: Back power off\r\n");
  hal_pin_set_mode_ex(HAL_CFG_GPS_BKP, HAL_PIN_MODE_OUT, HAL_PIN_MODE_EX_NORMAL, HAL_PIN_LVL_LOW);
}

/**
 * @brief Test hardware connection
 *
 * @return true if success
 */
bool GPS_HardwareTest(void) {
  if(uart) {
    if(mode != GPS_MODE_UBX) {
      // Setting command
      vTaskDelay(pdMS_TO_TICKS(500));
      uart->ops->write(uart, settingUBX, sizeof(settingUBX));
      vTaskDelay(500);
      uart->ops->clr_rx_data(uart);
      mode = GPS_MODE_UBX;
    }
    if(uart->ops->write(uart, settingUBX, sizeof(settingUBX)) == sizeof(settingUBX)) {
      if(uart->ops->read(uart, xfer_buf, SETTING_PKG_SIZE) == SETTING_PKG_SIZE) {
        if(CheckPKG(xfer_buf, SETTING_PKG_SIZE)) {
          DBG_D("GPS: Hardware test succeed\r\n");
          return true;
        }
      }
    }
  }
  DBG_E("GPS: Hardware test failed\r\n");
  return false;
}

/**
 * @brief Fix GPS
 *
 * @param pFix
 *
 * @return true if success
 */
bool GPS_Fix(GPS_FixData* pFix, TickType_t fix_timeout, TickType_t accuracy_timeout) {
  DBG_D("GPS: Start fix\r\n");
  if(logEnable) {
    DBG_L("[GPS]========[%d]\n", hal_rtc_get_time());
  }
#if 0
  vTaskDelay(pdMS_TO_TICKS(12000));
  DBG_I("GPS: return fake data for debug purpose\r\n");
  if(RTC_Valid()) {
    RTC_Get((RTC_Time_t*)pFix);
  } else {
    pFix->day = 26;
    pFix->month = 5;
    pFix->year = 2016;
    pFix->second = 00;
    pFix->minute = 54;
    pFix->hour = 15;
    pFix->tValid = true;
  }
  pFix->fixType = GPS_FIX_TYPE_3D;
  pFix->height = 420000;
  pFix->lat = 1013560000;
  pFix->lon = 301160000;
  pFix->gSpeed = 125;
  pFix->hAcc = 26000;
  pFix->vAcc = 66000;
  pFix->heading = 12300000;
  pFix->uStar = 5;
  pFix->vStar = 5;
  pFix->lValid = true;

  return true;
#else
  uint32_t ret = GPS_ERR_OK;
  TickType_t start = xTaskGetTickCount();
  memset(pFix, 0, sizeof(GPS_FixData));
  while(!util_timeout(start, xTaskGetTickCount(), fix_timeout)) {
    GPS_FixData fix;
    ret = getFix(&fix);
    if(ret == GPS_ERR_OK) {
      if(fix.lValid && fix.tValid) {
        DBG_V("GPS: full fix(%d, %d)\r\n", fix.hAcc, fix.vAcc);
        if(!pFix->lValid || fix.hAcc <= pFix->hAcc) {
          memcpy(pFix, &fix, sizeof(GPS_FixData));
        } else {
          pFix->time = fix.time;
          pFix->year = fix.year;
          pFix->month = fix.month;
          pFix->day = fix.day;
          pFix->hour = fix.hour;
          pFix->minute = fix.minute;
          pFix->second = fix.second;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
      } else {
        DBG_V("GPS: particial fix\r\n");
      }
    } else if(ret == GPS_ERR_PENDING) {
      DBG_V("GPS: fixing\r\n");
    } else if(ret == GPS_ERR_PKG) {
      DBG_W("GPS: frame error\r\n");
    } else if(ret == GPS_ERR_HW) {
      DBG_E("GPS: hardware error\r\n");
    }
    if(pFix->lValid) {
        if((pFix->hAcc < hAccuracyThreshold && pFix->vAcc < vAccuracyThreshold) || util_timeout(start, xTaskGetTickCount(), accuracy_timeout)) {
          DBG_I("GPS: fix succeed in %d seconds\r\n",  (xTaskGetTickCount() - start) / 1000);
          return true;
        }
    }
  }

  if(!pFix->lValid) {
    DBG_W("GPS: fix failed\r\n");
    return false;
  }

  return true;
#endif
}

bool GPS_Open(void)
{
   GPS_PowerOn();
   GPS_BackPowerOn();

   return true;
}

bool GPS_GetSNR(uint8_t *snr, uint8_t svid)
{
  enum minmea_sentence_id type;
  char *line = NULL;
  
  line = GPS_gets(1000);
  type = minmea_sentence_id(line, false);

  if(type == MINMEA_SENTENCE_GSV){
     struct minmea_sentence_gsv gsv;
     memset(&gsv, 0, sizeof(gsv));
     
     if(minmea_parse_gsv(&gsv, line)) {
         for(int i = 0; i < gsv.total_msgs; i++)
         {
             if(gsv.sats[i].nr == svid)
             {
               *snr = gsv.sats[i].snr;
               return true;
             }
         }
     }
  }

  return false;
}

bool GPS_Close(void)
{
   GPS_PowerOff();
   GPS_BackPowerOff();

   return true;
}

/**
 * @brief Fix GPS
 *
 * @param hAcc hAccuracy threshold
 * @param vAcc vAccuracy threshold
 *
 */
void GPS_SetParam(uint32_t hAcc, uint32_t vAcc) {
  hAccuracyThreshold = hAcc;
  vAccuracyThreshold = vAcc;
}

/**
 * @brief Enable or disable log
 *
 * @param en log enable
 *
 */
void GPS_SetLog(bool en) {
  logEnable = en;
}

