/*!
 *    @file  GPS.c
 *   @brief  GPS module based on NMEA
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

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "gps.h"
#include "util.h"
#include "hal_uart.h"
#include "hal_pin.h"
#include "debug.h"
#include "minmea.h"
#include "hal_cfg.h"
#include "hal_rtc.h"

/* Defines -------------------------------------------------------------------*/
DBG_SET_LEVEL(DBG_LEVEL_D);

#define FIX_GGA             0x01
#define FIX_GSA             0x02
#define FIX_GSV             0x04
#define FIX_RMC             0x08
#ifdef HAL_CFG_USE_868
#define FIX_ACC             0x10
#define FIX_MASK            0x1F
#else
#define FIX_MASK            0x0F
#endif

#define TEST_TIMEOUT        pdMS_TO_TICKS(4200)
#define CMD_TIMEOUT         pdMS_TO_TICKS(2200)
#define GET_TIMEOUT         pdMS_TO_TICKS(2000)
#define SEQ_TIMEOUT         pdMS_TO_TICKS(400)

#define FIRST_SV_TIMEOUT    pdMS_TO_TICKS(20 * 1000)
#define MORE_SV_TIMEOUT     pdMS_TO_TICKS(25 * 1000)
#define STABLE_TIMEOUT      pdMS_TO_TICKS(15 * 1000)

/* Typedefs ------------------------------------------------------------------*/
typedef struct FixState {
  TickType_t time;
  uint8_t mask;
  uint8_t quality;
  uint8_t got_gsv;
  uint8_t snr_cnt;
  uint8_t snr_sum;
  uint8_t nbsvs_predicted;
  uint8_t nbsvs_found;
  uint8_t nbsvs_fixed;
} FixState_t;

/* Private variables ---------------------------------------------------------*/
// UART port
static hal_uart_t* uart = NULL;
// Receive buffer
static uint8_t xfer_buf[MINMEA_MAX_LENGTH];
// Accuracy threshold
static uint32_t hAccuracyThreshold = 20000;
static uint32_t vAccuracyThreshold = 30000;
static bool logEnable = false;

/* Private functions ---------------------------------------------------------*/
// input a string
char* GPS_gets(TickType_t timeout) {
  int i = 0;
  TickType_t start = xTaskGetTickCount();
  while(!util_timeout(start, xTaskGetTickCount(), timeout) && i < sizeof(xfer_buf) - 1) {
    char c = 0;
    if(uart->ops->read(uart, &c, 1) == 1) {
      if(c == '\n') {
        if(i != 0) {
          xfer_buf[i] = '\0';
          DBG_V("%s", xfer_buf);
          DBG_F();
          return (char*)xfer_buf;
        }
      } else if(c != '\r') {
        xfer_buf[i++] = c;
      }
    }
  }

  return NULL;
}

// Try to get a fix
static uint32_t getFix (GPS_FixData* pFix, TickType_t timeout) {
  TickType_t start = xTaskGetTickCount();
  FixState_t state = { 0 };

  memset(pFix, 0, sizeof(GPS_FixData));
  while(!util_timeout(start, xTaskGetTickCount(), timeout)) {
    char *line = GPS_gets(GET_TIMEOUT);
    if(line == NULL) {
      continue;
    } else if(logEnable) {
      DBG_L(line);
      DBG_L("\n");
    }
    TickType_t now = xTaskGetTickCount();
    if(util_timeout(state.time, now, SEQ_TIMEOUT)) {
      if((state.mask | FIX_GSV) == FIX_MASK) {
        state.mask |= FIX_GSV;
        break;
      } else {
        memset(pFix, 0, sizeof(GPS_FixData));
        memset(&state, 0, sizeof(FixState_t));
      }
    }
    state.time = now;

    enum minmea_sentence_id type = minmea_sentence_id(line, false);
    switch (type) {
      case MINMEA_SENTENCE_GGA: {
        struct minmea_sentence_gga gga;
        if (minmea_parse_gga(&gga, line)) {
          pFix->uStar = gga.satellites_tracked;
          pFix->height = minmea_rescale(&gga.altitude, 1000);
          state.quality = gga.fix_quality;  // 0 - invalid; 1/2 - valid; 6 - Dead Reckoning
          state.mask |= FIX_GGA;
        }
        break;
      }
      case MINMEA_SENTENCE_GSA: {
        struct minmea_sentence_gsa gsa;
        if (minmea_parse_gsa(&gsa, line)) {
          pFix->fixType = gsa.fix_type - 1;
          pFix->hdop = minmea_rescale(&gsa.hdop, 10000);
          pFix->vdop = minmea_rescale(&gsa.vdop, 10000);
          state.mask |= FIX_GSA;
        }
        break;
      }
      case MINMEA_SENTENCE_GSV: {
        struct minmea_sentence_gsv gsv;
        memset(&gsv, 0, sizeof(gsv));
        if (minmea_parse_gsv(&gsv, line)) {
          for(int i = 0; i < sizeof(gsv.sats) / sizeof(gsv.sats[0]); i++) {
            DBG_V("  %d, %d, %d, %d\r\n", gsv.sats[i].nr, gsv.sats[i].snr, gsv.sats[i].elevation, gsv.sats[i].azimuth);
            if((gsv.sats[i].nr >= 1 && gsv.sats[i].nr <= 32) ||
                (gsv.sats[i].nr >= 65 && gsv.sats[i].nr <= 96)) {
              if(gsv.sats[i].snr != 0) {
                state.snr_cnt += 1;
                state.snr_sum += gsv.sats[i].snr;

                if(gsv.sats[i].elevation || gsv.sats[i].azimuth) {
                  state.nbsvs_fixed++;
                } else {
                  state.nbsvs_found++;
                }
              } else if(gsv.sats[i].elevation || gsv.sats[i].azimuth) {
                state.nbsvs_predicted++;
              }
            }
          }
          if(gsv.total_msgs == gsv.msg_nr) {
            pFix->vStar += state.nbsvs_fixed + state.nbsvs_found;
            pFix->nStar += state.nbsvs_found;
            state.got_gsv = true;
            state.nbsvs_predicted = 0;
            state.nbsvs_found = 0;
            state.nbsvs_fixed = 0;
          }
        }
        break;
      }
      case MINMEA_SENTENCE_RMC: {
        struct minmea_sentence_rmc rmc;
        if (minmea_parse_rmc(&rmc, line)) {
          if (rmc.valid) {
            pFix->year = rmc.date.year + 2000;
            pFix->month = rmc.date.month;
            pFix->day = rmc.date.day;
            pFix->hour = rmc.time.hours;
            pFix->minute = rmc.time.minutes;
            pFix->second = rmc.time.seconds;
            pFix->lat = minmea_tocoord(&rmc.latitude) * 10000000;
            pFix->lon = minmea_tocoord(&rmc.longitude) * 10000000;
            pFix->heading = minmea_rescale(&rmc.course, 100000);
            pFix->gSpeed = minmea_tofloat(&rmc.speed) * 510;
            pFix->tValid = true;
            pFix->lValid = true;
          }
          state.mask |= FIX_RMC;
        }
        break;
      }
      case MINMEA_INVALID:
        DBG_W("GPS: Invalid sentence [%s]\r\n", line)
        DBG_F();
        break;
      case MINMEA_UNKNOWN:
#ifdef HAL_CFG_USE_868
        {
          struct minmea_float f;
          if(!strncmp(line, "$GPACCURACY,", strlen("$GPACCURACY,")) && minmea_scan(line + strlen("$GPACCURACY,"), "f", &f)) {
            state.mask |= FIX_ACC;
            pFix->hAcc = minmea_tofloat(&f) > 9990000 ? 999999999 : minmea_tofloat(&f) * 1000;
          }
        }
#endif
        break;
      default:
        break;
    }
    if(type != MINMEA_SENTENCE_GSV) {
      if(state.got_gsv) {
        state.mask |= FIX_GSV;
      }
    }
    if(state.mask == FIX_MASK) {
#ifndef HAL_CFG_USE_868
      pFix->hAcc = 999999999;
#endif
      if(state.snr_cnt != 0) {
        pFix->signal = state.snr_sum / state.snr_cnt;
      }
      if(state.quality != 1 && state.quality != 2) {
        pFix->lValid = false;
      }
      break;
    }
  }

  if (state.mask != FIX_MASK) {
    return GPS_ERR_HW;
  }
  if (pFix->tValid && pFix->lValid) {
    return GPS_ERR_OK;
  }
  return GPS_ERR_PENDING;
}

/* Global functions ----------------------------------------------------------*/

/**
 * @brief Power on the gps module
 */
void GPS_PowerOn(void) {
  if(uart != NULL) {
    return;
  }
  DBG_I("GPS: Back power on\r\n");
  hal_pin_set_mode_ex(HAL_CFG_GPS_BKP, HAL_PIN_MODE_OUT, HAL_PIN_MODE_EX_NORMAL, HAL_PIN_LVL_HIGH);
  DBG_I("GPS: Power on\r\n");
  // Power control
  hal_pin_set_mode_ex(HAL_CFG_GPS_PWR, HAL_PIN_MODE_OUT, HAL_PIN_MODE_EX_NORMAL, HAL_PIN_LVL_HIGH);
  vTaskDelay(pdMS_TO_TICKS(2000));
  // UART port
  uart = hal_uart_get_instance(0);
  if(!uart) {
    return;
  }
  const hal_uart_cfg_t cfg = {
#if defined(HAL_CFG_USE_868) && HAL_CFG_USE_868
    .baudrate = HAL_UART_BAUDRATE_115200,
#else
    .baudrate = HAL_UART_BAUDRATE_9600,
#endif
    .parity = HAL_UART_PARITY_NONE,
    .tx_mode = HAL_UART_TX_MODE_NOCOPY,
    .rx_mode = HAL_UART_RX_MODE_BUFFERED,
    .rx_buf_size = 640,
    .rx_timeout_ms = 200,
    .tx_timeout_ms = 100,
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
  DBG_I("GPS: Power off\r\n");
  // UART port
  if(uart != NULL) {
    uart->ops->deinit(uart);
    uart = NULL;
  }
  // Power control
  hal_pin_set_mode_ex(HAL_CFG_GPS_PWR, HAL_PIN_MODE_OUT, HAL_PIN_MODE_EX_NORMAL, HAL_PIN_LVL_LOW);
  vTaskDelay(200);
}


/**
 * @brief Turn on the backup power
 */
void GPS_BackPowerOn(void) {
  DBG_I("GPS: Back power on\r\n");
  hal_pin_set_mode_ex(HAL_CFG_GPS_BKP, HAL_PIN_MODE_OUT, HAL_PIN_MODE_EX_NORMAL, HAL_PIN_LVL_HIGH);
}

/**
 * @brief Turn off the backup power
 */
void GPS_BackPowerOff(void) {
  DBG_I("GPS: Back power off\r\n");
  hal_pin_set_mode_ex(HAL_CFG_GPS_BKP, HAL_PIN_MODE_OUT, HAL_PIN_MODE_EX_NORMAL, HAL_PIN_LVL_HIGH);
}

/**
 * @brief Test hardware connection
 *
 * @return true if success
 */
bool GPS_HardwareTest(void) {
  if(uart != NULL) {
    GPS_FixData* fix = pvPortMalloc(sizeof(GPS_FixData));
    if(fix) {
      int ret = getFix(fix, TEST_TIMEOUT);
      vPortFree(fix);
      if(ret != GPS_ERR_HW) {
        DBG_I("GPS: Hardware test succeed\r\n");
        return true;
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
/* #include "rtc.h" */
bool GPS_Fix(GPS_FixData* pFix, TickType_t fix_timeout, TickType_t accuracy_timeout) {
  DBG_I("GPS: Start fix\r\n");
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
  TickType_t firstFixTime = 0;
  TickType_t stableTimer = 0;
  // DBG_L("====================== %d\n", hal_rtc_get_time());
  memset(pFix, 0, sizeof(GPS_FixData));
  GPS_FixData* fix = pvPortMalloc(sizeof(GPS_FixData));
  if(!fix) {
    DBG_E("GPS: memory error\r\n");
    return false;
  }
  while(!util_timeout(start, xTaskGetTickCount(), fix_timeout)) {
    ret = getFix(fix, CMD_TIMEOUT);
    if(ret == GPS_ERR_OK) {
      if(fix->lValid && fix->tValid) {
        DBG_D("GPS: full fix, time=%d, #stars=%d, signal=%d, accuracy=(%d, %d)\r\n", (xTaskGetTickCount() - start) / configTICK_RATE_HZ, fix->vStar, fix->signal, fix->hAcc, fix->vAcc);
        if(!pFix->lValid) {
          firstFixTime = xTaskGetTickCount();
        }
        if(!pFix->lValid || fix->hAcc <= pFix->hAcc) {
          memcpy(pFix, fix, sizeof(GPS_FixData));
        } else {
          pFix->time = fix->time;
          pFix->year = fix->year;
          pFix->month = fix->month;
          pFix->day = fix->day;
          pFix->hour = fix->hour;
          pFix->minute = fix->minute;
          pFix->second = fix->second;
        }
        if(fix->hAcc < hAccuracyThreshold && fix->vAcc < vAccuracyThreshold) {
          break;
        } else if(util_timeout(firstFixTime, xTaskGetTickCount(), MORE_SV_TIMEOUT)) {
          if(fix->nStar) {
            stableTimer = xTaskGetTickCount();
          } else if(util_timeout(stableTimer, xTaskGetTickCount(), STABLE_TIMEOUT)) {
            break;
          }
        }
      } else {
        DBG_D("GPS: particial fix, #stars=%d, signal=%d, hacc = %d\r\n", fix->vStar, fix->signal, fix->hAcc);
      }
    } else if(ret == GPS_ERR_PENDING) {
      DBG_D("GPS: fixing, time=%d, #stars=%d, signal=%d\r\n", (xTaskGetTickCount() - start) / configTICK_RATE_HZ, fix->vStar, fix->signal);
      if(!fix->vStar && util_timeout(start, xTaskGetTickCount(), FIRST_SV_TIMEOUT)) {
        DBG_I("GPS: no satellite found\r\n");
        fix->lValid = false;
        break;
      }
    } else if(ret == GPS_ERR_PKG) {
      DBG_D("GPS: frame error\r\n");
    } else if(ret == GPS_ERR_HW) {
      DBG_D("GPS: hardware error\r\n");
    }
    if(pFix->lValid) {
      if(util_timeout(start, xTaskGetTickCount(), accuracy_timeout)) {
        DBG_I("GPS: fix succeed in %d seconds, #stars=%d, signal=%d\r\n",  (xTaskGetTickCount() - start) / 1000, pFix->vStar, pFix->signal);
        break;
      }
    }
  }

  vPortFree(fix);
  if(!pFix->lValid) {
    DBG_W("GPS: fix failed\r\n");
    return false;
  }

  return true;
#endif
}

/**
 * @brief Set accuracy param
 *
 * @param hAcc
 * @param vAcc
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

