/*!
 *    @file  Debug.h
 *   @brief  The debug module
 *
 *  @author  Dale.J (dj), Dale.J@zoho.com
 *
 *  @internal
 *       Created:  04/30/2016
 *      Revision:  none
 *  Organization:  Druid Tech
 *     Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */

#ifndef __DEBUG_H__
#define __DEBUG_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/* Typedefs ------------------------------------------------------------------*/
enum DBG_Level {
  DBG_LEVEL_N = 0,  // off
  DBG_LEVEL_E,      // Error
  DBG_LEVEL_W,      // Warning
  DBG_LEVEL_I,      // Info
  DBG_LEVEL_D,      // Debug
  DBG_LEVEL_V,      // Verbose
};

/* Defines -------------------------------------------------------------------*/
// Debug control
#define DBG_ENABLE

#ifdef LOG_ENABLE
// Log macros
#define DBG_L(...)              \
do {                            \
  DBG_log(__VA_ARGS__);         \
} while(0);
#else
#define DBG_L(...)
#endif // #ifdef LOG_ENABLE

#ifdef DBG_ENABLE
// Debug level
#define DBG_SET_LEVEL(x) static const int debug = x
// Debug macros
#define DBG_E(...)              \
do {                            \
  if(debug >= DBG_LEVEL_E) {    \
    NRF_LOG_ERROR(__VA_ARGS__); \
  }                             \
} while(0);

#define DBG_W(...)              \
do {                            \
  if(debug >= DBG_LEVEL_W) {    \
    NRF_LOG_WARNING(__VA_ARGS__);\
  }                             \
} while(0);

#define DBG_I(...)              \
do {                            \
  if(debug >= DBG_LEVEL_I) {    \
    NRF_LOG_INFO(__VA_ARGS__);  \
  }                             \
} while(0);

#define DBG_D(...)              \
do {                            \
  if(debug >= DBG_LEVEL_D) {    \
    NRF_LOG_DEBUG(__VA_ARGS__); \
  }                             \
} while(0);

#define DBG_V(...)              \
do {                            \
  if(debug >= DBG_LEVEL_V) {    \
    NRF_LOG_DEBUG(__VA_ARGS__); \
  }                             \
} while(0);
/*
#define DBG_F(...)              \
do {                            \
  NRF_LOG_FLUSH(__VA_ARGS__);   \
} while(0);
*/
#else
#define DBG_SET_LEVEL(x)
#define DBG_E(...)
#define DBG_W(...)
#define DBG_I(...)
#define DBG_D(...)
#define DBG_V(...)
#define DBG_F(...)
#endif // #ifdef DBG_ENABLE

/* Function prototypes -------------------------------------------------------*/
// void DBG_Init(bool rxEnable);
// void DBG_log(const char *format, ...);
// char* DBG_gets(TickType_t timeout);
// void DBG_print(const char *format, ...);
// bool DBG_printBase64(char* buf, size_t len) ;
// Peripheral_Descriptor_t DBG_GetPort(void);

#endif // #ifndef __DEBUG_H__

