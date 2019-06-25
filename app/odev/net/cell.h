/*!
 *    @file  CELL.h
 *   @brief  Cellular module
 *
 *  @author  Dale.J (dj), Dale.J@zoho.com
 *
 *  @internal
 *       Created:  04/10/2016
 *      Revision:  none
 *  Organization:  Druid Tech
 *     Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */

#ifndef __CELL_H__
#define __CELL_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"

/* Defines -------------------------------------------------------------------*/
#define CELL_CMD_TIMEOUT    pdMS_TO_TICKS(32 * 1000)
#define CELL_MIN_RSSI       2
#define CELL_NB_CONNID      1200 // CAUTION: must be same as CELLULAR_NB_CONNID

/* Typedefs ------------------------------------------------------------------*/
typedef enum {
  NRS_IDLE  = 0,
  NRS_REGISTERED,
  NRS_ONGOING,
  NRS_DENIED,
  NRS_UNKNOWN,
  NRS_ROAMING
} NetRegStatus;

enum {
  CELL_ERR_HWT = 1 << 0,
  CELL_ERR_SIM = 1 << 1,
  CELL_ERR_CSQ = 1 << 2,
  CELL_ERR_REG = 1 << 3,
  CELL_ERR_NUM = 1 << 4,
  CELL_ERR_ATT = 1 << 5,
  CELL_ERR_PDP = 1 << 6,
  CELL_ERR_CON = 1 << 7,
  CELL_ERR_COM = 1 << 8,
  CELL_ERR_SMS  = 1 << 9,
  CELL_ERR_CMD  = 1 << 14,
  CELL_ERR_RST  = 1 << 15,
  CELL_ERR_INI_MASK = CELL_ERR_HWT | CELL_ERR_SIM,
  CELL_ERR_REG_MASK = CELL_ERR_CSQ | CELL_ERR_REG | CELL_ERR_NUM,
  CELL_ERR_PRE_MASK = CELL_ERR_ATT | CELL_ERR_PDP,
};

// Cellular data item (56B)
typedef struct {
  uint32_t time;
  int32_t  longitude;
  int32_t  latitude;
  uint16_t voltage;
  uint16_t err_flag;
  uint16_t hwt_time;
  uint16_t sim_time;
  uint16_t csq_time;
  uint16_t reg_time;
  uint16_t num_time;
  uint16_t att_time;
  uint16_t pdp_time;
  uint16_t con_time;
  uint16_t com_time;
  uint16_t sms_time;
  uint16_t tot_time;
  uint8_t  rssi;
  uint8_t  ber;
  uint32_t mnc;
  uint8_t  rat;
  uint8_t  version;
  int16_t  temperature;
  uint16_t ext_flag;
  uint8_t  ext_rssi;
  uint8_t  ext_ber;
  uint8_t  dummy[4];
} CellData_t;

/* Function prototypes -------------------------------------------------------*/
bool CELL_Reset(void);
bool CELL_Suspend(void);
bool CELL_Resume(void);
bool CELL_PowerOn(void);
bool CELL_PowerOff(void);
bool CELL_Shutdown(void);
bool CELL_Init(void);
bool CELL_Deinit(void);
bool CELL_Register(void);
bool CELL_Prepare(void);
int CELL_Connect(char* pHost, uint16_t port);
int CELL_Send(int connid, char* data, int len);
int CELL_Recv(int connid, char* data, int len);
bool CELL_Waitack(int connid);
bool CELL_Close(int connid);
bool CELL_HardwareTest(void);
bool CELL_AutoAnswerCalls(void);
bool CELL_SMSSend(char* num, char* msg, uint8_t len);
int CELL_GetCSQ(void);
int CELL_GetNC(void);
char* CELL_GetIMEI(void);
char* CELL_GetIMSI(void);
char* CELL_GetICCID(void);
char* CELL_GetNUM(void);
bool CELL_TxStart(void);
bool CELL_TxStop(void);
bool CELL_WriteNUM(uint8_t* pNUM);
bool CELL_CheckCSQ(int32_t level);
bool CELL_GetStatusPin(void);
CellData_t* CELL_GetData(void);
#endif // #ifndef __CELL_H__

