/*!
 *    @file  cell.c
 *   @brief  Common code for cellular module
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


/* Includes ------------------------------------------------------------------*/
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "util.h"
#include "cell.h"
#include "hal_rtc.h"

#include "attentive/at-freertos.h"
#include "attentive/cellular.h"

/* Defines -------------------------------------------------------------------*/
DBG_SET_LEVEL(DBG_LEVEL_I);

#define CGATT_TIMEOUT pdMS_TO_TICKS(1000 * 200)
#define CREG_TIMEOUT  pdMS_TO_TICKS(1000 * 180)
#define RSSI_TIMEOUT  pdMS_TO_TICKS(1000 * 60)
#define IMEI_TIMEOUT  pdMS_TO_TICKS(1000 * 10)
#define IMSI_TIMEOUT  pdMS_TO_TICKS(1000 * 10)
#define ICCID_TIMEOUT pdMS_TO_TICKS(1000 * 10)
#define WNUM_TIMEOUT  pdMS_TO_TICKS(1000 * 10)
#define RNUM_TIMEOUT  pdMS_TO_TICKS(1000 * 40)
#define CMD_RETRIES   10

/* Private variables ---------------------------------------------------------*/
static struct at *at = NULL;
static struct cellular *modem = NULL;
static char IMEI[CELLULAR_IMEI_LENGTH + 1] = {0};
static char IMSI[CELLULAR_IMSI_LENGTH + 1] = {0};
static char ICCID[CELLULAR_ICCID_LENGTH + 1] = {0};
static char NUM[CELLULAR_NUM_LENGTH + 1] = {0};
static TickType_t startTime = 0;
static CellData_t data = {0};

/* Global functions ----------------------------------------------------------*/

/**
 * @brief Test hardware connection
 *
 * @return true if success
 */
bool CELL_HardwareTest(void) {
  DBG_D("CELL: Hardware testing...\r\n");

  bool succeed = (modem->ops->test(modem) == 0);

  if(succeed) {
    DBG_I("CELL: Hardware test succeed\r\n");
  } else {
    DBG_E("CELL: Hardware test fail\r\n");
  }

  return succeed;
}

/**
 * @brief Enable auto answer calls
 *
 * @return true if success
 */
bool CELL_AutoAnswerCalls(void) {
  DBG_D("CELL: Enabling auto answer...\r\n");

  const char* response = modem->ops->command(modem, "ATS0=2", 2);
  bool succeed = response && *response == '\0';

  if(succeed) {
    DBG_I("CELL: Enable auto answer succeed\r\n");
  } else {
    DBG_E("CELL: Enable auto answer fail\r\n");
  }

  return succeed;
}

/**
 * @brief Read IMEI
 *
 * @param pIMEI
 *
 * @return true if success
 */
bool CELL_ReadIMEI(uint8_t* pIMEI) {
  DBG_D("CELL: Reading imei...\r\n");

  bool succeed = false;
  TickType_t start = xTaskGetTickCount();
  while(!util_timeout(start, xTaskGetTickCount(), IMEI_TIMEOUT)) {
    if(modem->ops->imei(modem, (char*)IMEI, sizeof(IMEI)) == 0) {
      succeed = true;
      if(pIMEI != NULL) {
        memcpy(pIMEI, IMEI, sizeof(IMEI));
      }
      break;
    } else {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  if(succeed) {
    DBG_I("CELL: Read imei succeed in %d seconds: %s\r\n", (xTaskGetTickCount() - start) / 1000, IMEI);
  } else {
    DBG_E("CELL: Read imei failed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
  }

  return succeed;
}

/**
 * @brief Get the IMEI string
 *
 * @return pointer to IMEI
 */
char* CELL_GetIMEI(void) {
  if(IMEI[0] != '\0' || CELL_ReadIMEI(NULL)) {
    return IMEI;
  } else {
    return NULL;
  }
}

/**
 * @brief Read IMSI from SIM card
 *
 * @param pIMSI
 *
 * @return true if success
 */
bool CELL_ReadIMSI(uint8_t* pIMSI) {
  DBG_D("CELL: Reading imsi...\r\n");

  bool succeed = false;
  TickType_t start = xTaskGetTickCount();
  while(!util_timeout(start, xTaskGetTickCount(), IMSI_TIMEOUT)) {
    if(modem->ops->imsi(modem, (char*)IMSI, sizeof(IMSI)) == 0) {
      succeed = true;
      if(pIMSI != NULL) {
        memcpy(pIMSI, IMSI, sizeof(IMSI));
      }
      break;
    } else {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  data.sim_time = hal_rtc_get_time() - data.time;
  if(succeed) {
    DBG_I("CELL: Read imsi succeed in %d seconds: %s\r\n", (xTaskGetTickCount() - start) / 1000, IMSI);
  } else {
    DBG_E("CELL: Read imsi failed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
    data.err_flag |= CELL_ERR_SIM;
  }

  return succeed;
}

/**
 * @brief Get the IMSI string
 *
 * @return pointer to IMSI
 */
char* CELL_GetIMSI(void) {
  if(IMSI[0] != '\0' || CELL_ReadIMSI(NULL)) {
    return IMSI;
  } else {
    return NULL;
  }
}

/**
 * @brief Read ICCID
 *
 * @param pICCID
 *
 * @return true if success
 */
bool CELL_ReadICCID(uint8_t* pICCID) {
  DBG_D("CELL: Reading iccid...\r\n");

  bool succeed = false;
  TickType_t start = xTaskGetTickCount();
  while(!util_timeout(start, xTaskGetTickCount(), ICCID_TIMEOUT)) {
    if(modem->ops->iccid(modem, (char*)ICCID, sizeof(ICCID)) == 0) {
      succeed = true;
      if(pICCID != NULL) {
        memcpy(pICCID, ICCID, sizeof(ICCID));
      }
      break;
    } else {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  if(succeed) {
    DBG_I("CELL: Read iccid succeed in %d seconds: %s\r\n", (xTaskGetTickCount() - start) / 1000, ICCID);
  } else {
    DBG_E("CELL: Read iccid failed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
  }

  return succeed;
}

/**
 * @brief Get the ICCID string
 *
 * @return pointer to ICCID
 */
char* CELL_GetICCID(void) {
  if(ICCID[0] != '\0' || CELL_ReadICCID(NULL)) {
    return ICCID;
  } else {
    return NULL;
  }
}

/**
 * @brief Write own number to SIM card
 *
 * @param pNUM
 *
 * @return true if success
 */
bool CELL_WriteNUM(uint8_t* pNUM) {
  DBG_D("CELL: Writing number...\r\n");

  bool succeed = false;
  TickType_t start = xTaskGetTickCount();
  while(!util_timeout(start, xTaskGetTickCount(), WNUM_TIMEOUT)) {
    if(modem->ops->onum(modem, (char*)pNUM) == 0) {
      modem->ops->cnum(modem, (char*)NUM, sizeof(NUM));
      succeed = true;
      break;
    } else {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  if(succeed) {
    DBG_I("CELL: Write number succeed in %d seconds: %s\r\n", (xTaskGetTickCount() - start) / 1000, NUM);
  } else {
    DBG_E("CELL: Write number failed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
  }

  return succeed;
}

/**
 * @brief Read own number from SIM card
 *
 * @param pNUM
 *
 * @return true if success
 */
bool CELL_ReadNUM(uint8_t* pNUM) {
  DBG_D("CELL: Reading number...\r\n");

  bool succeed = false;
  TickType_t start = xTaskGetTickCount();
  while(!util_timeout(start, xTaskGetTickCount(), RNUM_TIMEOUT)) {
    if(modem->ops->cnum(modem, (char*)NUM, sizeof(NUM)) == 0) {
      succeed = true;
      if(pNUM != NULL) {
        memcpy(pNUM, NUM, sizeof(NUM));
      }
      break;
    } else {
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  }

  data.num_time = hal_rtc_get_time() - data.time;
  if(succeed) {
    DBG_I("CELL: Read number succeed in %d seconds: %s\r\n", (xTaskGetTickCount() - start) / 1000, NUM);
  } else {
    DBG_E("CELL: Read number failed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
    data.err_flag |= CELL_ERR_NUM;
  }

  return succeed;
}

/**
 * @brief Get the own number string
 *
 * @return pointer to own number
 */
char* CELL_GetNUM(void) {
  if(NUM[0] != '\0' || CELL_ReadNUM(NULL)) {
    return NUM;
  } else {
    return NULL;
  }
}

/**
 * @brief Send SMS
 *
 * @param pNum
 * @param pMsg
 * @param len
 *
 * @return ture if success
 */
bool CELL_SMSSend(char* num, char* pMsg, uint8_t len) {
  DBG_D("CELL: Sending sms...\r\n");
  if(num == NULL || pMsg == NULL || len <= 0 || len > 160) {
    return false;
  }

#ifdef DBG_ENABLE
  TickType_t start = xTaskGetTickCount();
#endif
  bool succeed = (modem->ops->sms(modem, num, pMsg, len) == 0);

  data.sms_time = hal_rtc_get_time() - data.time;
  if(succeed) {
    DBG_I("CELL: Send sms succeed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
  } else {
    DBG_W("CELL: Send sms failed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
    data.err_flag |= CELL_ERR_SMS;
  }

  return succeed;
}

/**
 * @brief Check CSQ
 *
 * @return true if success
 */
bool CELL_CheckCSQ(int32_t level) {
  DBG_D("CELL: Checking csq...\r\n");

  int rssi = -1;
  bool succeed = false;
  TickType_t start = xTaskGetTickCount();
  while(!util_timeout(start, xTaskGetTickCount(), RSSI_TIMEOUT)) {
    rssi = CELL_GetCSQ();
    if(rssi <= -2) {
      break;
    } else if(rssi < 0) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    } else {
      rssi &= 0xffff;
      if(rssi >= level && rssi <= 31) {
        succeed = true;
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  if(succeed) {
    DBG_I("CELL: Check csq succeed in %d seconds: (%d, %d)\r\n", (xTaskGetTickCount() - start) / 1000, rssi & 0xffff, rssi >> 16);
  } else {
    DBG_W("CELL: Check csq failed in %d seconds: (%d, %d)\r\n", (xTaskGetTickCount() - start) / 1000, rssi & 0xffff, rssi >> 16);
  }

  return succeed;
}

/**
 * @brief Update APN
 *
 * @return true if success
 */
bool CELL_UpdateAPN(const char* apn) {
  DBG_D("CELL: Updating apn...\r\n");

  bool succeed = false;
  if(apn != NULL) {
    modem->apn = apn;
    succeed = true;
  } else {
    char *imsi = CELL_GetIMSI();
    if(imsi != NULL) {
      if(!memcmp(imsi, "46004", strlen("46004"))) {
        modem->apn = "cmiot";
        succeed = true;
      }
      else if(!memcmp(imsi, "46002", strlen("46002"))) {
        modem->apn = "cmnet";
        succeed = true;
      } else if(!memcmp(imsi, "46001", strlen("46001"))) {
        char *num = CELL_GetNUM();
        if(num != NULL && !memcmp(num, "4600102", strlen("4600102"))) {
          modem->apn = "unim2m.njm2mapn";
        } else {
          modem->apn = "uninet";
        }
        succeed = true;
      }
    }
  }

  if(succeed) {
    DBG_I("CELL: Update apn succeed: %s\r\n", modem->apn);
  } else {
    DBG_W("CELL: Update apn failed\r\n");
  }

  return succeed;
}

/**
 * @brief Open pdp context
 *
 * @return true if success
 */
bool CELL_OpenPDP(void) {
  DBG_D("CELL: Opening pdp...\r\n");
#ifdef DBG_ENABLE
  TickType_t start = xTaskGetTickCount();
#endif
  int succeed = modem->ops->pdp_open(modem, modem->apn) == 0;

  data.pdp_time = hal_rtc_get_time() - data.time;
  if(succeed) {
    DBG_I("CELL: Open pdp succeed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
  } else {
    DBG_W("CELL: Open pdp failed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
    data.err_flag |= CELL_ERR_PDP;
  }

  return succeed;
}

/**
 * @brief Connect server
 *
 * @param connid
 * @param pHost
 * @param port
 *
 * @return true if success
 */
int CELL_Connect(char* pHost, uint16_t port) {
  DBG_D("CELL: Connecting...\r\n");

#ifdef DBG_ENABLE
  TickType_t start = xTaskGetTickCount();
#endif
  int connid = modem->ops->socket_connect(modem, pHost, port);

  data.con_time = hal_rtc_get_time() - data.time;
  if(connid >= 0) {
    DBG_I("CELL: connect succeed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
  } else {
    DBG_W("CELL: connect failed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
    data.err_flag |= CELL_ERR_CON;
  }

  return connid;
}

/**
 * @brief Send data
 *
 * @param connid
 * @param data
 * @param len
 *
 * @return true if success
 */
int CELL_Send(int connid, char* data, int len) {
  DBG_D("CELL: sending data...\r\n");
  if(!modem){
    return -2;}
  int ret = modem->ops->socket_send(modem, connid, data, len, 0);

  if(ret == len) {
    DBG_I("CELL: send data succeed\r\n");
  } else {
    DBG_W("CELL: send data failed %d\r\n",ret);
    ret = -1;
  }

  return ret;
}

/**
 * @brief Wait ack
 *
 * @param connid
 *
 * @return true if success
 */
bool CELL_Waitack(int connid) {
  DBG_D("CELL: Waiting ack...\r\n");

#ifdef DBG_ENABLE
  TickType_t start = xTaskGetTickCount();
#endif
  bool succeed = (modem->ops->socket_waitack(modem, connid) == 0);

  if(succeed) {
    DBG_I("CELL: wait ack succeed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
  } else {
    DBG_W("CELL: wait ack failed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
  }

  return succeed;
}

/**
 * @brief Receive data
 *
 * @param connid
 * @param data
 * @param len
 *
 * @return true if success
 */
int CELL_Recv(int connid, char* data, int len) {
  DBG_V("CELL: Receiving data...\r\n");
  return modem->ops->socket_recv(modem, connid, data, len, 0);
}

/**
 * @brief Close socket
 *
 * @param connid
 *
 * @return true if success
 */
bool CELL_Close(int connid) {
  DBG_D("CELL: Disconnecting...\r\n");
  return modem->ops->socket_close(modem, connid) == 0;
}

/**
 * @brief Get CSQ of the CELL module
 *
 * @return The CSQ
 */
int CELL_GetCSQ(void) {
  return modem->ops->rssi(modem);
}

/**
 * @brief Get network code of the CELL module
 *
 * @return The network code
 */
int CELL_GetNC(void) {
  return modem->ops->cops(modem);
}

/**
 * @brief Register network
 *
 * @return true if success
 */
bool CELL_Register(void) {
  if(CELL_GetIMSI() == NULL) {
    return false;
  }

  DBG_D("CELL: Registering...\r\n");
  int errcnt = 0;
  bool succeed = false;
#ifdef DBG_ENABLE
  TickType_t start = xTaskGetTickCount();
#endif
  for(;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));

    int csq = modem->ops->rssi(modem);
    if(csq <= -2) { // No response
      data.err_flag |= CELL_ERR_CSQ | CELL_ERR_RST;
      break;
    } else if(csq < 0) { // Wrong response
      if(++errcnt >= CMD_RETRIES) {
        data.err_flag |= CELL_ERR_CSQ | CELL_ERR_CMD;
        break;
      }
    } else {
      int rssi = csq & 0xffff;
      int ber = csq >> 16;
      if(rssi == 99) { // Unknown
        if(util_timeout(startTime, xTaskGetTickCount(), CREG_TIMEOUT)) {
          data.err_flag |= CELL_ERR_CSQ;
          data.rssi = rssi;
          data.ber = ber;
          break;
        }
      } else if(rssi >= CELL_MIN_RSSI) { // TODO: consider ber
        if(data.csq_time == 0) {
          DBG_I("CELL: Check csq succeed in %d seconds: (%d, %d)\r\n", (xTaskGetTickCount() - start) / 1000, rssi, ber);
          data.csq_time = hal_rtc_get_time() - data.time;
          data.rssi = rssi;
          data.ber = ber;
#ifdef DBG_ENABLE
          start = xTaskGetTickCount();
#endif
        }
      } else if(util_timeout(startTime, xTaskGetTickCount(), RSSI_TIMEOUT)) {
        data.err_flag |= CELL_ERR_CSQ;
        data.rssi = rssi;
        data.ber = ber;
        break;
      }
    }

    if(data.csq_time) {
      int creg = modem->ops->creg(modem);
      if(creg <= -2) {
        data.err_flag |= CELL_ERR_REG | CELL_ERR_RST;
        break;
      } else if(creg < 0) {
        if(++errcnt >= CMD_RETRIES) {
          data.err_flag |= CELL_ERR_REG | CELL_ERR_CMD;
          break;
        }
      } else if(creg == CREG_REGISTERED_HOME || creg == CREG_REGISTERED_ROAMING) {
        DBG_W("CELL: Creg succeed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
        data.reg_time = hal_rtc_get_time() - data.time;
        int mnc = CELL_GetNC();
        data.mnc = mnc & 0xffffff;
        data.rat = mnc >> 24;
        break;
      }
      if(util_timeout(startTime, xTaskGetTickCount(), CREG_TIMEOUT)) {
        data.err_flag |= CELL_ERR_REG;
        break;
      }
    }
  }

  if(data.err_flag & CELL_ERR_CSQ) {
    DBG_W("CELL: Check csq failed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
    data.csq_time = hal_rtc_get_time() - data.time;
  } else if(data.err_flag & CELL_ERR_REG){
    DBG_W("CELL: Creg failed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
    data.reg_time = hal_rtc_get_time() - data.time;
  } else {
    succeed = CELL_GetNUM();
  }

  if(succeed) {
    DBG_I("CELL: Register succeed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
  } else {
    DBG_W("CELL: Register failed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
  }

  return succeed;
}

/**
 * @brief Prepare for data service
 *
 * @return true if success
 */
bool CELL_Prepare(void) {
  DBG_D("CELL: Preparing...\r\n");

  int errcnt = 0;
  bool succeed = false;
#ifdef DBG_ENABLE
  TickType_t start = xTaskGetTickCount();
#endif
  for(;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));

    int cgatt = modem->ops->cgatt(modem);
    if(cgatt <= -2) {
      data.err_flag |= CELL_ERR_ATT | CELL_ERR_RST;
      break;
    } else if(cgatt < 0) {
      if(modem->ops->imsi(modem, (char*)IMSI, sizeof(IMSI)) == -1) { // Card error
        data.err_flag |= CELL_ERR_ATT | CELL_ERR_SIM;
        break;
      }
      if(++errcnt >= CMD_RETRIES) { // Command error
        data.err_flag |= CELL_ERR_ATT | CELL_ERR_CMD;
        break;
      }
    } else if(cgatt == 1) {
      DBG_I("CELL: Attach succeed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
      data.att_time = hal_rtc_get_time() - data.time;
      break;
    }

    if(util_timeout(startTime, xTaskGetTickCount(), CGATT_TIMEOUT)) {
      data.err_flag |= CELL_ERR_ATT;
      break;
    }
  }

  if(data.err_flag & CELL_ERR_ATT) {
    DBG_W("CELL: Attach failed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
    data.att_time = hal_rtc_get_time() - data.time;
  } else {
    succeed = CELL_UpdateAPN(NULL) && CELL_OpenPDP();
  }

  if(succeed) {
    DBG_I("CELL: Prepare succeed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
  } else {
    DBG_W("CELL: Prepare failed in %d seconds\r\n", (xTaskGetTickCount() - start) / 1000);
  }

  return succeed;
}

/**
 * @brief Get cellular data
 *
 * @return the data
 */
CellData_t* CELL_GetData(void) {
  data.tot_time = hal_rtc_get_time() - data.time;

  if(!modem || !modem->ops) {
    data.ext_flag |= CELL_ERR_HWT;
    return &data;
  }

  do {
    int ret = modem->ops->imsi(modem, (char*)IMSI, sizeof(IMSI));
    if(ret != 0) {
      data.ext_flag |= CELL_ERR_SIM;
    }
    if(ret <= -2) {
      data.ext_flag |= CELL_ERR_RST;
      break;
    }

    ret = modem->ops->cnum(modem, (char*)NUM, sizeof(NUM));
    if(ret != 0) {
      data.ext_flag |= CELL_ERR_NUM;
    }
    if(ret <= -2) {
      data.ext_flag |= CELL_ERR_RST;
      break;
    }

    ret = modem->ops->rssi(modem);
    if(ret < 0) {
      data.ext_flag |= CELL_ERR_CSQ;
    } else {
      data.ext_rssi = ret & 0xffff;
      data.ext_ber = ret >> 16;
    }
    if(ret <= -2) {
      data.ext_flag |= CELL_ERR_RST;
      break;
    }

    ret = modem->ops->creg(modem);
    if(ret <= -2) {
      data.ext_flag |= CELL_ERR_REG | CELL_ERR_RST;
      break;
    } else if(ret < 0) {
      data.ext_flag |= CELL_ERR_REG | CELL_ERR_CMD;
    } else if(ret != CREG_REGISTERED_HOME && ret != CREG_REGISTERED_ROAMING) {
      data.ext_flag |= CELL_ERR_REG;
    }

    ret = modem->ops->cgatt(modem);
    if(ret <= -2) {
      data.ext_flag |= CELL_ERR_ATT | CELL_ERR_RST;
      break;
    } else if(ret < 0) {
      data.ext_flag |= CELL_ERR_ATT | CELL_ERR_CMD;
    } else if(ret != 1) {
      data.ext_flag |= CELL_ERR_ATT;
    }
  } while(0);

  return &data;
}

/**
 * @brief Init cellular module
 *
 * @return true if success
 */
bool CELL_Init(void) {
  DBG_D("CELL: Initialize...\r\n");

  // Cleanup
  memset(&data, 0, sizeof(CellData_t));
  IMEI[0] = 0;
  IMSI[0] = 0;
  ICCID[0] = 0;
  NUM[0] = 0;
  // Start time
  startTime = xTaskGetTickCount();
  data.time = hal_rtc_get_time();

  if(at == NULL) {
    at = at_alloc_freertos();
  }

  if(modem == NULL) {
    modem = cellular_alloc();
  }

  bool succeed = false;
  if(at != NULL && modem != NULL) {
    at_open(at);
    //succeed = (cellular_attach(modem, at, "uninet") == 0);
    succeed = (cellular_attach(modem, at, "cmnet") == 0);
  }

  data.hwt_time = hal_rtc_get_time() - data.time;
  if(succeed) {
    DBG_I("CELL: Initialize succeed in %d seconds\r\n", (xTaskGetTickCount() - startTime) / 1000);
  } else {
    DBG_E("CELL: Initialize failed in %d seconds\r\n", (xTaskGetTickCount() - startTime) / 1000);
    data.err_flag |= CELL_ERR_HWT;
  }

  return succeed;
}

/**
 * @brief Init cellular module
 *
 * @return true if success
 */
bool CELL_Deinit(void) {
  DBG_D("CELL: Deinitialize...\r\n");

  if(modem != NULL) {
    if(modem->ops->shutdown(modem) == 0) {
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
    cellular_detach(modem);
    modem = NULL;
  }

  if(at != NULL) {
    at_free(at);
    at = NULL;
  }

  DBG_I("CELL: Deinitialize succeed\r\n");
  return true;
}

/**
 * @brief Shutdown the cellular module
 */
bool CELL_Shutdown(void) {
  CELL_Deinit();
  CELL_PowerOff();
  return true;
}

/**
 * @brief Reset the CELL module
 */
bool CELL_Reset(void) {
  CELL_Shutdown();
  vTaskDelay(pdMS_TO_TICKS(800));
  return CELL_PowerOn();
}

/**
 * @brief Suspend the cellular module
 *
 * @return true if success
 */
bool CELL_Suspend(void) {
  DBG_D("CELL: Suspending...\r\n");

  bool succeed = false;
  if(modem != NULL && modem->ops->suspend != NULL) {
    succeed = (modem->ops->suspend(modem) == 0);
  }

  if(succeed) {
    DBG_I("CELL: Suspend succeed\r\n");
  } else {
    DBG_E("CELL: Suspend fail\r\n");
  }

  return succeed;
}

/**
 * @brief Resume the cellular module
 *
 * @return true if success
 */
bool CELL_Resume(void) {
  DBG_D("CELL: Resuming...\r\n");

  bool succeed = false;
  if(modem != NULL && modem->ops->resume != NULL) {
    succeed = (modem->ops->resume(modem) == 0);
  }

  if(succeed) {
    DBG_I("CELL: Resume succeed\r\n");
  } else {
    DBG_E("CELL: Resume fail\r\n");
  }

  return succeed;
}
