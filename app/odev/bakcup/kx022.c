/*!
 *    @file  KX022.c
 *   @brief  Accelerometer Sensor based on KX022
 *
 *  @author  Dale.J (dj), Dale.J@zoho.com
 *
 *  @internal
 *       Created:  04/ 8/2016
 *      Revision:  none
 *  Organization:  Druid Tech
 *     Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */


/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "FreeRTOS_IO.h"
#include "semphr.h"
#include "acc.h"
#include "task.h"
#include "debug.h"
#include "string.h"
#include "kx022_reg.h"

/* Defines -------------------------------------------------------------------*/
DBG_SET_LEVEL(DBG_LEVEL_I);

#define LOCK_TIMEOUT        pdMS_TO_TICKS(100)
#define HW_TEST_RETRY       2         // 2
#define ACT_THRESH          80        // (32counts/g * 2.5g = 80)
#define ACT_TIME            12        // (12.5Hz * 1.0s = 12)

/* Private variables ---------------------------------------------------------*/
static Peripheral_Descriptor_t xSPI = NULL;
static SemaphoreHandle_t xSem = NULL;
static ACC_Callback_t wakeCB = NULL;
static ACC_Callback_t dataCB = NULL;

/* Private functions ---------------------------------------------------------*/
// Active CS
static void cs_low(void) {
  HAL_GPIO_WritePin(ACC_NSS_GPIO_Port, ACC_NSS_Pin, GPIO_PIN_RESET);
}
// Deactive CS
static void cs_high(void) {
  HAL_GPIO_WritePin(ACC_NSS_GPIO_Port, ACC_NSS_Pin, GPIO_PIN_SET);
}
// Read FIFO
static uint32_t read_fifo(uint8_t *buf, uint32_t len) {
  buf[0] = KX022_BUF_READ | KX022_READ;

  cs_low();
  FreeRTOS_write(xSPI, buf, 1);
  FreeRTOS_read(xSPI, buf, len * 2);
  cs_high();

  return len;
}
// Read register
static uint8_t read_reg(uint8_t addr) {
  uint8_t buf[2];

  buf[0] = addr | KX022_READ;

  cs_low();
  FreeRTOS_write(xSPI, buf, 1);
  FreeRTOS_read(xSPI, buf, 1);
  cs_high();

  return buf[0];
}
// Read registers
static uint8_t read_regs(uint8_t addr, uint8_t *buf, uint8_t len) {
  buf[0] = addr | KX022_READ;

  cs_low();
  FreeRTOS_write(xSPI, buf, 1);
  FreeRTOS_read(xSPI, buf, len);
  cs_high();

  return len;
}
// Write register
static void write_reg(uint8_t addr, uint8_t val) {
  uint8_t buf[2];

  buf[0] = addr | KX022_WRITE;
  buf[1] = val;

  cs_low();
  FreeRTOS_write(xSPI, buf, 2);
  cs_high();
}
//// Write two registers
//static void write_2regs(uint8_t addr, uint16_t val) {
//  uint8_t buf[4];

//  buf[0] = addr | KX022_WRITE;
//  buf[1] = val & 0xff;
//  buf[2] = val >> 8;

//  cs_low();
//  FreeRTOS_write(xSPI, buf, 3);
//  cs_high();
//}
//// Write registers
//static void write_regs(uint8_t addr, uint8_t *buf, uint8_t len) {
//  uint8_t tmp[1];

//  tmp[0] = addr | KX022_WRITE;

//  cs_low();
//  FreeRTOS_write(xSPI, tmp, 1);
//  FreeRTOS_write(xSPI, buf, len);
//  cs_high();
//}

/* Global functions ----------------------------------------------------------*/

/**
 * @brief Test the hardware connection
 *
 * @return true if success
 */
bool ACC_HardwareTest(void) {
  DBG_D("ACC: Hardware test\r\n");
  bool ret = false;

  if(!ACC_Lock()) {
    return false;
  }

  for(int i = 0; i < HW_TEST_RETRY; i++) {                    // check the devid register
    if(0x14 == read_reg(KX022_WHO_AM_I)) {
      ret = true;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(4));
  }

  if(!ret) {
    DBG_E("ACC: Hardware test failed\r\n");
  } else {
    // Weird, but it works! We have to map an interrupt to the INT pin, or that pin would be configured to a high-Z state
    // Yet, no interrupt is needed right now, so we just map any interrupt to the INT pin, without enabling it
    // TODO
/*     write_reg(KX022_INTMAP2, KX022_INT_INACT);                // Active high */
/* #if !(DEV_TYPE == 2 && HW_VERSION == 1) */
/*     write_reg(KX022_INTMAP1, KX022_INT_INACT);                // Active high */
/* #endif */
  }

  ACC_Unlock();
  return ret;
}

/**
 * @brief Software reset xl362
 *
 * @return true if success
 */
bool ACC_SoftwareReset(void) {
  DBG_D("ACC: Software reset\r\n");
  int ret = false;

  if(!ACC_Lock()) {
    return false;
  }

  write_reg(KX022_CNTL2, KX022_SOFT_RESET);
  vTaskDelay(pdMS_TO_TICKS(4));

  if(0x14 == read_reg(KX022_WHO_AM_I)) {
    ret = true;
  }

  ACC_Unlock();
  return ret;
}

/**
 * @brief Power cycle xl362
 *
 * @return true if success
 */
bool ACC_HardwareReset(void) {
  DBG_D("ACC: Hardware reset\r\n");
  int ret = false;

  if(!ACC_Lock()) {
    return false;
  }

  write_reg(KX022_CNTL2, KX022_SOFT_RESET);
  vTaskDelay(pdMS_TO_TICKS(4));

  if(0x14 == read_reg(KX022_WHO_AM_I)) {
    ret = true;
  }

  ACC_Unlock();
  return ret;
}

/**
 * @brief Enable shake detection
 *
 * @return true if success
 */
bool ACC_EnableShake(ACC_Callback_t cb) {
  DBG_D("ACC: Enabling shake detection\r\n");

  if(!ACC_HardwareTest()) {
    return false;
  }

  if(!ACC_Lock()) {
    return false;
  }

  write_reg(KX022_CNTL1, KX022_RANGE_4G | KX022_WAKEUP_EN);
  write_reg(KX022_CNTL3, KX022_WUF_ODR_12);
  write_reg(KX022_WUFC, ACT_TIME);
  write_reg(KX022_ATH, ACT_THRESH);
  write_reg(KX022_INC1, KX022_INT_EN | KX022_INT_ACTV_HIGH | KX022_INT_LATCH_ON);
  write_reg(KX022_INC4, KX022_INT_WAKEUP);
  write_reg(KX022_CNTL1, KX022_POWER_ON | KX022_RANGE_4G | KX022_WAKEUP_EN);

  wakeCB = cb;
  ACC_Unlock();
  return true;
}

/**
 * @brief Enable sample (for motion detection)
 *
 * @return true if success
 */
bool ACC_EnableSample(ACC_Callback_t cb) {
  DBG_D("ACC: Enabling motion detection\r\n");

  if(!ACC_HardwareTest()) {
    return false;
  }

  if(!ACC_Lock()) {
    return false;
  }

  write_reg(KX022_CNTL1, KX022_RANGE_4G);
  write_reg(KX022_ODCNTL, KX022_ODR_25 | KX022_LP_FILTER_M2);
  write_reg(KX022_BUF_CNTL1, ACC_FIFO_WATERMARK);
  write_reg(KX022_BUF_CNTL2, KX022_BUF_EN | KX022_BUF_16BITS | KX022_BUF_MODE_STREAM);
  write_reg(KX022_INC1, KX022_INT_EN | KX022_INT_ACTV_HIGH | KX022_INT_LATCH_ON);
  write_reg(KX022_INC4, KX022_INT_WATERMARK);
  write_reg(KX022_CNTL1, KX022_POWER_ON | KX022_RANGE_4G);

  dataCB = cb;
  ACC_Unlock();
  return true;
}

/**
 * @brief Read the status register
 *
 * @param pStatus
 *
 * @return true if success
 */
bool ACC_ReadStatus(uint8_t *pStatus) {
  if(!ACC_Lock()) {
    return false;
  }

  if(0x14 != read_reg(KX022_WHO_AM_I)) {
    ACC_Unlock();
    return false;
  }

  *pStatus = read_reg(KX022_INS2);
  read_reg(KX022_INT_REL);

  ACC_Unlock();
  return true;
}

/**
 * @brief Read data
 *
 * @return true if success
 */
bool ACC_ReadData(int16_t* buf) {
  DBG_V("ACC: Reading data\r\n");
  memset(buf, 0, sizeof(buf[0]) * 3);

  if(!ACC_Lock()) {
    return false;
  }

  read_regs(KX022_XOUTL, (uint8_t*)buf, sizeof(buf[0]) * 3);
  ACC_Unlock();

  return true;
}

/**
 * @brief Clear FIFO
 *
 * @return true if success
 */
bool ACC_ClearFIFO(void) {
  DBG_V("ACC: Clearing FIFO\r\n");

  if(!ACC_Lock()) {
    return false;
  }
  write_reg(KX022_BUF_CLEAR, 0);

  ACC_Unlock();
  return true;
}

/**
 * @brief Read FIFO
 *
 * @return true if success
 */
bool ACC_ReadFIFO(int16_t* buf, int32_t *cnt) {
  DBG_V("ACC: Reading FIFO\r\n");

  uint16_t read = *cnt;
  uint8_t len = 0;

  // Default output
  *cnt = 0;

  // Check output buffer size
  if(read % 3 != 0 || read < 3) {
    return false;
  }

  // Lock
  if(!ACC_Lock()) {
    return false;
  }

  // Check data count
  read_regs(KX022_BUF_STATUS1, &len, 1);
  len /= 2;
  if(len < 3) {
    ACC_Unlock();
    return false;
  }

  // Check output buffer
  if(read > len) {
    read = len;
  }

  // Read data
  read_fifo((uint8_t*)buf, read);
  ACC_Unlock();

  for(int i = 0; i < read; i++) {
    buf[i] /= 8;
  }

  *cnt = read;
  return true;
}

/**
 * @brief Lock for an operation
 *
 * @return true if success
 */
bool ACC_Lock(void) {
  if(xSem == NULL) {
    xSem = xSemaphoreCreateBinary();
    xSemaphoreGive(xSem);
  }

  if(xSem != NULL && xSemaphoreTake(xSem, LOCK_TIMEOUT)) {
    if(xSPI == NULL) {
      xSPI = FreeRTOS_open(boardACC_METER_SPI, 0);
    }
    if(xSPI != NULL) {
      return true;
    }
    xSemaphoreGive(xSem);
  }

  DBG_W("ACC: Lock failed\r\n");
  return false;
}

/**
 * @brief Unlock
 *
 * @return true if success
 */
bool ACC_Unlock(void) {
  if(xSem != NULL) {
    if(xSPI != NULL) {
      FreeRTOS_close(xSPI);
      xSPI = NULL;
    }
    if(xSemaphoreGive(xSem)) {
      return true;
    }
  }

  DBG_E("ACC: Unlock failed\r\n");
  return false;
}

/**
 * @brief Interrupter handler
 *
 * @return None
 */
void ACC_InterruptHandler(void) {
  uint8_t status = 0;
  ACC_ReadStatus(&status);
  if((status & KX022_INT_WAKEUP) != 0 && wakeCB != NULL) {
    wakeCB();
  }
  if((status & KX022_INT_WATERMARK) != 0 && dataCB != NULL) {
    dataCB();
  }
}
