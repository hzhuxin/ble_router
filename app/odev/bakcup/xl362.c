/*!
 *    @file  xl362.c
 *   @brief  Accelerometer Sensor based on XL362
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
#include "xl362_reg.h"

/* Defines -------------------------------------------------------------------*/
DBG_SET_LEVEL(DBG_LEVEL_I);
#define MODE_MASK           0x0307    // Mode Mask
#define MODE_IDLE           0x0000    // Standby
#define MODE_MOTION         0x0200    // 12.5Hz (Motion detection)
#define MODE_SHAKE          0x0201    // 25Hz (Shake detection)

#define LOCK_TIMEOUT        pdMS_TO_TICKS(100)
#define HW_TEST_RETRY       2         // 2
#define ACT_THRESH          2000      // (1024 * 1.95g = 2000)
#define ACT_TIME            15        // (25Hz * 0.6S = 15)

/* Private variables ---------------------------------------------------------*/
static Peripheral_Descriptor_t xSPI = NULL;
static SemaphoreHandle_t xSem = NULL;
static ACC_Callback_t wakeCB = NULL;
static ACC_Callback_t fifoCB = NULL;

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
  buf[0] = XL362_FIFO_READ;

  cs_low();
  FreeRTOS_write(xSPI, buf, 1);
  FreeRTOS_read(xSPI, buf, len * 2);
  cs_high();

  return len;
}
// Read register
static uint8_t read_reg(uint8_t addr) {
  uint8_t buf[4];

  buf[0] = XL362_REG_READ;
  buf[1] = addr;

  cs_low();
  FreeRTOS_write(xSPI, buf, 2);
  FreeRTOS_read(xSPI, buf, 1);
  cs_high();

  return buf[0];
}
// Read registers
static uint8_t read_regs(uint8_t addr, uint8_t *buf, uint8_t len) {
  buf[0] = XL362_REG_READ;
  buf[1] = addr;

  cs_low();
  FreeRTOS_write(xSPI, buf, 2);
  FreeRTOS_read(xSPI, buf, len);
  cs_high();

  return len;
}
// Write register
static void write_reg(uint8_t addr, uint8_t val) {
  uint8_t buf[4];

  buf[0] = XL362_REG_WRITE;
  buf[1] = addr;
  buf[2] = val;

  cs_low();
  FreeRTOS_write(xSPI, buf, 3);
  cs_high();
}
// Write two registers
static void write_2regs(uint8_t addr, uint16_t val) {
  uint8_t buf[4];

  buf[0] = XL362_REG_WRITE;
  buf[1] = addr;
  buf[2] = val & 0xff;
  buf[3] = val >> 8;

  cs_low();
  FreeRTOS_write(xSPI, buf, 4);
  cs_high();
}
//// Write registers
//static void write_regs(uint8_t addr, uint8_t *buf, uint8_t len) {
//  uint8_t tmp[4];

//  tmp[0] = XL362_REG_WRITE;
//  tmp[1] = addr;

//  cs_low();
//  FreeRTOS_write(xSPI, tmp, 2);
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
    if(0xAD == read_reg(XL362_DEVID_AD)) {
      ret = true;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  if(!ret) {
    DBG_E("ACC: Hardware test failed\r\n");
  } else {
    // Weird, but it works! We have to map an interrupt to the INT pin, or that pin would be configured to a high-Z state
    // Yet, no interrupt is needed right now, so we just map any interrupt to the INT pin, without enabling it
    write_reg(XL362_INTMAP2, XL362_INT_INACT);                // Active high
#if !(DEV_TYPE == 2 && HW_VERSION == 1)
    write_reg(XL362_INTMAP1, XL362_INT_INACT);                // Active high
#endif
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
  if(!ACC_Lock()) {
    return false;
  }

  write_reg(XL362_SOFT_RESET, XL362_SOFT_RESET_KEY);
  vTaskDelay(pdMS_TO_TICKS(2));

  ACC_Unlock();
  return true;
}

/**
 * @brief Power cycle xl362
 *
 * @return true if success
 */
bool ACC_HardwareReset(void) {
  DBG_D("ACC: Hardware reset\r\n");

  for(int i = 0; i < 2; i++) {
    // Pull down the power pin (AND spi_cs pin)
    HAL_GPIO_WritePin(ACC_VDD_GPIO_Port, ACC_VDD_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACC_NSS_GPIO_Port, ACC_NSS_Pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(5));

    // Release power pin (AND spi_cs pin)
    HAL_GPIO_WritePin(ACC_NSS_GPIO_Port, ACC_NSS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ACC_VDD_GPIO_Port, ACC_VDD_Pin, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(10));

    if(ACC_HardwareTest()) {
      DBG_I("ACC: Hardware reset succeed\r\n");
      return true;
    }
  }

  DBG_E("ACC: Hardware reset failed\r\n");
  return false;
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

  uint16_t reg = 0;
  read_regs(XL362_FILTER_CTL, (uint8_t*)&reg, sizeof(reg));
  if((reg & MODE_MASK) != MODE_SHAKE) {
    write_reg(XL362_FILTER_CTL, XL362_RANGE_2G | XL362_RATE_25 | XL362_HALF_BW);      // 25Hz, +/-2g
    write_2regs(XL362_THRESH_ACTL, ACT_THRESH);                                       // Active Threshold
    write_reg(XL362_TIME_ACT, ACT_TIME);                                              // Active Time
    write_reg(XL362_ACT_INACT_CTL, XL362_ACT_ENABLE | XL362_ACT_DC);                  // Enable Active detect, Absolute mode
    write_reg(XL362_INTMAP2, XL362_INT_ACT);                                          // Enable Act level INT, active high
    write_reg(XL362_POWER_CTL, XL362_LOW_POWER | XL362_MEASURE_3D);                   // Start Measure
  }

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

  uint16_t reg = 0;
  read_regs(XL362_FILTER_CTL, (uint8_t*)&reg, sizeof(reg));
  if((reg & MODE_MASK) != MODE_MOTION) {
    write_reg(XL362_FILTER_CTL, XL362_RANGE_2G | XL362_RATE_25 | XL362_HALF_BW);      // 25Hz, +/-2g
    write_reg(XL362_FIFO_SAMPLES, (ACC_FIFO_WATERMARK * 3) & 0xff);                   // FIFO level
    write_reg(XL362_FIFO_CONTROL, XL362_FIFO_MODE_STREAM | XL362_FIFO_SAMPLES_AH);    // FIFO stream mode
    write_reg(XL362_INTMAP2, XL362_INT_FIFO_WATERMARK);                               // Enable FIFO watermark interrupt, active high
    write_reg(XL362_POWER_CTL, XL362_LOW_POWER | XL362_MEASURE_3D);                   // Start Measure
  }

  fifoCB = cb;
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

  if(0xAD != read_reg(XL362_DEVID_AD)) {
    ACC_Unlock();
    return false;
  }

  *pStatus = read_reg(XL362_STATUS);

  ACC_Unlock();
  return true;
}

/**
 * @brief Read data
 *
 * @return true if success
 */
bool ACC_ReadData(int16_t* buf) {
  memset(buf, 0, sizeof(buf[0]) * 3);
  DBG_V("ACC: Reading data\r\n");

  if(!ACC_Lock()) {
    return false;
  }

  read_regs(XL362_XDATAL, (uint8_t*)buf, sizeof(buf[0]) * 3);
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
  write_reg(XL362_FIFO_CONTROL, XL362_FIFO_MODE_OFF);
  write_reg(XL362_FIFO_CONTROL, XL362_FIFO_MODE_STREAM | XL362_FIFO_SAMPLES_AH);

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
  uint16_t len = 0;
  int i = 0;

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
  read_regs(XL362_FIFO_ENTRIES_L, (uint8_t*)&len, 2);
  len &= 0x3FF;
  if(len < 3) {
    ACC_Unlock();
    return false;
  }

  // Check output buffer
  if(read > len) {
    read = len - (len % 3);
  }
  len = read - 2;

  // Read data
  read_fifo((uint8_t*)buf, len);
  switch(buf[len - 1] & XL362_FIFO_DATA_AXIS_MASK) {
    case XL362_FIFO_DATA_AXIS_X:
      read_fifo((uint8_t*)&buf[len], 2);
      len += 2;
      break;
    case XL362_FIFO_DATA_AXIS_Y:
      read_fifo((uint8_t*)&buf[len], 1);
      len += 1;
      break;
    default:
      break;
  }
  ACC_Unlock();

  for(i = 0; i < 3; i++) {
    if((buf[i] & XL362_FIFO_DATA_AXIS_MASK) == XL362_FIFO_DATA_AXIS_X
        && (buf[i + 1] & XL362_FIFO_DATA_AXIS_MASK) == XL362_FIFO_DATA_AXIS_Y
        && (buf[i + 2] & XL362_FIFO_DATA_AXIS_MASK) == XL362_FIFO_DATA_AXIS_Z) {
          break;
    }
  }
  if(i >= 3) {
    return false;
  }

  if(i > 0) {
    len -= i;
    memcpy(buf, &buf[i], len * 2);
  }
  for(int i = 0; i < len; i++) {
    buf[i] <<= 2;
    buf[i] /= 4;
  }

  *cnt = len;
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
  if((status & XL362_INT_ACT) != 0 && wakeCB != NULL) {
    wakeCB();
  }
  if((status & XL362_INT_FIFO_WATERMARK) != 0 && fifoCB != NULL) {
    fifoCB();
  }
}
