/*!
 *    @file  W25Q.c
 *   @brief  Flash driver over spi interface
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
#include "semphr.h"
#include "debug.h"
#include "w25q.h"
#include "nrfx_log.h"
#include "hal_cfg.h"
#include "hal_spi.h"
#include "hal_pin.h"

/* Defines -------------------------------------------------------------------*/
DBG_SET_LEVEL(DBG_LEVEL_I);

#define MAX_READY_WAIT_COUNT        500000
#define MAX_COMMAND_SEND_COUNT      10
#define LOCK_TIMEOUT                pdMS_TO_TICKS(2000)

/* Private variables ---------------------------------------------------------*/
static hal_spi_t* spi = NULL;
static uint32_t W25Q_CHIP_SIZE = 0;

/* Private functions ---------------------------------------------------------*/
// Deactive CS
static void cs_high(void) {
  hal_pin_set_mode_ex(HAL_CFG_FLS_NSS, HAL_PIN_MODE_OUT, HAL_PIN_MODE_EX_NORMAL, HAL_PIN_LVL_HIGH);
}
// Active CS
static void cs_low(void) {
  hal_pin_set_mode_ex(HAL_CFG_FLS_NSS, HAL_PIN_MODE_OUT, HAL_PIN_MODE_EX_NORMAL, HAL_PIN_LVL_LOW);
}
// Read status
static bool read_status_reg(uint8_t *pStatus) {
  bool ret = false;
  uint8_t cmd = READ_STATUS_REG;

  cs_low();
  if(spi->ops->write(spi, &cmd, 1) == 1
     && spi->ops->read(spi, pStatus, 1) == 1) {
    ret = true;
  }
  cs_high();

  return ret;
}
// Wait till ready
static bool wait_till_ready(void) {
  uint32_t statusReadCount = MAX_READY_WAIT_COUNT;
  while(statusReadCount--)
  {
    uint8_t status = 0;
    if(!read_status_reg(&status)) {
      break;
    }
    if((status & STATUS_BUSY) == 0) {
      return true;
    }
  }

  DBG_E("W25Q: Wait ready failed\r\n");
  return false;
}
//// Read Device ID
//bool read_dev_id(uint16_t *pID) {
//  if(wait_till_ready()) {
//    uint8_t buf[4];

//    buf[0] = MAN_DEV_ID;              // Command to read read Manufacturer Id, Device ID
//    buf[1] = 0;
//    buf[2] = 0;
//    buf[3] = 0;                       // dummy transactions

//    cs_low();
//    spi->ops->write(spi, buf, 4);
//    spi->ops->read(spi, buf, 2);
//    cs_high();

//    *pID = *(uint16_t*)buf;
//    return true;
//  }

//  return false;
//}
// Write Enable
static bool write_enable(void) {
  if(wait_till_ready())
  {
    uint32_t commandSendCount = MAX_COMMAND_SEND_COUNT;
    while(commandSendCount--)
    {
      uint8_t cmd = WRITE_ENABLE;

      cs_low();
      spi->ops->write(spi, &cmd, 1);
      cs_high();

      uint32_t statusReadCount = MAX_READY_WAIT_COUNT;
      while(statusReadCount--)
      {
        uint8_t status = 0;
        if(!read_status_reg(&status)) {
          continue;
        }
        if(((status & STATUS_BUSY) == 0) &&((status & STATUS_WEL) != 0))
        {
          return true;
        }
      }
    }
  }

  DBG_E("W25Q: Write enable failed\r\n");
  return false;
}
//// Write enable volatile
//static bool write_enable_volatile(void) {
//  if(wait_till_ready())
//  {
//    uint32_t commandSendCount = MAX_COMMAND_SEND_COUNT;
//    while(commandSendCount--)
//    {
//      uint8_t cmd = WRITE_ENABLE_VOL;

//      cs_low();
//      spi->ops->write(spi, &cmd, 1);
//      cs_high();

//      uint32_t statusReadCount = MAX_READY_WAIT_COUNT;
//      while(statusReadCount--)
//      {
//        uint8_t status = 0;
//        if(!read_status_reg(&status)) {
//          continue;
//        }
//        if(((status & STATUS_BUSY) == 0) && ((status & STATUS_WEL) != 0))
//        {
//          return true;
//        }
//      }
//    }
//  }
//  return false;
//}
//// Write disable
//static bool write_disable(void)
//{
//  if(wait_till_ready())
//  {
//    uint32_t commandSendCount = MAX_COMMAND_SEND_COUNT;
//    while(commandSendCount--)
//    {
//      uint8_t cmd = WRITE_DISABLE;

//      cs_low();
//      spi->ops->write(spi, &cmd, 1);
//      cs_high();

//      uint32_t statusReadCount = MAX_READY_WAIT_COUNT;
//      while(statusReadCount--)
//      {
//        uint8_t status = 0;
//        if(!read_status_reg(&status)) {
//          continue;
//        }
//        if(((status & STATUS_BUSY) == 0) && ((status & STATUS_WEL) == 0))
//        {
//          return true;
//        }
//      }
//    }
//  }
//  return false;
//}
//// Write status
//static bool write_status_reg(uint8_t data)
//{
//  if(wait_till_ready()) {
//    uint8_t buf[2];

//    buf[0] = WRITE_STATUS_REG;
//    buf[1] = data;

//    cs_low();
//    spi->ops->write(spi, buf, 2);
//    cs_high();
//  } else {
//    return false;
//  }

//  return wait_till_ready();
//}

// Page program
static bool page_program(uint32_t address, uint16_t size, uint8_t *pData)
{
  uint16_t ret = false;
  uint16_t offset = address & W25Q_PAGE_MASK;     // offset in the page

  if(size + offset <= W25Q_PAGE_SIZE)              // check for max page size
  {
    if(write_enable()) {
      uint8_t buf[4];
      buf[0] = PAGE_PROGRAM;                      // Command for page programming
      buf[1] =(address >> 16) & 0xff;
      buf[2] =(address >> 8) & 0xff;
      buf[3] = address & 0xff;

      cs_low();
      if(spi->ops->write(spi, buf, 4) == 4) {
        if(spi->ops->write(spi, pData, size) == size) {
          ret = true;
        }
      }
      cs_high();
      if(ret) {
        ret = wait_till_ready();
      }
    }
  }

  if(!ret) {
    DBG_E("W25Q: Page program failed @0x%08x, len = %d, oset = %d\r\n", address, size, offset);
  }
  return ret;
}

/* Global functions ----------------------------------------------------------*/
uint32_t W25Q_Size(void) {
  return W25Q_CHIP_SIZE;
}

uint32_t W25Q_Sectors(void) {
  return W25Q_CHIP_SIZE / W25Q_SECTOR_SIZE;
}

bool W25Q_Erase(uint32_t address, uint32_t size) {
  if(address + size > W25Q_CHIP_SIZE) {
    return false;
  }

  if((size & W25Q_SECTOR_MASK) != 0) {
    return false;
  }

  uint32_t erase_size = W25Q_BLOCK64_SIZE;
  while(size >= erase_size) {
    if(!W25Q_BlockErase(address, erase_size)) {
      return false;
    }
    address += erase_size;
    size -= erase_size;
  }

  erase_size = W25Q_BLOCK32_SIZE;
  while(size >= erase_size) {
    if(!W25Q_BlockErase(address, erase_size)) {
      return false;
    }
    address += erase_size;
    size -= erase_size;
  }

  erase_size = W25Q_SECTOR_SIZE;
  while(size >= erase_size) {
    if(!W25Q_BlockErase(address, erase_size)) {
      return false;
    }
    address += erase_size;
    size -= erase_size;
  }

#ifdef CHECK_ERASE
  {
    uint32_t dat;
    W25Q_Read(address, sizeof(dat), (uint8_t*)&dat);
    if(0xffffffff != dat)
    {
      return false;
    }
  }
#endif

  return true;
 }

/**
 * @brief Chip erase
 *
 * @return true if success
 */
bool W25Q_ChipErase(void) {
  bool ret = false;
  uint8_t cmd = CHIP_ERASE;

  if(write_enable()) {
    cs_low();
    spi->ops->write(spi, &cmd, 1);
    cs_high();
    ret = wait_till_ready();
  }

  if(!ret) {
    DBG_E("W25Q: Chip erase failed\r\n");
  }
  return ret;
}

/**
 * @brief Block erase
 *
 * @param block_size
 *
 * @return true if success
 */
bool W25Q_BlockErase(uint32_t address, uint32_t size) {
  bool ret = false;
  uint8_t cmd = 0;

  switch(size) {
    case W25Q_SECTOR_SIZE:
      cmd = SECTOR_ERASE;
      break;
    case W25Q_BLOCK32_SIZE:
      cmd = BLOCK32_ERASE;
      break;
   case W25Q_BLOCK64_SIZE:
      cmd = BLOCK64_ERASE;
      break;
   default:
      break;
  }

  if(cmd != 0 && write_enable()) {
    uint8_t buf[4];

    buf[0] = cmd;
    buf[1] = (address >> 16) & 0xff;
    buf[2] = (address >> 8) & 0xff;
    buf[3] = (address) & 0xff;

    cs_low();
    spi->ops->write(spi, buf, 4);
    cs_high();
    ret = wait_till_ready();
  }

  if(!ret) {
    DBG_E("W25Q: Block erase failed @0x%08x\r\n", address);
  }
  return ret;
}

/**
 * @brief Read data from flash across page boundaries and at any addresses
 *
 * @param address
 * @param size
 * @param pBuf
 *
 * @return Number of bytes acutally read
 */
uint32_t W25Q_Read(uint32_t address, uint32_t size, uint8_t* pBuf) {
  uint32_t bytes_read = 0;

  // check that all bytes to be retrieved are located in valid flash memory address space
  if(size + address > W25Q_CHIP_SIZE)
  {
    size = W25Q_CHIP_SIZE - address;
  }

  if(wait_till_ready()) {
    uint8_t buf[4];

      buf[0] = READ_DATA;                             // Command for sequencial reading from memory
      buf[1] =(address >> 16) & 0xff;
      buf[2] =(address >> 8) & 0xff;
      buf[3] = address & 0xff;

      cs_low();
      if(spi->ops->write(spi, buf, 4) == 4) {
        if(spi->ops->read(spi, pBuf, size) == size) {
          bytes_read = size;
        }
      }
      cs_high();
  }

  if(bytes_read == 0) {
    DBG_E("W25Q: Read failed @0x%08x, len=%04x\r\n", address, size);
  }
  return bytes_read;
}

 /**
 * @brief Write data to falsh across page boundaries and at any addresses
 *
 * @param address
 * @param size
 * @param pData
 *
 * @return Number of bytes actually written
 */
uint32_t W25Q_Write(uint32_t address, uint32_t size, uint8_t * pData) {
  uint32_t currentAddress = address;
  uint32_t currentEndOfPage = (currentAddress | W25Q_PAGE_MASK);
  uint32_t bytes_left_to_send;
  uint32_t bytes_written;

  // limit to the maximum count of bytes that can be written
  if (size > W25Q_CHIP_SIZE - address)
  {
    size = W25Q_CHIP_SIZE - address;
  }

  bytes_left_to_send = size;
  bytes_written = 0;

  while (bytes_written < size)
  {
    // limit the transaction to the upper limit of the current page
    if (currentAddress + bytes_left_to_send > currentEndOfPage)
    {
      bytes_left_to_send = currentEndOfPage - currentAddress + 1;
    }
    // CAUSION: This is a SILLY PATCH, erase a sector when we write at the beginning
    if((currentAddress & W25Q_SECTOR_MASK) == 0) {
      W25Q_Erase(currentAddress, W25Q_SECTOR_SIZE);
    }
    //write the current page data
    if(!page_program(currentAddress, bytes_left_to_send, pData + bytes_written))
    {
      return bytes_written;
    }
    //address points to the first memory position of the next page
    bytes_written += bytes_left_to_send;
    currentAddress = currentEndOfPage + 1;
    currentEndOfPage += W25Q_PAGE_SIZE;
    bytes_left_to_send = size - bytes_written;
  }
  return bytes_written;
}

/**
 * @brief Release from power down
 *
 * @return true if success
 */
bool W25Q_ReleaseFromPowerDown(void) {
  uint8_t buf[4];

  buf[0] = REL_POWER_DOWN;          // Command to realease from power down & read Device ID
  buf[1] = 0;
  buf[2] = 0;
  buf[3] = 0;                       // dummy transactions

  cs_low();
  spi->ops->write(spi, buf, 4);
  spi->ops->read(spi, buf, 1);
  cs_high();

  if(buf[0] == W25Q16_DEV_ID && wait_till_ready()) {
    W25Q_CHIP_SIZE = W25Q16_CHIP_SIZE;
    DBG_D("W25Q: Release from power down succeed\r\n");
    return true;
  }

  if(buf[0] == GD25Q127_DEV_ID && wait_till_ready()) {
    W25Q_CHIP_SIZE = GD25Q127_CHIP_SIZE;
    DBG_D("W25Q: Release from power down succeed\r\n");
    return true;
  }

  DBG_E("W25Q: Release from power down failed\r\n");
  return false;
}

/**
 * @brief Power down
 *
 * @return true if success
 */
bool W25Q_PowerDown(void) {
  if(!wait_till_ready()) {
    DBG_E("W25Q: Power down failed\r\n");
    return false;
  }

  uint8_t cmd = POWER_DOWN;
  cs_low();
  spi->ops->write(spi, &cmd, 1);
  cs_high();

  DBG_D("W25Q: Power down succeed\r\n");
  return true;
}

/**
 * @brief Check the hardware connection
 *
 * @return true if success
 */
bool W25Q_HardwareTest(void) {
  if(W25Q_Lock() && W25Q_Unlock()) {
    return true;
  }

  DBG_E("W25Q: hardware test failed\r\n");
  return false;
}

/**
 * @brief Lock W25Q for an operation
 *
 * @return true if success
 */
bool W25Q_Lock(void) {
  if(!spi) {
    spi = hal_spi_get_instance(0);
    if(spi) {
      hal_spi_cfg_t cfg = {
        .mode = HAL_SPI_MODE_0,
        .freq = HAL_SPI_FREQ_4M,
        .ss_pin = HAL_PIN_NOT_USED,
        .sck_pin = HAL_CFG_FLS_SCK,
        .miso_pin = HAL_CFG_FLS_MISO,
        .mosi_pin = HAL_CFG_FLS_MOSI,
      };
      if(spi->ops->init(spi, &cfg) == HAL_ERR_OK) {
        spi->ops->unlock(spi);
      } else {
        DBG_E("W25Q: init failed\r\n");
        return false;
      }
    }
  }

  if(spi && spi->ops->lock(spi) == HAL_ERR_OK) {
    if(W25Q_ReleaseFromPowerDown()) {
      return true;
    }
  }

  spi->ops->unlock(spi);
  DBG_E("W25Q: Lock failed\r\n");

  return false;
}

/**
 * @brief Unlock W25Q
 *
 * @return true if success
 */
bool W25Q_Unlock(void) {
  if(spi) {
    W25Q_PowerDown();
    if(spi->ops->unlock(spi) == HAL_ERR_OK) {
      return true;
    }
  }

  DBG_E("W25Q: Unlock failed\r\n");
  return false;
}

#if 0
/**
 ****************************************************************************************
 * @brief Read data from a given starting address and calculate the check sum
 *
 * @param[in] *rd_data_ptr:  Points to the position the read data will be stored
 * @param[in] address:       Starting address of data to be read
 * @param[in] size:          Size of the data to be read
 *
 * @return  The check sum
 ****************************************************************************************
 */
int32_t w25q_check_data (uint32_t address, uint32_t size) {
  uint8_t buf[4];
  uint32_t i, sum = 0;

  // check that all bytes to be retrieved are located in valid flash memory address space
  if (size + address > W25Q_CHIP_SIZE)
  {
    return ERR_INVAL;
  }

  int8_t status = wait_till_ready();
  if (status != ERR_OK)
  {
    return status;
  }

  buf[0] = READ_DATA;                             // Command for sequencial reading from memory
  buf[1] = (address >> 16) & 0xff;
  buf[2] = (address >> 8) & 0xff;
  buf[3] = address & 0xff;

  cs_low();
  spi->ops->write(spi, buf, 4);
  for(i = 0; i < size; i++)
  {
    spi->ops->read(spi, buf, 1);
    sum += buf[0];
  }
  cs_high();

  return sum;
}
#endif

