/*!
 *    @file  w25q.h
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

#ifndef __W25Q_H__
#define __W25Q_H__


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Defines -------------------------------------------------------------------*/
// Flash ID
#define W25Q40_MAN_ID           0xEF
#define W25Q40_DEV_ID           0x12
#define W25Q16_MAN_ID           0xEF
#define W25Q16_DEV_ID           0x14
#define W25Q16_MAN_DEV_ID       (W25Q16_MAN_ID | (W25Q16_DEV_ID << 8))
#define GD25Q127_MAN_ID         0xC8
#define GD25Q127_DEV_ID         0x17
#define GD25Q127_MAN_DEV_ID     (GD25Q127_MAN_ID | (GD25Q127_DEV_ID << 8))
// Flash Size
#define W25Q40_CHIP_SIZE        (512 * 1024)
#define W25Q16_CHIP_SIZE        (2 * 1024 * 1024)
#define GD25Q127_CHIP_SIZE      (16 * 1024 * 1024)
// Sector Size
#define W25Q_PAGE_SIZE          256
#define W25Q_PAGE_MASK          (W25Q_PAGE_SIZE - 1)
#define W25Q_SECTOR_SIZE        4096
#define W25Q_SECTOR_MASK        (W25Q_SECTOR_SIZE - 1)
#define W25Q_BLOCK32_SIZE       32768
#define W25Q_BLOCK32_MASK       (W25Q_BLOCK32_SIZE - 1)
#define W25Q_BLOCK64_SIZE       65536
#define W25Q_BLOCK64_MASK       (W25Q_BLOCK_SIZE - 1)

// Status Register
#define STATUS_BUSY             0x01
#define STATUS_WEL              0x02
#define STATUS_BP0              0x04
#define STATUS_BP1              0x08
#define STATUS_BP2              0x10
#define STATUS_TB               0x20
#define STATUS_SRP              0x80
// Commands
#define WRITE_ENABLE            0x06
#define WRITE_ENABLE_VOL        0x50
#define WRITE_DISABLE           0x04
#define READ_STATUS_REG         0x05
#define WRITE_STATUS_REG        0x01
#define PAGE_PROGRAM            0x02
#define QUAD_PAGE_PROGRAM       0x32
#define CHIP_ERASE              0xC7
#define SECTOR_ERASE            0x20
#define BLOCK32_ERASE           0x52
#define BLOCK64_ERASE           0xD8
//                              ^^^// or 0x60
#define ERASE_SUSPEND           0x75
#define ERASE_RESUME            0x7a
#define POWER_DOWN              0xb9
#define HIGH_PERF_MODE          0xa3
#define MODE_BIT_RESET          0xff
#define REL_POWER_DOWN          0xab
#define MAN_DEV_ID              0x90
#define READ_UNIQUE_ID          0x4b
#define JEDEC_ID                0x9f
#define READ_DATA               0x03
#define FAST_READ               0x0b

/* Function prototypes -------------------------------------------------------*/
bool W25Q_HardwareTest(void);
bool W25Q_ChipErase(void);
bool W25Q_BlockErase(uint32_t address, uint32_t size);
bool W25Q_Erase(uint32_t address, uint32_t size);
uint32_t W25Q_Read(uint32_t address, uint32_t size, uint8_t* pBuf);
uint32_t W25Q_Write(uint32_t address, uint32_t size, uint8_t* pData);
bool W25Q_Lock(void);
bool W25Q_Unlock(void);
uint32_t W25Q_Size(void);
uint32_t W25Q_Secotrs(void);
#endif // #ifndef __W25Q_H__
