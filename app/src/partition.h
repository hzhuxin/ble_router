/*!
 *    @file  Partition.h
 *   @brief  The storage module based on the spi flash
 *
 *  @author  Dale.J (dj), Dale.J@zoho.com
 *
 *  @internal
 *       Created:  04/21/2016
 *      Revision:  none
 *  Organization:  Druid Tech
 *     Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */

#ifndef __PARTITION_H__
#define __PARTITION_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"

/* Typedefs ------------------------------------------------------------------*/
typedef struct {
  uint32_t write;
  uint32_t read;
  uint32_t count;
  uint32_t sequence;
} Part_Index_t;

typedef struct {
  uint32_t base;
  uint32_t capacity;
  uint32_t itemSize;
  Part_Index_t *index;
  SemaphoreHandle_t lock;
#if configSUPPORT_STATIC_ALLOCATION
  StaticSemaphore_t lock_memory;
#endif
} Part_Info_t;

/* Function prototypes -------------------------------------------------------*/
bool Part_Lock(Part_Info_t *part);
bool Part_Unlock(Part_Info_t *part);
bool Part_Erase(Part_Info_t *part);
bool Part_Write(Part_Info_t *part, uint8_t* pItem);
uint32_t Part_Writes(Part_Info_t *part, uint8_t* pItem, uint32_t nbItems);
uint32_t Part_Count(Part_Info_t *part);
uint32_t Part_Read(Part_Info_t *part, uint8_t* pOut, uint32_t outBufSize);
uint32_t Part_Peek(Part_Info_t *part, uint8_t* pOut, uint32_t outBufSize);
bool Part_Get(Part_Info_t *part, uint32_t offset, uint8_t* pOut);
bool Part_Delete(Part_Info_t *part, uint32_t nbItems);
#endif // #ifndef __PARTITION_H__

