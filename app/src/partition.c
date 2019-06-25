/*!
 *    @file  Partition.c
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


/* Includes ------------------------------------------------------------------*/
#include "partition.h"
#include "string.h"
#include "debug.h"
#include "w25q.h"

/* Defines -------------------------------------------------------------------*/
DBG_SET_LEVEL(DBG_LEVEL_W);

/* Private functions ---------------------------------------------------------*/
// Read an index
static bool readIndex(Part_Info_t *part, uint32_t nb) {
  if(nb < W25Q_SECTOR_SIZE / sizeof(Part_Index_t)) {
    Part_Index_t* index = part->index;
    int oset = nb * sizeof(Part_Index_t);
    if(W25Q_Read(part->base + oset, sizeof(Part_Index_t), (uint8_t*)index) == sizeof(Part_Index_t)) {
      return true;
    }
  }

  DBG_E("PART: Read index #%d failed\r\n", nb);
  return false;
}
// Check if an index is valid
static bool isIndexValid(Part_Info_t *part, uint32_t nb) {
  if(nb < W25Q_SECTOR_SIZE / sizeof(Part_Index_t)) {
    Part_Index_t* index = part->index;
    if(index->sequence == nb && index->write < part->capacity && index->read < part->capacity) {
      return (index->count == (index->write + part->capacity - index->read) % part->capacity);
    }
  }

  return false;
}
// Check if an index is blank
static bool isIndexBlank(Part_Info_t *part) {
  uint8_t* s = (uint8_t*)part->index;

  for(int i = 0; i < sizeof(Part_Index_t); i++) {
    if(s[i] != 0xff) {
      return false;
    }
  }

  return true;
}
// Write index item of a part
static bool writeIndex(Part_Info_t *part) {
  Part_Index_t* index = part->index;
  if(++index->sequence >= W25Q_SECTOR_SIZE / sizeof(Part_Index_t)) {
    index->sequence = 0;
  }
  int oset = index->sequence * sizeof(Part_Index_t);
  if(W25Q_Write(part->base + oset, sizeof(Part_Index_t), (uint8_t*)index) == sizeof(Part_Index_t)) {
    return true;
  }

  DBG_E("PART: Write index failed\r\n");
  return false;
}
// Read index item for a part
static bool searchIndex(Part_Info_t *part) {
  DBG_D("PART: Searching index for @%08x...\r\n", part);
  Part_Index_t* index = part->index;
  // The first item is not valid, just init it to the default value
  DBG_V("PART: check the 1st index\r\n");
  if(readIndex(part, 0) && !isIndexValid(part, 0)) {
    memset(index, 0, sizeof(Part_Index_t));
    W25Q_Write(part->base, sizeof(Part_Index_t), (uint8_t*)index);
    return true;
  }
  DBG_V("PART: check the last index\r\n");
  uint32_t last = W25Q_SECTOR_SIZE / sizeof(Part_Index_t) - 1;
  // The last item is valid, then we are done here
  if(readIndex(part, last) && isIndexValid(part, last)) {
    return true;
  }
  int low = 0;
  int high = last;
  // The last item is not blank
  if(!isIndexBlank(part)) {
    low = last;
  }
  // Find the last item that is not blank
  while(low + 1 < high) {
    int mid = (low + high) >> 1;
    DBG_V("PART: check index #%d\r\n", mid);
    if(readIndex(part, mid) && isIndexBlank(part)) {
      high = mid;
    } else {
      low = mid;
    }
  }
  // Find the item that is valid
  if(readIndex(part, low) && isIndexValid(part, low)) {
    return true;
  }
  DBG_W("PART: found invalid index #%d\r\n", low);
  if(low-- > 0 && readIndex(part, low) && isIndexValid(part, low)) {
    index->sequence = 0;
    W25Q_Write(part->base, sizeof(Part_Index_t), (uint8_t*)index);
    return true;
  }
  // Something went wrong, init the first item
  DBG_W("PART: no valid index found\r\n");
  memset(index, 0, sizeof(Part_Index_t));
  W25Q_Write(part->base, sizeof(Part_Index_t), (uint8_t*)index);
  return true;
}

/* Global functions ----------------------------------------------------------*/

/**
 * @brief Write an item to a partition
 *
 * @param part
 * @param pItem
 *
 * @return true if success
 */
bool Part_Write(Part_Info_t *part, uint8_t* pItem) {
  bool ret = false;

  if(Part_Lock(part)) {
    if(W25Q_Lock()) {
      Part_Index_t* index = part->index;
      if(index->count < part->capacity) {
        int oset = W25Q_SECTOR_SIZE + index->write * part->itemSize;
        W25Q_Write(part->base + oset, part->itemSize, pItem);
        // Update index
        index->count++;
        index->write++;
        if(index->write >= part->capacity) {
          index->write = 0;
        }
        // Write index
        writeIndex(part);
        W25Q_Unlock();
        ret = true;
      }
    }
    Part_Unlock(part);
  }

  if(!ret) {
    DBG_W("PART: Write failed @%08x\r\n", part);
  }
  return ret;
}
/**
 * @brief Write items to a partition
 *
 * @param part
 * @param pItem
 * @param nbItems
 *
 * @return true if success
 */
uint32_t Part_Writes(Part_Info_t *part, uint8_t* pItem, uint32_t nbItems) {
  uint32_t ret = 0;

  if(Part_Lock(part)) {
    Part_Index_t* index = part->index;
    // Items avaliable to write
    nbItems = (index->count + nbItems <= part->capacity) ? nbItems : part->capacity - index->count;
    if(nbItems > 0) {
      if(W25Q_Lock()) {
        // Write twice, take one
        int n = (index->write + nbItems <= part->capacity) ? nbItems : part->capacity - index->write;
        int oset = W25Q_SECTOR_SIZE + index->write * part->itemSize;
        int len = part->itemSize * n;
        W25Q_Write(part->base + oset, len, pItem);
        pItem += len;
        // Write twice, take two
        if(n < nbItems) {
          oset = W25Q_SECTOR_SIZE;
          len = part->itemSize * (nbItems - n);
          W25Q_Write(part->base + oset, len, pItem);
        }
        // Update index
        index->count += nbItems;
        index->write += nbItems;
        if(index->write >= part->capacity) {
          index->write %= part->capacity;
        }
        // Write index
        writeIndex(part);
        W25Q_Unlock();
        ret = nbItems;
      }
    }
    Part_Unlock(part);
  }

  if(!ret) {
    DBG_W("PART: Writes failed @%08x\r\n", part);
  }
  return ret;
}

/**
 * @brief Count number of items in a partition
 *
 * @return number of items
 */
uint32_t Part_Count(Part_Info_t *part) {
  uint32_t count = 0;
  bool ret = false;

  if(Part_Lock(part)) {
    count = part->index->count;
    ret = true;
    Part_Unlock(part);
  }

  if(!ret) {
    DBG_E("PART: Count failed @%08x\r\n", part);
  }
  return count;
}

/**
 * @brief Get an item from a partition
 *
 * @param part
 * @param offset
 * @param pOut
 *
 * @return true if success
 */
bool Part_Get(Part_Info_t *part, uint32_t offset, uint8_t* pOut) {
  bool ret = false;

  if(Part_Lock(part)) {
    if(W25Q_Lock()) {
      Part_Index_t* index = part->index;
      if(offset < index->count) {
        int n = (index->read + offset) % part->capacity;
        int oset = W25Q_SECTOR_SIZE + n * part->itemSize;
        int len = part->itemSize;
        W25Q_Read(part->base + oset, len, pOut);
        ret = true;
      }

      W25Q_Unlock();
    }
    Part_Unlock(part);
  }

  if(!ret) {
    DBG_I("PART: Get failed @%08x\r\n", part);
  }
  return ret;
}

/**
 * @brief Read items from a partition
 *
 * @param part
 * @param pOut
 * @param nbItems
 *
 * @return number of item read
 */
uint32_t Part_Read(Part_Info_t *part, uint8_t* pOut, uint32_t outBufSize) {
  uint16_t nbItems = 0;

  if(Part_Lock(part)) {
    if(W25Q_Lock()) {
      Part_Index_t* index = part->index;
      // Items avaliable to read
      nbItems = outBufSize / part->itemSize;
      nbItems = (nbItems <= index->count) ? nbItems : index->count;
      if(nbItems > 0) {
        // Read twice, take one
        int n = (index->read + nbItems <= part->capacity) ? nbItems : part->capacity - index->read;
        int oset = W25Q_SECTOR_SIZE + index->read * part->itemSize;
        int len = part->itemSize * n;
        W25Q_Read(part->base + oset, len, pOut);
        pOut += len;
        // Read twice, take two
        if(n < nbItems) {
          oset = W25Q_SECTOR_SIZE;
          len = part->itemSize * (nbItems - n);
          W25Q_Read(part->base + oset, len, pOut);
        }
      }
      W25Q_Unlock();
    }
    Part_Unlock(part);
  }

  if(nbItems == 0) {
    DBG_I("PART: Read failed @%08x\r\n", part);
  }
  return nbItems;
}

/**
 * @brief Peek items from a partition
 *
 * @param part
 * @param pOut
 * @param nbItems
 *
 * @return number of items
 */
uint32_t Part_Peek(Part_Info_t *part, uint8_t* pOut, uint32_t outBufSize) {
  uint16_t nbItems = 0;

  if(Part_Lock(part)) {
    if(W25Q_Lock()) {
      Part_Index_t* index = part->index;
      // Items avaliable to read
      nbItems = outBufSize / part->itemSize;
      nbItems = (nbItems <= index->count) ? nbItems : index->count;
      if(nbItems > 0) {
        // Item offset for read
        uint16_t osetRead = (index->read + (index->count - nbItems)) % part->capacity;
        // Read twice, take one
        int n = (osetRead + nbItems <= part->capacity) ? nbItems : part->capacity - osetRead;
        int oset = W25Q_SECTOR_SIZE + osetRead * part->itemSize;
        int len = part->itemSize * n;
        W25Q_Read(part->base + oset, len, pOut);
        pOut += len;
        // Read twice, take two
        if(n < nbItems) {
          oset = W25Q_SECTOR_SIZE;
          len = part->itemSize * (nbItems - n);
          W25Q_Read(part->base + oset, len, pOut);
        }
      }
      W25Q_Unlock();
    }
    Part_Unlock(part);
  }

  if(nbItems == 0) {
    DBG_I("PART: Peek failed @%08x\r\n", part);
  }
  return nbItems;
}

/**
 * @brief Delete items from a partition
 *
 * @param part
 * @param nbItems
 *
 * @return true if success
 */
bool Part_Delete(Part_Info_t *part, uint32_t nbItems) {
  bool ret = false;

  if(Part_Lock(part)) {
    if(W25Q_Lock()) {
      Part_Index_t* index = part->index;
      // Items avaliable to delete
      nbItems = (nbItems <= index->count) ? nbItems : index->count;
      if(nbItems > 0) {
        // Update index
        index->count -= nbItems;
        index->read = (index->read + nbItems) % part->capacity;
        // Write index
        writeIndex(part);
      }
      W25Q_Unlock();
      ret = true;
    }
    Part_Unlock(part);
  }

  if(!ret) {
    DBG_E("PART: Delete failed @%08x\r\n", part);
  }
  return ret;
}

/**
 * @brief Erase a partition
 *
 * @param part
 *
 * @return true if success
 */
bool Part_Erase(Part_Info_t *part) {
  bool ret = false;

  if(Part_Lock(part)) {
    if(W25Q_Lock()) {
      // Clear partition index
      part->index->read = 0;
      part->index->write = 0;
      part->index->count = 0;
      part->index->sequence = 0;
      // Write partition index
      W25Q_Write(part->base, sizeof(Part_Index_t), (uint8_t*)part->index);
      W25Q_Unlock();
      ret = true;
    }
    Part_Unlock(part);
  }

  if(!ret) {
    DBG_E("PART: Erase failed @%08x\r\n", part);
  }
  return ret;
}

/**
 * @brief Lock a partition
 *
 * @param part
 *
 * @return true if success
 */
bool Part_Lock(Part_Info_t *part) {
  if(part != NULL && part->lock == NULL) {
    if(W25Q_Lock()) {
      // Find the latest index
      bool ret = searchIndex(part);
      W25Q_Unlock();

      // Create lock object
      if(ret) {
#if configSUPPORT_STATIC_ALLOCATION
        part->lock = xSemaphoreCreateMutexStatic(&part->lock_memory);
#else
        part->lock = xSemaphoreCreateMutex();
#endif
        xSemaphoreGive(part->lock);
      }
    }
  }

  if(part != NULL && part->index != NULL && part->lock != NULL) {
    if(xSemaphoreTake(part->lock, pdMS_TO_TICKS(1000))) {
      return true;
    }
  }

  DBG_E("PART: Lock failed @%08x\r\n", part);
  return false;
}

/**
 * @brief Unlock a partition
 *
 * @param part
 *
 * @return true if success
 */
bool Part_Unlock(Part_Info_t *part) {
  if(part != NULL && part->index != NULL && part->lock != NULL) {
    if(xSemaphoreGive(part->lock)) {
      return true;
    }
  }

  DBG_E("PART: Unlock failed @%08x\r\n", part);
  return false;
}
