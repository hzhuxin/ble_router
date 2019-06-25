/*!
 *    @file  Behavior.c
 *   @brief  The behaivor module
 *
 *  @author  Dale.J (dj), Dale.J@zoho.com
 *
 *  @internal
 *       Created:  01/29/2018
 *      Revision:  none
 *  Organization:  Druid Tech
 *     Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */


/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include "buffer.h"
#include "behavior.h"
#include "debug.h"

/* Defines -------------------------------------------------------------------*/
DBG_SET_LEVEL(DBG_LEVEL_I);

#define _MIN_RECORD_COUNT   100
#define _MIN_PEROID         30
#define _MAX_PEROID         600
#define _MIN_WINDOW_SIZE    5
#define _MAX_WINDOW_SIZE    51

/* Private variables ---------------------------------------------------------*/
static int record_peroid = 0;
static Behavior_Record_t record;
static DataPoint_t acc_buf[_MAX_WINDOW_SIZE];
static DataPoint_t sba_buf[_MAX_WINDOW_SIZE];
static buffer_t acc = BUFFER_CREATE_STATIC(sizeof(*acc_buf), sizeof(acc_buf) / sizeof(*acc_buf), acc_buf);
static buffer_t sba = BUFFER_CREATE_STATIC(sizeof(*sba_buf), sizeof(sba_buf) / sizeof(*sba_buf), sba_buf);

/* Private functions ---------------------------------------------------------*/
/**
 * @brief Update the filter
 *
 * @param pa  pointer to accelerometer data
 *
 * @return true if output is ready
 */
static bool UpdateFilter(DataPoint_t *pa) {
  // Check input
  if(!pa) {
    return false;
  }

  // Accumulator for sba calculation
  static DataPoint32_t sum = { 0, 0, 0 };

  // Accumulate the new acc
  sum.x += pa->x;
  sum.y += pa->y;
  sum.z += pa->z;

  // Minus the old acc
  if(acc.count >= acc.capacity) {
    DataPoint_t *p = acc.ops->get(&acc, 0);

    sum.x -= p->x;
    sum.y -= p->y;
    sum.z -= p->z;

    acc.ops->remove(&acc, 1);
  }

  // Save acc
  acc.ops->push(&acc, pa);

  // Calculate & save sba
  if(acc.count >= acc.capacity / 2) {
    DataPoint_t t;
    t.x = sum.x / acc.count;
    t.y = sum.y / acc.count;
    t.z = sum.z / acc.count;

    if(sba.count >= sba.capacity) {
      sba.ops->remove(&sba, 1);
    }
    sba.ops->push(&sba, &t);
  }

  // return
  if(sba.count >= sba.capacity) {
    return true;
  }

  return false;
}


/* Global functions ----------------------------------------------------------*/

/**
 * @brief Set the behavior calculator peroid
 *
 * @param peroid  record peroid
 *
 * @return 0 if success
 */
int Behavior_SetPeroid(int peroid) {
  if(record_peroid != peroid) {
    record_peroid = peroid;
  }

  return 0;
}

/**
 * @brief Init the behavior calculator
 *
 * @param w_size  window size
 * @param peroid  record peroid
 *
 * @return 0 if success
 */
int Behavior_Init(int w_size, int peroid) {
  // Check input
  if(w_size < _MIN_WINDOW_SIZE || w_size > _MAX_WINDOW_SIZE || !(w_size & 1)) {
    return -1;
  }
  if(peroid < _MIN_PEROID || peroid > _MAX_PEROID) {
    return -1;
  }

  // Update param
  acc.capacity = w_size;
  sba.capacity = w_size;
  record_peroid = peroid;

  // Clear buffer
  acc.ops->clear(&acc);
  sba.ops->clear(&sba);
  memset(acc_buf, 0, sizeof(acc_buf));
  memset(sba_buf, 0, sizeof(sba_buf));

  return 0;
}

/**
 * @brief Update the behavior calculator
 *
 * @param pa   pointer to the accelerometer
 * @param len  number of data items
 * @param time timestamp
 *
 * @return behavior record if needs to save
 */
Behavior_Record_t* Behavior_Update(DataPoint_t *pa, int len, uint32_t time) {
  // Check input
  if(!pa || len < 1) {
    return NULL;
  }

  // Local variable
  static uint32_t record_timestamp = 0;
  static uint32_t record_count = 0;

  static DataPoint32_t dba = { 0, 0, 0 };
#if DEV_TYPE == 101 && HW_VERSION == 2
  static DataPoint32_t mdl = { 0, 0, 0 };
  static DataPoint_t last = { 0, 0, 0 };
#endif

  DBG_V("\n------------------------------------\n")
  // Update
  for(int i = 0; i < len; i++) {
    if(UpdateFilter(pa++)) {
      DataPoint_t* pacc = acc.ops->get(&acc, 0);
      DataPoint_t* psba = sba.ops->get(&sba, 0);

      if(!record_count++) {
#if DEV_TYPE == 101 && HW_VERSION == 2
        mdl.x = abs(pacc->x - last.x);
        mdl.y = abs(pacc->y - last.y);
        mdl.z = abs(pacc->z - last.z);
#endif

        dba.x = abs(pacc->x - psba->x);
        dba.y = abs(pacc->y - psba->y);
        dba.z = abs(pacc->z - psba->z);

        record_timestamp = time;
      } else {
#if DEV_TYPE == 101 && HW_VERSION == 2
        mdl.x += abs(pacc->x - last.x);
        mdl.y += abs(pacc->y - last.y);
        mdl.z += abs(pacc->z - last.z);
#endif

        dba.x += abs(pacc->x - psba->x);
        dba.y += abs(pacc->y - psba->y);
        dba.z += abs(pacc->z - psba->z);
      }
			DBG_V("Behavior [%04d]  Raw(%d, %d, %d)", record_count, pacc->x, pacc->y, pacc->z);
			DBG_V("\tSBA(%d, %d, %d)", psba->x, psba->y, psba->z);
			DBG_V("\tDBA(%d, %d, %d)", dba.x, dba.y, dba.z);
#if DEV_TYPE == 101 && HW_VERSION == 2
			DBG_V("\tMNDL(%d, %d, %d)\n", mdl.x, mdl.y, mdl.z);
#endif
    }

#if DEV_TYPE == 101 && HW_VERSION == 2
    DataPoint_t* pacc = acc.ops->get(&acc, 0);
    last.x = pacc->x;
    last.y = pacc->y;
    last.z = pacc->z;
#endif
  }

  // output
  if(time - record_timestamp >= record_peroid) {
    if(record_count > _MIN_RECORD_COUNT) {
#if DEV_TYPE == 101 && HW_VERSION == 2
      record.meandl.x = 10 * mdl.x / record_count;
      record.meandl.y = 10 * mdl.y / record_count;
      record.meandl.z = 10 * mdl.z / record_count;

      record.odba.x = 10 * dba.x / record_count;
      record.odba.y = 10 * dba.y / record_count;
      record.odba.z = 10 * dba.z / record_count;

      record.time = time;
      record_count = 0;

			DBG_D("Behavior: odba(%d, %d, %d), meandl(%d, %d, %d)\n",
          record.odba.x, record.odba.y, record.odba.z,
			    record.meandl.x, record.meandl.y, record.meandl.z);
#else
      record.odba = (dba.x + dba.y + dba.z) * 10 / record_count;

      record.time = time;
      record_count = 0;

			DBG_D("Behavior: odba=%d\n", record.odba);
#endif

      return &record;
    } else {
      record_count = 0;
    }
  }

  return NULL;
}

