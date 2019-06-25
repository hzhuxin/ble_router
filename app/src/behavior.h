/*!
 *    @file  behavior.h
 *   @brief  The behavior module
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

#ifndef __BEHAVIOR_H__
#define __BEHAVIOR_H__

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/* Typedefs ------------------------------------------------------------------*/
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} DataPoint_t;

typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} DataPoint32_t;

typedef struct Behavior_Record {
    uint32_t time;
#if DEV_TYPE == 101 && HW_VERSION == 2
    DataPoint_t odba;
    DataPoint_t meandl;
    int16_t dummy[12];
#else
    uint32_t odba;
#endif
} Behavior_Record_t;

/* Global variables ----------------------------------------------------------*/


/* Defines -------------------------------------------------------------------*/

/**
 * @brief Set the behavior calculator peroid
 *
 * @param peroid  record peroid
 *
 * @return 0 if success
 */
int Behavior_SetPeroid(int peroid);

/**
 * @brief Init the behavior calculator
 *
 * @param w_size  window size
 * @param peroid  record peroid
 *
 * @return 0 if success
 */
int Behavior_Init(int w_size, int peroid);

/**
 * @brief Update the behavior calculator
 *
 * @param acc  pointer to the accelerometer
 * @param len  number of data items
 * @param time timestamp
 *
 * @return behavior record if needs to save
 */
Behavior_Record_t* Behavior_Update(DataPoint_t *pa, int len, uint32_t time);

#endif // #ifndef __BEHAVIOR_H__

