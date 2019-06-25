/*!
 *    @file  ACC.h
 *   @brief  Accelerometer
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

#ifndef __ACC_H__
#define __ACC_H__

/* Includes ------------------------------------------------------------------*/
#include "hal.h"

/* Defines -------------------------------------------------------------------*/

/* Typedefs ------------------------------------------------------------------*/

typedef void (*dev_acc_evt_cb_t) (dev_acc_event_t evt, void* p_context);

typedef enum dev_acc_odr {
    DEV_ACC_ODR_12HZ = 0,
    DEV_ACC_ODR_25HZ,
    DEV_ACC_ODR_50HZ,
    DEV_ACC_ODR_100HZ,
    DEV_ACC_ODR_MAX,
} dev_acc_odr_t;

struct dev_acc_ops;
typedef struct dev_acc {
    int32_t instance;
    dev_acc_odr_t odr;
    const struct dev_acc_ops* ops;
} dev_acc_t;

typedef struct dev_acc_ops {
    hal_err_t (*init) (dev_acc_t *acc, dev_acc_evt_cb_t evt_cb);
    hal_err_t (*deinit) (dev_acc_t *acc);

    hal_err_t (*lock) (dev_acc_t *acc);
    hal_err_t (*unlock) (dev_acc_t *acc);

    hal_err_t (*sample_enable) (dev_acc_t* acc, bool en);
    hal_err_t (*wake_enable) (dev_acc_t* acc, bool en);

    hal_err_t (*read_fifo) (dev_acc_t* acc, void* buf, int len);
} dev_acc_ops_t;
typedef void (*ACC_Callback_t) (void);

/* Function prototypes -------------------------------------------------------*/
bool ACC_HardwareTest(void);
bool ACC_EnableSample(ACC_Callback_t cb);
bool ACC_EnableShake(ACC_Callback_t cb);
bool ACC_ReadStatus(uint8_t *pStatus);
bool ACC_ReadFIFO(int16_t* buf, int32_t* cnt);
bool ACC_ReadData(int16_t* buf);
bool ACC_ClearFIFO(void);
bool ACC_Init(void);
bool ACC_Deinit(void);
bool ACC_Lock(void);
bool ACC_Unlock(void);
bool ACC_HardwareReset(void);
bool ACC_SoftwareReset(void);
void ACC_InterruptHandler(void);

#endif // #ifndef __ACC_H__
