/**
 * @brief 
 * 
 * @file ble_druid_svc.h
 * @author Chenfei
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.
 * All Rights Reserved.
 * @date 2018-07-10
 */
#ifndef BLE_DRUID_SVC_H
#define BLE_DRUID_SVC_H

#include <stdint.h>
#include <stdbool.h>
#include "ble_gap.h"
#include "hal.h"

#define MIN_CONNECTION_INTERVAL         (uint16_t) MSEC_TO_UNITS(15, UNIT_1_25_MS) /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL         (uint16_t) MSEC_TO_UNITS(500, UNIT_1_25_MS)  /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY                   0                                           /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT             (uint16_t) MSEC_TO_UNITS(4000, UNIT_10_MS)  /**< Determines supervision time-out in units of 10 milliseconds. */

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define SCAN_TIME_DEFAULT               5000        //default scanning timeout is 5S
#define BLE_TX_COMPLETE_WAIT            4000        //MS
#define BLE_RX_BUF_SIZE                 1024
#define BLE_TX_BUF_SIZE                 1024

void ble_init(bool *erase_bonds);
uint32_t ble_act_slave_send(void * const p_data, uint16_t length);
uint8_t get_ble_s_max_data_len(void);
bool get_s_connect_status(void);
bool scan_start(void);
uint8_t get_device_mac_addr(uint8_t * const p_mac, uint8_t const size);
void multi_qwr_conn_handle_assign(uint16_t conn_handle);
uint16_t get_conn_gatt_mtu(uint16_t conn_handle);


#endif //BLE_DRUID_SVC_H
