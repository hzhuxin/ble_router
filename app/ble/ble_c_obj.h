#ifndef BLE_C_APP_H
#define BLE_C_APP_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "app_freertos.h"
#include "ble_gap.h"
#include "ble.h"
#include "hal.h"


#define SCAN_TIME_DEFAULT   5000        //default scanning timeout is 5S

typedef enum
{
    INVALID_MODE = 0,
    NEED_NAME_AND_ADDR, //attention name and addr, when search slave device
    NEED_ONLY_NAME,         //only attention device name
    NEED_ONLY_ADDR,          //only attention device MAC addr
    NEED_NONE,              //don't attention neither name nor addr
}search_mode;
/*
// #define BLE_C_QUEUE_ARRY_DEF(_name, _cnt)   \
     static QueueHandle_t    _name[_cnt];    \
     memset(_name,0,sizeof(_name));
*/
#define BLE_GAP_SCAN_PARAMS_DEFAULT             \
{                                               \
    .extended      = 0,                         \
    .active        = 1,                         \
    .interval      = SCAN_INTERVAL,             \
    .window        = SCAN_WINDOW,               \
    .timeout       = SCAN_DURATION,             \
    .scan_phys     = BLE_GAP_PHY_1MBPS,         \
    .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,\
}

#define SCAN_TARGET_MSG_DEFAULT                 \
{                                               \
    .name       = NULL,                               \
    .addr       = NULL,                               \
    .scan_params= NULL,                               \
    .mode       = NEED_NONE,                          \
    .handler    = NULL,                            \
}
typedef struct target_s target_t;


typedef void (*scan_handler_t)(target_t const * p_content);
typedef void (*ble_c_rx_handler_t)(ble_data_t * p_ble_data);
typedef void (*ble_c_connect_handler_t)(void *arg);
typedef void (*ble_c_disconect_handler_t)(void *arg);

struct target_s
{
    char            * name;
    ble_gap_addr_t  * peer_addr;
    ble_data_t      * data;
};

typedef struct config_c
{
    ble_c_rx_handler_t          rx_handler;
    ble_c_connect_handler_t     conn_handler;
    ble_c_disconect_handler_t   disc_handler;
    uint32_t                    tx_timeout;
}ble_c_cfg_t;

typedef struct
{
    char                  * name;
    uint8_t               * addr;
    ble_gap_scan_params_t * scan_params;
    search_mode             mode;
    scan_handler_t          handler;
}scan_target_t;

typedef struct
{
    uint8_t                 *addr;
    uint16_t                *conn_handle;
    ble_gap_scan_params_t * scan_params;
    ble_gap_conn_params_t * conn_params;
    //conn_handler_t          handler;

}connect_target_t;

typedef struct ble_c 
{
  void* lock;
  void* priv;
  const struct ble_c_ops* ops;
} ble_c_t;

void centr_init(void);
//void ble_c_on_ble_evt(ble_evt_t const*p_ble_evt);
void on_ble_central_evt(ble_evt_t const * p_ble_evt);
void db_discovery_init(void);
void set_ble_c_mtu_len(uint8_t mtu_len);

typedef struct ble_c_ops
{
    //hal_err_t (*init)(ble_c_t *obj);
    hal_err_t (*lock) (ble_c_t* obj);
    hal_err_t (*unlock) (ble_c_t* obj);
    hal_err_t (*init)(ble_c_t *obj,ble_c_cfg_t * config);
    hal_err_t (*deinit)(ble_c_t *obj);
    hal_err_t (*scan)(ble_c_t *obj, scan_target_t const *p_target, int32_t timeout);
    hal_err_t (*scan_stop)(ble_c_t *obj);
    hal_err_t (*connect)(ble_c_t *obj, connect_target_t const *p_target, int32_t timeout);
    hal_err_t (*disconnect)(ble_c_t *obj, uint16_t *conn_handle);
    hal_err_t (*transmit)(ble_c_t *obj, uint16_t conn_handle,uint8_t *buf,uint16_t length);
    hal_err_t (*receive)(ble_c_t *obj,uint16_t conn_handle,uint8_t *buf,uint16_t size,uint32_t timeout);
    hal_err_t (*get_mtu)(ble_c_t *obj, uint16_t conn_handle);
    //hal_err_t (*status)(ble_c_t *obj);
}ble_c_ops_t;


ble_c_t* ble_c_get_instance(int id) ;
#endif //BLE_C_APP_H

