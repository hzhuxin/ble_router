
#ifndef BLE_S_APP_H
#define BLE_S_APP_H

#include <stdint.h>
#include <stdbool.h>
#include "app_freertos.h"
#include "hal.h"
#include "ble_types.h"
#include "ble.h"

typedef void (*ble_s_rx_handler_t)(ble_data_t * p_ble_data);
typedef void (*ble_s_connect_handler_t)(void *arg);
typedef void (*ble_s_disconect_handler_t)(void *arg);
typedef void (*ble_s_notify_handler_t)(void *arg);
typedef void (*ble_s_unnotify_handler_t)(void *arg);

typedef struct config_s
{
    ble_s_rx_handler_t rx_handler;
    ble_s_connect_handler_t conn_handler;
    ble_s_disconect_handler_t disc_handler;
    ble_s_notify_handler_t notify_handler;
    ble_s_unnotify_handler_t unnotify_handler;
    uint32_t tx_timeout;
}ble_s_cfg_t;
typedef struct ble_s
{
    void* lock;
    void* priv;
    const struct ble_s_ops* ops;
} ble_s_t;

void services_init(void);
void set_ble_s_mtu_len(uint8_t mtu_len);
void on_ble_peripheral_evt(ble_evt_t const * p_ble_evt);

typedef struct ble_s_ops
{
    hal_err_t (*init)(ble_s_t *obj,ble_s_cfg_t * config);
    hal_err_t (*deinit)(ble_s_t *obj);
    hal_err_t (*lock) (ble_s_t* obj);
    hal_err_t (*unlock) (ble_s_t* obj);
    hal_err_t (*disconnect)(ble_s_t *obj);
    hal_err_t (*transmit)(ble_s_t *obj,uint8_t *buf, int16_t len);
    hal_err_t (*receive)(ble_s_t *obj,uint8_t *buf, int16_t size,uint32_t timeout);
    hal_err_t (*status)(ble_s_t *obj,bool *status);
    hal_err_t (*get_mac)(ble_s_t *obj,uint8_t *buf,uint8_t size);
    hal_err_t (*get_mtu)(ble_s_t *obj);
}ble_s_ops_t;

ble_s_t* ble_s_get_instance(void) ;

#endif //BLE_S_APP_H
