


#ifndef _BLE_C_APP_H_
#define _BLE_C_APP_H_

#include <stdint.h>
#include "app_freertos.h"

typedef enum
{
    BLE_C_START_CYCLE_SCAN_AND_RECEIVE = 0, //cycle to scan valid device around the router, if scanned, then connect and receive the data and upload
    BLE_C_STOP_CYCLE_SCAN_AND_RECEIVE,      //stop cycle scanning & connection & receiving
    BLE_C_SCAN_TO_SAVE_TARGET,              //save the target message after scaned the device
    BLE_C_GET_SCAN_STATUS_ONLY,                  //only to scan a any device to veriry scanning function
    BLE_C_CONNECT_BY_MAC,               //only to connect a target which is given
    BLE_C_UPLOAD_AS_SLAVE,
    BLE_C_DOWNLOAD_AS_CENTER,
    BLE_C_MAX
}ble_c_task_trig_type_t;

typedef struct
{
    bool active;
    bool next_run;      //1: can run next state
    bool up_load;
    bool down_load;
    bool connect;
}task_cycle_opt_flag_t;
typedef struct
{
    uint32_t scan_timeout;          //the timeout of once scan
    uint32_t scan_interval;         //if no device around the router,the scan interval
    uint32_t conn_timeout;          //the timeout of once connection
    uint32_t recv_target_timeout;   //the timeout of receive target data
    uint32_t recv_net_timeout;      //waiting for net response timeout
}ble_c_params_t;

typedef struct
{
    uint8_t addr[7];
    uint8_t level;
}scan_result_t;
typedef struct scan_result_list_s
{
    uint8_t addr[7];
    uint8_t level;
    struct scan_result_list_s *next;
}scan_result_list_t;

typedef struct
{
    int16_t vendor;
    int8_t ver;
    int8_t code;
    int16_t temp;
    int16_t reverse;
}slv_adv_t;

//ble_c_params default config
#define BLE_C_PARAMS_DEFAULT        \
{                                   \
    .scan_timeout   = 10000,        \
    .scan_interval  = 60000,        \
    .conn_timeout   = 10000,        \
    .recv_target_timeout = 3000,    \
    .recv_net_timeout   = 6000,     \
}

TaskHandle_t create_ble_c_task(void);
TaskHandle_t get_ble_c_task_handle(void);
void setting_ble_c_params(ble_c_params_t * p_params);
bool set_dev_name(char const *p_name);
#endif //_BLE_C_APP_H_