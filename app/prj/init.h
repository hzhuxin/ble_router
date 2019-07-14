
#ifndef _INIT_H
#define _INIT_H

#include <stdint.h>
#include "app_freertos.h"
//#include "net_manage.h"
//#include "ble_c_app.h"
//#include "ble_s_app.h"
//#include "setting_rights.h"

typedef enum
{
    TASK_REQUEST_MIN = 0,
    TASK_REQUEST_SETTING,
    TASK_REQUEST_SETTING_FROM_PHONE,
    TASK_REQUEST_SETTING_FROM_NET,
    TASK_REQUEST_RITGHTS_IDENTITY,
    TASK_REQUEST_CONNECT_NET_SERVER,
    TASK_REQUEST_DISCONNECT_NET_SERVER,
    TASK_REQUEST_UPLOAD_VIA_NET,
    TASK_REQUEST_NET_RESPONSE,
    TASK_REQUEST_UPLOAD_VIA_BLE,
    TASK_REQUEST_BLE_C_START_SCAN_ANLY,       //only scan to veriry the scanning function 
    TASK_REQUEST_BLE_C_START_SCAN_TARGET,     //to search some specified target is existing
    //TASK_REQUEST_STOP_BLE_SCAN,
    TASK_REQUEST_BLE_C_START_SCAN_RECEIVE,    //cycle to scan valid device around the router, if scanned, then connect and receive the data and upload
    TASK_REQUEST_BLE_C_STOP_SCAN_RECEIVE,     //stop cycle scanning & connection & receiving
    TASK_REQUEST_BLE_C_SCAN_TO_SAVE_TARGET,       //save the target message after scaned the device
    TASK_REQUEST_BLE_C_SCAN_OUT_DEVICE,
    TASK_REQUEST_BLE_C_SCAN_COMPLETE,
    TASK_REQUEST_BLE_C_CONNECTED,
    TASK_REQUEST_BLE_C_DISCONNECTED,
    TASK_REQUEST_BLE_C_TX,
    TASK_REQUEST_BLE_C_RX,
    TASK_REQUEST_IS_RIGHTS,
    TASK_REQUEST_NO_RIGHTS,
    TASK_REQUEST_RESPONSE,
    TASK_REQUEST_MAX,
}task_req_type_t;

typedef struct
{
    uint8_t *p_data;
    uint32_t length;
}task_data_t;
typedef struct task_trig
{
    task_req_type_t    req_type;
    void * p_content;
    void  *handle;
}task_trig_t;

/*
* _____________________________________________________________________________________________________________|___________________
*----------------------------------------head------------------------------------------------------------------|
* | protocal-version | vendor-id | command-type |   data-len   | encryption-select | reserve |   crc16         | protobuf-data
* |  accumulation    |   start   |              |  protobuf    |                   |         |caculate all     |
* |    from 1        |  from 1   |              | data length  |                   |         |data except self |
* ________________________________________________________________________________________________________________________________
*
*/ 
typedef struct
{
    uint16_t    prot_version;
    uint16_t    vendor_id;
    uint16_t    cmd_type;
    uint16_t    frame_len;
    uint8_t     option_encryption;
    uint8_t     reserve;
    uint16_t    crc16;
}pkg_prot_head_t;

typedef struct
{
    pkg_prot_head_t     prot_head;
    uint8_t             p_prot_data[0];
}pkg_prot_frame_t;

void creat_init_task(void);

#endif //_INIT_H