
#ifndef _SETTING_RIGHTS_H
#define _SETTING_RIGHTS_H

#include <stdint.h>
#include "init.h"
#include "app_freertos.h"

#define DEVICE_NUMBERS_MAX      100

typedef enum
{
    SETTING_MANAGE_MIN = 0,
    SETTING_ADD_ONE_DEV,
    SETTING_DEL_ONE_DEV,
    SETTING_DEL_ALL_DEV,
    SETTING_ADD_DEL_MAX
}setting_manage_type_t;
typedef enum
{
    SETTING_RIGHTS_MIN = 0,
    SETTING_RIGHTS_NOT_NEED,
    SETTING_RIGHTS_NEED,
    SETTING_RIGHTS_MAX
}setting_rights_type_t;
typedef struct
{
    //uint32_t timestamp;
    //uint32_t router_id;
    //uint32_t requester; //phone or server
    //uint8_t  manage_type;
    char     name[32];
    char     mac[14];
    uint8_t  mode;      //need right identity or not
    char     ip[16];  
}setting_params_t;

typedef struct
{
    uint32_t id;
    uint8_t  mode;
    uint8_t dummy[3];
}router_params_t;
typedef struct
{
    char *slv_mac;
}setting_repeat_t;

typedef struct mac_list_t
{
    uint8_t addr[8]; /**< 48-bit address, LSB format. */
    struct mac_list_t *next;
}mac_list_t;

void create_setting_rights_task(void);
bool judge_rights(void * arg);
TaskHandle_t get_setting_handle(void);
task_data_t *get_slave_settings(void);
mac_list_t * get_next_addr(void);
#endif //_SETTING_RIGHTS_H
