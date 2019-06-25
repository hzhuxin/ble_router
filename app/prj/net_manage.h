

#ifndef _NET_MANAGE_H_
#define _NET_MANAGE_H_

#include "app_freertos.h"
#include "hal.h"


void create_net_manage_task(void);
hal_err_t net_trans(uint8_t * buf,uint16_t len,uint16_t out_size);
TaskHandle_t *get_net_handle(void);
bool net_set_ip_port(char const *p_ip, uint32_t pport);
#endif //_NET_MANAGE_H_