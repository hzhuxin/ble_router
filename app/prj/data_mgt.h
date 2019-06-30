/**
 */

#ifndef _DATA_MGT_H
#define _DATA_MGT_H

#include <stdint.h>
#include "data_cache.h"

typedef struct
{
    uint8_t mac[6];
    int timestamp;
    uint16_t vol;
}router_msg_t;

typedef struct 
{
    router_msg_t header;
    slv_msg_t    data[0];
}xfer_pkg_t;

void create_data_mgt_task(void);
void send_msg_to_cache_queue(slv_msg_t *slv);

#endif //_DATA_MGT_H
