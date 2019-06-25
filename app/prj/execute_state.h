
#ifndef _OPT_STATUS_H
#define _OPT_STATUS_H

#include "router_execsta.pb.h"
#include "pb_callback.h"
uint32_t opt_status_encode(uint8_t *const p_out, 
                         uint16_t size, 
                         proto_router_execute_state_t execute_state,
                         void *uuid);

#endif //_OPT_STATUS_H