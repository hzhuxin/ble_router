/**
 * @brief 
 * 
 * @file opt_status.c
 * @date 2018-10-13
 * @author Chenfei
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

/********************************includes*************************************/
#include "execute_state.h"
#include "hal.h"

/********************************defines*************************************/

/********************************functions*************************************/

uint32_t opt_status_encode(uint8_t *const p_out, 
                         uint16_t size, 
                         proto_router_execute_state_t execute_state,
                         void *uuid)
{
    proto_router_exec_sta_rsp_t prot_execute_state;

    memset(&prot_execute_state,0,sizeof(proto_router_exec_sta_rsp_t));

    prot_execute_state.Iden.Token = 0;

    prot_execute_state.Iden.UUID.funcs.encode = encode_repeated_var_string;
    prot_execute_state.Iden.UUID.arg = (char*)uuid;

    pb_ostream_t            m_stream ;    
    m_stream = pb_ostream_from_buffer(p_out,size);
    bool res = pb_encode(&m_stream,proto_router_exec_sta_rsp_fields,&prot_execute_state);
    if(res)
    {
        return m_stream.bytes_written;
    }
    else
    {
        return 0;
    }
}