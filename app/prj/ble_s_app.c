/**
 * @brief 
 * 
 * @file slave_manage.c
 * @date 2018-09-28
 * @author Chenfei
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

#include "ble_s_app.h"
#include "ble_s_obj.h"
#include "ble_gap.h"
#include "init.h"
#include "setting_rights.h"
#include "execute_state.h"

static TaskHandle_t     ble_s_handle = NULL;
static ble_s_t      *ble_s = NULL;

/*uint32_t encode(uint8_t *const p_out, 
                         uint16_t size, 
                         protocol_status_type_t status)
{
    protocol_status_req_t prot_status;
    char mac_addr[15];

    memset(mac_addr,0,sizeof(mac_addr));
    memset(&prot_status,0,sizeof(protocol_status_req_t));
    uint8_t addr[8];
    ble_s->ops->get_mac(ble_s,addr,sizeof(addr));
    HexToStr(mac_addr,addr,6);
    prot_status.Iden.Token = 0;

    prot_status.Iden.UUID.funcs.encode = encode_repeated_var_string;
    prot_status.Iden.UUID.arg = mac_addr;

    prot_status.StatusInfo = status;

    pb_ostream_t            m_stream ;    
    m_stream = pb_ostream_from_buffer(p_out,size);
    bool res = pb_encode(&m_stream,protocol_status_req_fields,&prot_status);
    if(res)
    {
        return m_stream.bytes_written;
    }
    else
    {
        return 0;
    }
}*/
/*static void response(protocol_status_type_t status)
{

    uint8_t buf[200];
    pkg_prot_head_t *prot_head = (pkg_prot_head_t *)buf;
    pkg_prot_frame_t *prot_frame = (pkg_prot_frame_t *)buf;
    uint16_t len = 0;
    char mac_addr[15];

    memset(mac_addr,0,sizeof(mac_addr));
    uint8_t addr[8];
    ble_s->ops->get_mac(ble_s,addr,sizeof(addr));
    HexToStr(mac_addr,addr,6);

    len = opt_status_encode(prot_frame->p_prot_data,sizeof(buf)-sizeof(pkg_prot_head_t),status,mac_addr);

    prot_head->cmd_type = PROTOCOL_HEADER_TYPE_TYPE_ROUTER_SETTING_RSP;
    prot_head->crc16 = crc16_compute((const uint8_t *)prot_head,sizeof(pkg_prot_head_t)-2,NULL);
    prot_head->crc16 = crc16_compute((const uint8_t *)prot_frame->p_prot_data,len,&prot_head->crc16);
    len += sizeof(pkg_prot_head_t);

    ble_s->ops->transmit(ble_s,buf,len);
}*/
void ble_s_rx_handler(uint8_t *buf, uint32_t len)
{
    task_data_t app_data;     
    task_trig_t task_notify;
    app_data.p_data = buf;
    app_data.length = len;
    task_notify.req_type = TASK_REQUEST_SETTING;
    task_notify.p_content = &app_data;
    task_notify.handle = (TaskHandle_t*)&ble_s_handle;
    xTaskNotify(xTaskGetHandle("net\0"), (uint32_t)&task_notify, eNoAction);
    task_trig_t notify_value;
    if(xTaskNotifyWait( 0,0,(uint32_t*)&notify_value,3000 ) == pdPASS)
    {
        //response(notify_value.req_type?
        //         PROTOCOL_STATUS_TYPE_SUCCESSED : PROTOCOL_STATUS_TYPE_FAILED);
        task_data_t *task_data = (task_data_t *)(notify_value.p_content);
        ble_s->ops->transmit(ble_s,task_data->p_data,task_data->length);
    }
    // else
    // {
    //     response(PROTOCOL_STATUS_TYPE_FAILED);
    // }
    
}
void ble_s_handle_task(void *arg)
{
    ble_s = ble_s_get_instance();
    if((ble_s->ops->lock(ble_s) == HAL_ERR_OK) && \
       (ble_s->ops->init(ble_s,NULL) == HAL_ERR_OK))
    {
        static uint8_t buf[800] = {0};
        static int16_t len = 0;
        static task_data_t app_data;     
        static task_trig_t task_notify;
        while(1)
        {
            len = ble_s->ops->receive(ble_s,buf,sizeof(buf),5000);
            if(len > 0)
            {
                app_data.p_data = buf;
                app_data.length = len;
                task_notify.req_type = TASK_REQUEST_SETTING;
                task_notify.p_content = &app_data;
                xTaskNotify(xTaskGetHandle("net\0"), (uint32_t)&task_notify, eNoAction);
            }
        }
    }
    ble_s->ops->deinit(ble_s);
}
void create_ble_s_task(void)
{
    if(xTaskCreate( ble_s_handle_task, "BS\0", 512, NULL, 1, &ble_s_handle ) != pdPASS)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    } 
}