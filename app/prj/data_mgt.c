/**
 * 
 */

#include "app_freertos.h"
#include "net_manage.h"
#include "init.h"
#include "hal_cfg.h"
#include "hal_pin.h"
#include "hal_cpu.h"
#include "debug.h"
#include "ble_c_app.h"
#include "HexStr.h"
#include "hal_rtc.h"
#include "hal_uart.h"
#include "data_mgt.h"
#include "data_cache.h"

#define XFER_SLV_DEFAULT_NUM    10
#define XFER_SLV_BYTES          (XFER_SLV_DEFAULT_NUM* sizeof(slv_msg_t))  

DBG_SET_LEVEL(DBG_LEVEL_D);

static TaskHandle_t     data_manage_task_handle = NULL; 
static QueueHandle_t    msg_queue = NULL;
static xfer_pkg_t       *xp_buf = NULL;

void update_router_msg(router_msg_t *msg)
{
    memcpy(&msg->mac,hal_cpu_get_mac(),6);
    msg->timestamp = hal_rtc_get_time();
    msg->vol = 0;
}
void data_mgt_task_main(void *arg)
{
    cache_init();
    int32_t records;
    xfer_t xfer;
    static slv_msg_lst_t *node[XFER_SLV_DEFAULT_NUM] = {0};
    static slv_msg_t slv;
    xp_buf = (xfer_pkg_t *)pvPortMalloc(XFER_SLV_BYTES + sizeof(router_msg_t));
    if(!xp_buf)
        {return;}
    msg_queue = xQueueCreate(XFER_SLV_DEFAULT_NUM,sizeof(slv_msg_lst_t));
    if(!msg_queue)
        {return;}
    while(1)
    {
        records = cache_read_list_by_time(node,XFER_SLV_DEFAULT_NUM);
        if(records > 0 && records <= XFER_SLV_DEFAULT_NUM)
        {
            update_router_msg(&xp_buf->header);
            xfer.data = (uint8_t *)xp_buf;
            xfer.len = records * sizeof(slv_msg_t);
            xfer.max_size = XFER_SLV_BYTES;
            if(send_msg_to_server(&xfer))
            {
                cache_list_update_upload_state(node,XFER_SLV_DEFAULT_NUM);
            }
            else
            {
                DBG_W("Upload data fail!\r\n");
            }
            
        }
        while(xQueueReceive(msg_queue,&slv,1) == pdPASS)
        {
            cache_insert_list(&slv);
        }
    }
}
void create_data_mgt_task(void)
{
    if(xTaskCreate( data_mgt_task_main, "net", 512, NULL, 1, &data_manage_task_handle ) != pdPASS)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}
void send_msg_to_cache_queue(slv_msg_t *slv)
{
    xQueueSendToBack(msg_queue,slv,1);
}
//end
