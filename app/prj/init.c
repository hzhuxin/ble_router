/**
 * @brief 
 * 
 * @file init.c
 * @date 2018-09-28
 * @author Chenfei
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

#include "app_freertos.h"
#include "init.h"
#include "debug.h"
#include "hal_pin.h"
//#include "app_log.h"
#include "pca10056.h"
#include "ble_s_obj.h"
#include "ble_c_app.h"
#include "ble_s_app.h"
#include "net_manage.h"
#include "setting_rights.h"
#include "crc16.h"
#include "Define.pb.h"
#include "router_execsta.pb.h"
#include "pb_callback.h"
#include "HexStr.h"
#include "hal_wdt.h"
#include "hal_rtc.h"


#define R_RUN_INDEX     NRF_GPIO_PIN_MAP(1,9)
#define G_RUN_INDEX     NRF_GPIO_PIN_MAP(0,12)
DBG_SET_LEVEL(DBG_LEVEL_I);
//DBG_LOG_ENABLE(true);

static TaskHandle_t init_handle = NULL;
static TaskHandle_t ble_c_handle = NULL;
static TaskHandle_t wdt_handle = NULL;
static ble_s_t      *ble_s = NULL;

static uint32_t encode(uint8_t *const p_out, 
                         uint16_t size, 
                         proto_router_execute_state_t state)
{
    proto_router_exec_sta_rsp_t prot_state;
    char mac_addr[15];

    memset(mac_addr,0,sizeof(mac_addr));
    memset(&prot_state,0,sizeof(proto_router_exec_sta_rsp_t));
    uint8_t addr[8];
    ble_s->ops->get_mac(ble_s,addr,sizeof(addr));
    HexToStr(mac_addr,addr,6);
    prot_state.Iden.Token = 0;

    prot_state.Iden.UUID.funcs.encode = encode_repeated_var_string;
    prot_state.Iden.UUID.arg = mac_addr;

    prot_state.State = state;

    pb_ostream_t            m_stream ;    
    m_stream = pb_ostream_from_buffer(p_out,size);
    bool res = pb_encode(&m_stream,proto_router_exec_sta_rsp_fields,&prot_state);
    if(res)
    {
        return m_stream.bytes_written;
    }
    else
    {
        return 0;
    }
}
static void response(uint8_t *buf,uint16_t size,proto_router_execute_state_t status)
{
    if(buf == NULL)
    {
        return;
    }
    pkg_prot_head_t *prot_head = (pkg_prot_head_t *)buf;
    pkg_prot_frame_t *prot_frame = (pkg_prot_frame_t *)buf;
    uint16_t len = 0;

    len = encode(prot_frame->p_prot_data,size-sizeof(pkg_prot_head_t),status);

    prot_head->cmd_type = PROTOCOL_V2_HEADER_TYPE_TYPE_ACTIVE_RSP;
    prot_head->crc16 = crc16_compute((const uint8_t *)prot_head,sizeof(pkg_prot_head_t)-2,NULL);
    prot_head->crc16 = crc16_compute((const uint8_t *)prot_frame->p_prot_data,len,&prot_head->crc16);
    len += sizeof(pkg_prot_head_t);

    ble_s->ops->transmit(ble_s,buf,len);
}
static void wdt_callback(void)
{
    DBG_D("watch dog callback");
    uint32_t time_stamp = hal_rtc_get_time();
    //save_time();
}
static void wdt_handle_task(void *arg)
{
    static hal_wdt_cfg_t cfg = 
    {
      .timeout = 30000,
      .stop_when_sleep = 0,
      .stop_when_debug = 0,
    };
    hal_err_t ret = hal_wdt_init(&cfg,wdt_callback);
    DBG_I("watch dog init %d",ret);
    while(1)
    {
        DBG_D("...watch dog feed...");
        hal_wdt_feed();
        vTaskDelay(20000);
    }
}
void create_wdt_task(void)
{
    if(xTaskCreate( wdt_handle_task, "wdt", 64, NULL, 1, &wdt_handle ) != pdPASS)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}
void creat_app_task(void)
{
    create_setting_rights_task();
    ble_c_handle = create_ble_c_task();
    //create_ble_s_task();
    create_net_manage_task();
    //create_wdt_task();
}
void init_handle_task(void *arg)
{
    DBG_I("init_handle_task startup");
    hal_pin_set_mode(R_RUN_INDEX,HAL_PIN_MODE_OUT);
    hal_pin_write(R_RUN_INDEX,HAL_PIN_LVL_LOW);
    static task_trig_t task_notify;
#if NRF_MODULE_ENABLED(BLE_SLV)
    ble_s = ble_s_get_instance();
    if((ble_s->ops->lock(ble_s) == HAL_ERR_OK) && \
       (ble_s->ops->init(ble_s,NULL) == HAL_ERR_OK))
    {
        static uint8_t buf[500] = {0};
        static int32_t len = 0;
        hal_rtc_set_time(1514736000);
        while(1)
        {
            if(1)   //read active signal
            {
                //activation command
                creat_app_task();
                vTaskDelay(pdMS_TO_TICKS(1000));
                
                task_notify.req_type = TASK_REQUEST_CONNECT_NET_SERVER;
                //task_notify.req_type = TASK_REQUEST_BLE_C_START_SCAN_RECEIVE;
                task_notify.p_content = NULL;
                task_notify.handle = &init_handle;
                //TaskHandle_t handle = xTaskGetHandle("blec");
                //xTaskNotify(get_ble_c_task_handle(), (uint32_t)&task_notify, eSetBits);
                xTaskNotify(get_net_handle(), (uint32_t)&task_notify, eSetBits);                
                //ble_s->ops->unlock(ble_s);
                //ble_s->ops->deinit(ble_s);
                //vTaskDelete(NULL);
                vTaskDelay(1000);
                vTaskSuspend(init_handle);
            }
            len = ble_s->ops->receive(ble_s,buf,sizeof(buf),1000);
            if(len > 0)
            {
                pkg_prot_head_t *prot_head = (pkg_prot_head_t *)buf;
                //pkg_prot_frame_t *prot_frame = (pkg_prot_frame_t *)buf;
                uint16_t give_crc16 = prot_head->crc16;
                uint16_t cacul_crc16 = crc16_compute((const uint8_t *)prot_head,sizeof(pkg_prot_head_t)-2,NULL);
                if((cacul_crc16 == give_crc16) && (prot_head->cmd_type == PROTOCOL_V2_HEADER_TYPE_TYPE_ACTIVE_REQ))
                {
                    creat_app_task();
                    vTaskDelay(100);
                    task_trig_t task_notify;
                    task_notify.req_type = TASK_REQUEST_CONNECT_NET_SERVER;
                    task_notify.p_content = NULL;
                    task_notify.handle = init_handle;
                    response(buf,sizeof(buf),PROTO_ROUTER_EXECUTE_STATE_SUCCESSED);
                    xTaskNotify(xTaskGetHandle("net\0"), (uint32_t)&task_notify, eNoAction);
                    ble_s->ops->disconnect(ble_s);
                    ble_s->ops->deinit(ble_s);
                    ble_s->ops->unlock(ble_s);
                }
                else
                {
                    response(buf,sizeof(buf),PROTO_ROUTER_EXECUTE_STATE_FAILED);
                }
            }
            hal_pin_write(R_RUN_INDEX,HAL_PIN_LVL_HIGH);
            vTaskDelay(50);
            hal_pin_write(R_RUN_INDEX,HAL_PIN_LVL_LOW);
            vTaskDelay(1000);
        }
    }
#else
    while(1)
        {
            if(1)   //read active signal
            {
                //activation command
                creat_app_task();
                vTaskDelay(pdMS_TO_TICKS(1000));                
                //task_notify.req_type = TASK_REQUEST_CONNECT_NET_SERVER;
                task_notify.req_type = TASK_REQUEST_BLE_C_START_SCAN_RECEIVE;
                task_notify.p_content = NULL;
                task_notify.handle = &init_handle;
                //TaskHandle_t handle = xTaskGetHandle("blec");
                xTaskNotify(get_ble_c_task_handle(), (uint32_t)&task_notify, eSetBits);
                //xTaskNotify(get_net_handle(), (uint32_t)&task_notify, eSetBits);
                //vTaskDelete(NULL);
                vTaskDelay(1000);
                vTaskSuspend(init_handle);
            }
            hal_pin_write(R_RUN_INDEX,HAL_PIN_LVL_HIGH);
            vTaskDelay(50);
            hal_pin_write(R_RUN_INDEX,HAL_PIN_LVL_LOW);
            vTaskDelay(1000);
        }
#endif //NRF_MODULE_ENABLED(BLE_SLV)
}
void creat_init_task(void)
{ 
    if(xTaskCreate( init_handle_task, "actv", 256, NULL, 1, &init_handle ) != pdPASS)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }  
}