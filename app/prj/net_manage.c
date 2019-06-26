/**
 * @brief 
 * 
 * @file net_manage.c
 * @date 2018-09-28
 * @author Chenfei
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

/*********************************includes***************************************/
#include "app_freertos.h"
//#include "app_log.h"
#include "net_manage.h"
#include "init.h"
//#include "cell.h"
#include "hal_cfg.h"
#include "hal_pin.h"
#include "debug.h"
#include "setting_rights.h"
#include "ble_c_app.h"
#include "HexStr.h"
#include "hal_rtc.h"

/*********************************defines***************************************/
#define HEART_BEAT_SLOT     5000
DBG_SET_LEVEL(DBG_LEVEL_D);

static TaskHandle_t net_handle = NULL;
//static int16_t connid = -1;
//static char *pHost = "47.52.138.210";
//static uint16_t port = 4190;
static char pHost[16] = "47.52.138.210";
static uint16_t port = 9510;
static uint8_t net_buf[1024];
static bool activ_flag = 0;
static bool connect_falg = 0;
//static task_trig_t task_notify;

/**********************************functions**************************************/
bool net_set_ip_port(char const *p_ip, uint32_t pport)
{
    if(p_ip)
    {
        if(strlen(p_ip) > sizeof(pHost))
        {
            return false;
        }
        strcpy(pHost,p_ip);
    } 
    port = pport;
    return true;
}
static bool net_start(void)
{
//     if(!CELL_Reset() || !CELL_Init())
//     {
//         DBG_I("net reset and init failed\r\n");
//         return false;
//     }
//     DBG_I("net reset and init ok\r\n");
//     if(!CELL_Register() || !CELL_Prepare())
//     {
//         DBG_I("net register or prepare failed\r\n");
//         DBG_I("remain stack size = %d",uxTaskGetStackHighWaterMark(NULL));
//         return false;
//     }   
//     DBG_I("net register or prepare ok\r\n");
//     DBG_I("connect host: %s, port=%d\r\n",pHost, port);
//     connid = CELL_Connect(pHost, port);
//     //*connid = CELL_Connect("www.baidu.com", 80);
//     if(connid < 0)
//     {
//         DBG_I("net connect failed\r\n");
//         DBG_I("net_start..., remain stack size = %d\r\n",uxTaskGetStackHighWaterMark(NULL));
//         return false;
//     }
//     DBG_I("net connect ok, connid= %d\r\n",connid);
//     return true;
}
void net_close(void)
{
    // CELL_Close(connid);
    // CELL_Shutdown();
}
void notity_ble_stop(void)
{
    static task_trig_t task_notify;

    memset(&task_notify,0,sizeof(task_notify));
    task_notify.req_type = TASK_REQUEST_BLE_C_STOP_SCAN_RECEIVE;
    task_notify.p_content = NULL;
    task_notify.handle = net_handle;
    xTaskNotify(get_ble_c_task_handle(), (uint32_t)&task_notify, eSetBits);
    DBG_I("notited ble of stop\r\n");
}
void re_connect(void)
{
    notity_ble_stop();
    
    static task_trig_t task_notify;
    activ_flag = 0;
    connect_falg = 0;
    memset(&task_notify,0,sizeof(task_notify));
    task_notify.req_type = TASK_REQUEST_CONNECT_NET_SERVER;
    task_notify.p_content = NULL;
    task_notify.handle = net_handle;
    xTaskNotify(net_handle, (uint32_t)&task_notify, eSetBits);
}

bool net_heart_beat(void)
{
    // uint16_t i;
    // for( i=0;i<20;i++)
    // {
    //     net_buf[i] = i;
    // }   
    // int len = CELL_Send(connid,(char *)net_buf,i);
    // if(len != i)
    // {
    //     DBG_E("net send failed of %d",len);
    //     return false;
    // }
    return true;
   // return ((CELL_Send(connid,(char *)net_buf,i) == i)? true : false);
}
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

    int ret = CELL_Send(connid,buf,len);
}*/
void net_rx_handler(void * arg)
{    
    static task_trig_t task_notify;
    task_notify.handle = (TaskHandle_t*)&net_handle;
    task_notify.p_content = (task_data_t *)arg;

    memset(&task_notify,0,sizeof(task_notify));
    task_notify.req_type = TASK_REQUEST_BLE_C_TX;
    xTaskNotify(get_ble_c_task_handle(), (uint32_t)&task_notify, eNoAction);

}
bool net_tx(void *arg)
{
    if(!arg)
    {
        DBG_I("net_tx pointer is null\r\n");
    }
    task_data_t *p_net = (task_data_t *)arg;
    int ret;// = CELL_Send(connid,(char *)p_net->p_data,p_net->length);
    if(ret != p_net->length)
    {
        DBG_I("net test of send data failed, len = %d,ret = %d\r\n",p_net->length,ret);
        return false;
    }
    return true;
}
bool net_send(uint8_t *p_tx, uint16_t len)
{
    if(!p_tx)
    {
        DBG_I("net_send pointer is null\r\n");
    }
    char buf[200];
    HexToStr(buf,p_tx,len);
    DBG_I("%s",buf);
    int ret ;//= CELL_Send(connid,(char *)p_tx,len);
    if(ret != len)
    {
        DBG_I("net test of send data failed, len = %d,ret = %d\r\n",len,ret);
        return false;
    }
    return true;
}
int32_t net_wait_res(uint8_t *net_buf, uint16_t buf_size)
{
    int32_t time = 10;
    int32_t len = 0;
    //while(!util_timeout(start, xTaskGetTickCount(), 10000))
    while(time-- > 0)
    {
        DBG_I("waiting response... %d S\r\n",time);
        //len = CELL_Recv(connid,(char *)net_buf,buf_size);
        if(len > 0)
        {
            DBG_I("received response\r\n");
            return len ;
        }
        vTaskDelay(1000);
    }
    return -1;
}
void net_upload(task_data_t *priv)
{
    if(!priv)
    {
        DBG_E("net upload pointer is NULL\r\n");
        return;
    }
    static task_trig_t task_notify;
    static task_data_t net_data;
    uint32_t len = 0;
    if(net_tx(priv) == true)
    {
        len = net_wait_res(net_buf,sizeof(net_buf));
        if(len > 0)
        {
            memset(&task_notify,0,sizeof(task_notify));
            net_data.p_data = net_buf;
            net_data.length = len;
            task_notify.handle = net_handle;
            task_notify.p_content = &net_data;
            task_notify.req_type = TASK_REQUEST_BLE_C_TX;
            xTaskNotify(get_ble_c_task_handle(), (uint32_t)&task_notify, eSetBits);
            DBG_I("------net received from server(0x%x),len=%d-----:\r\n",task_notify.p_content,len);
            return;
        }
    }
}
void net_test(void)
{
    hal_pin_set_mode(HAL_CFG_CELL_TXD,HAL_PIN_MODE_OUT);
    hal_pin_set_mode(HAL_CFG_CELL_RXD,HAL_PIN_MODE_IN);
    hal_pin_set_mode(15,HAL_PIN_MODE_OUT);
    hal_pin_set_mode(17,HAL_PIN_MODE_IN);
   // CELL_Reset();
    vTaskDelay(1000);
    while(1)
    {
        if(hal_pin_read(17))
        {
            hal_pin_write(HAL_CFG_CELL_TXD,HAL_PIN_LVL_HIGH);
        }
        else
        {
            hal_pin_write(HAL_CFG_CELL_TXD,HAL_PIN_LVL_LOW);
        }
        if(hal_pin_read(HAL_CFG_CELL_RXD))
        {
            hal_pin_write(15,HAL_PIN_LVL_HIGH);
        }
        else
        {
            hal_pin_write(15,HAL_PIN_LVL_LOW);
        }
    }
}
void net_handle_task(void *arg)
{
    static task_trig_t *notify_value = NULL;
    static task_trig_t task_notify;
    static int len = 0;
    static task_data_t net_data; 
    static TickType_t wait_tick = portMAX_DELAY;
    static TickType_t start_time;
    //net_test();
    while(1)
    {
        DBG_D("net manage waiting notify, wait_tick = %d\r\n",wait_tick);
        if(xTaskNotifyWait( 0,ULONG_MAX,(uint32_t *)&notify_value,wait_tick ) == pdPASS)
        {
            DBG_I("NET task notitied type %d from task \"%s\"\r\n",notify_value->req_type,pcTaskGetName(notify_value->handle));
            switch(notify_value->req_type)
            {
                case TASK_REQUEST_CONNECT_NET_SERVER:
                    DBG_I("TASK_REQUEST_CONNECT_NET_SERVER\r\n");
                    activ_flag = 1;
                    wait_tick = 1000;  //if no connect, trying to connect every 60S           
                    break;
                case TASK_REQUEST_DISCONNECT_NET_SERVER:
                    DBG_I("TASK_REQUEST_DISCONNECT_NET_SERVER\r\n");
                    net_close();
                    wait_tick = portMAX_DELAY;
                    task_notify.req_type = TASK_REQUEST_BLE_C_STOP_SCAN_RECEIVE;
                    task_notify.p_content = NULL;
                    task_notify.handle = net_handle;
                    xTaskNotify(get_ble_c_task_handle(), (uint32_t)&task_notify, eSetBits);
                    break;
                case TASK_REQUEST_UPLOAD_VIA_NET:
                    DBG_I("TASK_REQUEST_UPLOAD_VIA_NET\r\n");
                    task_data_t *p_net = (task_data_t *)notify_value->p_content; 
                    memcpy(net_buf,p_net->p_data,p_net->length);
                    net_data.p_data = net_buf;
                    net_data.length = p_net->length;
                    DBG_I("-----net received from BLEC(0x%x),len=%d---:\r\n",p_net,p_net->length);
                    net_upload(&net_data); 
                    //net_upload(notify_value->p_content);                     
                    break;
                case TASK_REQUEST_NET_RESPONSE:
                    DBG_I("TASK_REQUEST_NET_RESPONSE\r\n");
                    net_rx_handler(notify_value->p_content);
                    break;
                default :
                    break;
            }
            notify_value = NULL;
        }
        if(connect_falg)
        {
            //len = CELL_Recv(connid,(char *)net_buf,sizeof(net_buf));
            if(len > 0)
            {               
                net_data.p_data = net_buf;
                net_data.length = len;
                net_rx_handler(&net_data);
            }
            else if(len < 0)
            {
                connect_falg = 0;
                activ_flag = 0;
                re_connect();
            }
            if((xTaskGetTickCount() - start_time) > HEART_BEAT_SLOT)
            {
                DBG_I("net heart beet\r\n");
                if(!net_heart_beat())
                {
                    re_connect();
                }
                start_time = xTaskGetTickCount();
            }
        }   
        else if(activ_flag)
        {
            DBG_I("net starting......\r\n");
            wait_tick = 1000;
            if(net_start())
            {
                //if connect success, set wait tick 1000mS                       
                connect_falg = 1;
                task_notify.req_type = TASK_REQUEST_BLE_C_START_SCAN_RECEIVE;
                task_notify.p_content = NULL;
                task_notify.handle = net_handle;
                xTaskNotify(get_ble_c_task_handle(), (uint32_t)&task_notify, eSetBits);
                start_time = xTaskGetTickCount();
            }
            else
            {
                net_close();
                connect_falg = 0;
                wait_tick = 5000;
            }
        }   
    }
}
void create_net_manage_task(void)
{ 
    if(xTaskCreate( net_handle_task, "net", 512, NULL, 1, &net_handle ) != pdPASS)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}

TaskHandle_t *get_net_handle(void)
{
    return net_handle;
}
//end
