/**
 * @brief 
 * 
 * @file central_manage.c
 * @date 2018-09-28
 * @author Chenfei
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */


#include "debug.h"
#include "hal_pin.h"
#include "ble_c_obj.h"
#include "ble_s_obj.h"
#include "ble_init.h"
#include "ble_c_app.h"
#include "init.h"

#define S_RUN_INDEX     15
#define RUN_INDEX       17
#define SCAN_TIMEOUT    10000
#define CONN_TIMEOUT    10000
#define SCAN_TRIGGER_TIMEOUT    2000
#define BLE_RECEIVE_TIMEOUT     3000
#define WAIT_NET_BACK_TIMEOUT   6000

#define CHECK_POINTER(p)            \
{                                   \
  if(!p)                            \
  {return HAL_ERR_PARAM;}           \
}

static TaskHandle_t         ble_c_handle = NULL;
//static QueueHandle_t        wait_net_queue = NULL;
static SemaphoreHandle_t    scan_sema = NULL;
//static SemaphoreHandle_t    scan_trigger_sema = NULL;
static uint8_t              target_addr[BLE_GAP_ADDR_LEN];
static ble_c_t              * ble_c = NULL;
static char                 name[] = "DRUID_C_S\0";
static uint16_t             conn = BLE_CONN_HANDLE_INVALID;
static scan_target_t        scan_target = SCAN_TARGET_MSG_DEFAULT;
static connect_target_t     conn_target;
static ble_c_params_t       ble_c_params = BLE_C_PARAMS_DEFAULT;

/**@brief Parameters used when scanning. */
static ble_gap_scan_params_t const m_scan_params =
{
    .extended      = 0,
    .active        = 1,
    .interval      = 100,
    .window        = 1200,
    .timeout       = 0,
    .scan_phys     = BLE_GAP_PHY_1MBPS,
    .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
};
/**@brief Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param =
{
    MIN_CONNECTION_INTERVAL,
    MAX_CONNECTION_INTERVAL,
    SLAVE_LATENCY,
    SUPERVISION_TIMEOUT
};
void setting_ble_c_params(ble_c_params_t * p_params)
{
    //memcpy((uint8_t *)&ble_c_params,(uint8_t *)p_params,sizeof(ble_c_params));
    if(p_params->scan_timeout != 0)
    {
        DBG_I("set ble_c scan_timeout = %d",p_params->scan_timeout);
        ble_c_params.scan_timeout   = p_params->scan_timeout;
    }
    if(p_params->scan_interval != 0)
    {
        DBG_I("set ble_c scan_interval = %d",p_params->scan_interval);
        ble_c_params.scan_interval  = p_params->scan_interval;
    }
    if(p_params->conn_timeout != 0)
    {
        DBG_I("set ble_c conn_timeout = %d",p_params->conn_timeout);
        ble_c_params.conn_timeout   = p_params->conn_timeout;
    }
    if(p_params->recv_target_timeout != 0)
    {
        DBG_I("set ble_c recv_target_timeout = %d",p_params->recv_target_timeout);
        ble_c_params.recv_target_timeout = p_params->recv_target_timeout;
    }
    if(p_params->recv_net_timeout != 0)
    {
        DBG_I("set ble_c recv_net_timeout = %d",p_params->recv_net_timeout);
        ble_c_params.recv_net_timeout   = p_params->recv_net_timeout;
    }
}
static void scan_handler(target_t const * p_content)
{
    DBG_I("scan complete");
    if(p_content != NULL)
    {
        //scan_flag = 1;
        memcpy(target_addr,p_content->peer_addr->addr,BLE_GAP_ADDR_LEN)
        DBG_I("target name: %s",p_content->name);
        DBG_I("target addr: %02x:%02x:%02x:%02x:%02x:%02x\r\n",
                             p_content->peer_addr->addr[0],
                             p_content->peer_addr->addr[1],
                             p_content->peer_addr->addr[2],
                             p_content->peer_addr->addr[3],
                             p_content->peer_addr->addr[4],
                             p_content->peer_addr->addr[5]
                             );
        //if(p_content->name[strlen(name)] == 0x31)    //"1": means slave has data to transmition
        {
            if(scan_sema == NULL)
            {
                scan_sema = xSemaphoreCreateBinary();
            }
            xSemaphoreGive(scan_sema);
            ble_c->ops->scan_stop(ble_c);
        }
    }
    else
    {
        ble_c->ops->scan_stop(ble_c);
    }
}
static bool ble_c_scan(ble_c_t *ble_c,scan_target_t *scan_target)
{
    bool ret = false;
    if(ble_c == NULL || scan_target == NULL)
    {
        DBG_E("ble_c_scan, pointer OBJ = NULL, or scan_target= NULL")
        return false;
    }
    if(scan_sema == NULL)
    {
        scan_sema = xSemaphoreCreateBinary();
    }
    if(ble_c->ops->scan(ble_c,scan_target,SCAN_TIMEOUT) == HAL_ERR_OK)
    {
        if(xSemaphoreTake(scan_sema,SCAN_TIMEOUT) == pdPASS)
        {
            ret = true;
        }
    }
    DBG_E("ble_c_scan %s",ret?"success":"failed");
    return ret;
}
static bool send_config(void)
{
    bool ret = false;
    //get config message status
    return ret;
}

static void unvar_transmition(void)
{
    if(ble_c == NULL )
    {
        DBG_E("ble_c_scan, pointer OBJ = NULL")
        return;
    }
    uint8_t buf[BLE_TX_BUF_SIZE];
    int len = 0;
    //ble_data_t net_data;
    while(1)
    {
        len = ble_c->ops->receive(ble_c,conn,buf,sizeof(buf),BLE_RECEIVE_TIMEOUT);
        if((len <= 0) || (len > BLE_RX_BUF_SIZE))
        {
            break;
        }
        //transmit data via net, and receive data from net
        len = net_trans(buf,len,sizeof(buf));
        //len = net_trans(&wait_net_queue,buf,len);
        // if(xQueueReceive(c_rx_queue, &net_data , WAIT_NET_BACK_TIMEOUT) != pdPASS)
        // {
        //     break;
        // }
        if((len <= 0) || (len > BLE_RX_BUF_SIZE))
        {
            break;
        }
        if(ble_c->ops->transmit(ble_c,conn,buf,len) != HAL_ERR_OK)
        {
            break;
        }
    }
}
static void scan_to_receive(void)
{
    if(ble_c == NULL )
    {
        DBG_E("ble_c_scan, pointer OBJ = NULL")
        return;
    }
    //uint32_t timeout;
    //while(1)
    {
        if(ble_c_scan(ble_c,&scan_target) == true)
        {
            conn_target.addr = target_addr;
            //get rights
            if(ble_c->ops->connect(ble_c,&conn_target,CONN_TIMEOUT) == HAL_ERR_OK)
            {
                send_config();
                unvar_transmition();
                ble_c->ops->disconnect(ble_c,&conn);
            }
            //timeout = 1;
        }       
        // else
        // {
        //     //if no terminal device, the long interval time will be used,it can be set
        //     timeout = 60000;    
        // }
        // DBG_E("scan_to_receive, waiting interval to scan next target")
        // vTaskDelay(timeout);
    }
}
static void ble_c_handle_task(void *arg)
{
    //get instance
    //ble_c_cfg_t config  = NULL;
    // config.rx_handler = NULL;
    // config.conn_handler = NULL;
    // config.disc_handler = NULL;
    // config.tx_timeout = 0;
    ble_c = ble_c_get_instance(0);
    if((ble_c) && \
       (ble_c->ops->lock(ble_c) == HAL_ERR_OK) && \
       (ble_c->ops->init(ble_c,NULL) == HAL_ERR_OK) \
       )
    {
        DBG_I("ble_c_handle_task startup");
        //scan parameters
        scan_target.name = name;
        //scan_target.addr = addr;
        scan_target.scan_params = (ble_gap_scan_params_t *)&m_scan_params;
        scan_target.mode = NEED_ONLY_NAME;
        scan_target.handler = scan_handler;        //若使用回调，则不进入柱塞

        //connect parameters
        conn = BLE_CONN_HANDLE_INVALID;
        memset(&conn_target,0,sizeof(connect_target_t));
        conn_target.scan_params = (ble_gap_scan_params_t *)&m_scan_params;
        conn_target.conn_params = (ble_gap_conn_params_t *)&m_connection_param;
        conn_target.conn_handle = &conn;

        //uint32_t notify_value; 
        static task_trig_t notify_value;
        TickType_t wait_tick = portMAX_DELAY;
        task_cycle_opt_flag_t opt_flag;
        while(1)
        {
            //if(xSemaphoreTake(scan_trigger_sema,SCAN_TRIGGER_TIMEOUT) == pdPASS)
            if(xTaskNotifyWait( 0,0,(uint32_t *)&notify_value,wait_tick ) == pdPASS)
            {
                DBG_I("take notity pass, notity value %d",notify_value.req_type);
                switch(notify_value.req_type)
                {
                    case TASK_REQUEST_BLE_C_START_SCAN_ANLY:
                        break;
                    case TASK_REQUEST_BLE_C_START_BLE_SCAN_TARGET:
                        break;
                    case TASK_REQUEST_BLE_C_SCAN_TO_SAVE_TARGET:
                        break;
                    case TASK_REQUEST_BLE_C_START_SCAN_RECEIVE:
                        //scan_to_receive();
                        opt_flag.scan_receive = 1;
                        break;
                    case TASK_REQUEST_BLE_C_STOP_SCAN_RECEIVE:
                        opt_flag.scan_receive = 0;
                        break;
                    case TASK_REQUEST_BLE_C_DOWNLOAD:
                        break;
                    case TASK_REQUEST_BLE_C_UPLOAD:
                        break;
                    default:
                        break;
                }
            }

            wait_tick = portMAX_DELAY;
            if(opt_flag.scan_receive)
            {
                scan_to_receive();
                wait_tick = 0;
            }
            //if()  //other task operate type flag
        }
    }
    ble_c->ops->deinit(ble_c);
    ble_c->ops->lock(ble_c);
}

TaskHandle_t create_ble_c_task(void)
{
    if(xTaskCreate( ble_c_handle_task, "blec\0", 1024, NULL, 1, &ble_c_handle ) != pdPASS)
    {
        return NULL;
    }
    return ble_c_handle;
}

TaskHandle_t get_ble_c_task_handle(void)
{
    return ble_c_handle;
}