/**
 * @brief 
 * 
 * @file central_manage.c
 * @date 2018-09-28
 * @author Chenfei
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */


#include "debug.h"
//#include "app_log.h"
#include "hal_pin.h"
#include "ble_c_obj.h"
#include "ble_s_obj.h"
#include "ble_init.h"
#include "ble_c_app.h"
#include "init.h"
#include "HexStr.h"
#include "setting_rights.h"
#include "crc16.h"
#include "net_manage.h"
#include "data_cache.h"
#include "data_mgt.h"
#include "hal_rtc.h"

#define S_RUN_INDEX     15
#define RUN_INDEX       17
#define SCAN_TIMEOUT    10000
#define CONN_TIMEOUT    10000
#define SCAN_TRIGGER_TIMEOUT    2000
#define BLE_RECEIVE_TIMEOUT     21000
#define NET_RESPONSE_TIMEOUT    20000

DBG_SET_LEVEL(DBG_LEVEL_E);

#define CHECK_POINTER(p)            \
{                                   \
  if(!p)                            \
  {return HAL_ERR_PARAM;}           \
}

static TaskHandle_t             ble_c_handle = NULL;
//static QueueHandle_t          wait_net_queue = NULL;
//static SemaphoreHandle_t      scan_sema = NULL;
//static SemaphoreHandle_t      scan_trigger_sema = NULL;
//static uint8_t                target_addr[BLE_GAP_ADDR_LEN];
static ble_c_t                  *ble_c = NULL;
static char                     name[] = "XZL3\0";
static uint16_t                 conn = BLE_CONN_HANDLE_INVALID;
static scan_target_t            scan_target = SCAN_TARGET_MSG_DEFAULT;
static connect_target_t         conn_target;
static ble_c_params_t           ble_c_params = BLE_C_PARAMS_DEFAULT;
static TickType_t               start_time = 0;
static task_cycle_opt_flag_t    opt_flag;
//static scan_result_list_t       m_scan_result[DEVICE_NUMBERS_MAX];
static scan_result_list_t       * m_scan_result_list = NULL;
static scan_result_list_t       * max_lvl_node = NULL;
static uint16_t                 scan_result_cnt = 0;
//static slv_msg_t                slv;
/**@brief Parameters used when scanning. */
static ble_gap_scan_params_t const m_scan_params =
{
    .extended      = 0,
    .active        = 0,
    .interval      = 1000,
    .window        = 50,
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
bool set_dev_name(char const *p_name)
{
    if(strlen(p_name) > 31)
    {
        DBG_E("set_dev_name: invalide device name length");
        return false;
    }
    strcpy(name,p_name);
    return true;
}
static bool scan_result_list_init(void)
{
    if(m_scan_result_list != NULL)
    {
        return true;
    }
    vPortFree(m_scan_result_list);
    m_scan_result_list = (scan_result_list_t *)pvPortMalloc(sizeof(scan_result_list_t));
    if(m_scan_result_list == NULL)
    {
        DBG_I("link list init failed\r\n");
        return false;
    }
    m_scan_result_list->level = 0;
    m_scan_result_list->next = NULL;
    DBG_I("link list init successed\r\n");
    return true;
}
/**
 * @brief 
 * 
 * @param mac 
 * @return scan_result_list_t* 
 */
static scan_result_list_t * scan_result_list_search(uint8_t const *addr)
{
    if(m_scan_result_list == NULL)
    {
        return NULL;
    }
    scan_result_list_t *p = m_scan_result_list;
    for(;p != NULL;p=p->next)
    {
        if(memcmp(addr,p->addr,6) == 0)
        {
            return p;
        }
    }
    return NULL;
}
/**
 * @brief 
 * 
 * @param mac 
 * @return true 
 * @return false 
 */
static bool scan_result_list_insert(uint8_t const *addr, uint16_t level)
{
    if(m_scan_result_list == NULL)
    {
        if(scan_result_list_init() != true)
        {
            return false;
        }
    }
    if(scan_result_list_search(addr) != NULL)
    {
        DBG_D("This scanned device is already existed \r\n");
        return true;
    }
    scan_result_list_t *node = NULL;
    scan_result_list_t *p = NULL;
    for(p = m_scan_result_list;p->next != NULL;p=p->next)
    {
        ;
        DBG_I("linklist address p = 0x%x, p->next = 0x%x\r\n",p,p->next);
    }
    node = (scan_result_list_t *)pvPortMalloc(sizeof(scan_result_list_t));;
    if(node == NULL)
    {
        DBG_E("pvPortMalloc for new list node failed\r\n");
        return false;
    }
    memcpy(node->addr,addr,6);
    node->level = level;
    p->next = node;
    node->next = NULL;
    scan_result_cnt++;
    DBG_I("Insert device %02x:%02x:%02x:%02x:%02x:%02x successed",
            addr[0],addr[1],addr[2],addr[3],addr[4],addr[5]);
    DBG_I("Insert No. %d device at addr(0x%x) successed\r\n",scan_result_cnt-1,node);
    return true;
}
static scan_result_list_t *get_scan_result_max_level(void)
{
    if(m_scan_result_list == NULL)
    {
        DBG_E("m_scan_result_list is NULL\r\n");
        return NULL;
    }
    if(m_scan_result_list->next == NULL)
    {
        DBG_E("there is no target in list.\r\n");
        return NULL;
    }
    scan_result_list_t *p = m_scan_result_list;
    scan_result_list_t *max_node = m_scan_result_list;
    p=p->next;  //first node has no value   
    for(;p != NULL;p=p->next)
    {
        if(p->level > max_node->level)
        {
            max_node = p;
        }
    }
    return max_node;
}
static bool scan_result_list_delete(scan_result_list_t *node)
{
    if(node == NULL)
    {
        DBG_E("list_delete: The node pointer is NULL\r\n");
        return false;
    }
    scan_result_list_t *pos_front = m_scan_result_list;
    for(;!pos_front->next;pos_front = pos_front->next)
    {
        if(pos_front->next == node)
        {
            break;
        }
    }
    if(pos_front->next != node)
    {
        DBG_E("list_delete: not found the node to delete\r\n");
        return false;       
    }
    pos_front->next = node->next;
    DBG_I("list_delete: delete node %02x:%02x:%02x:%02x:%02x:%02x succeed\r\n",
            node->addr[0],node->addr[1],node->addr[2],node->addr[3],node->addr[4],node->addr[5]);
    vPortFree(node);
    node = NULL;
    scan_result_cnt--;
    return true;
}
static int scan_rerults_all_delete(scan_result_list_t *node)
{
    if(node == NULL)
    {
        return 0;
    }
    if(!scan_rerults_all_delete(node->next))
    {
        DBG_I("list delete all: node %02x:%02x:%02x:%02x:%02x:%02x succeed\r\n",
            node->addr[0],node->addr[1],node->addr[2],node->addr[3],node->addr[4],node->addr[5]);
        memset(node,0,sizeof(mac_list_t));
        vPortFree(node);
        node = NULL;
    }
    scan_result_cnt = 0;
    return 0;
}
static void ble_c_stop_scan(void)
{
    DBG_I("scan timeout, stop scanning");
    ble_c->ops->scan_stop(ble_c);
}
static void scan_handler(target_t const * p_content)
{
    static task_trig_t task_notify;
    if(p_content != NULL)
    {
        task_notify.req_type = TASK_REQUEST_BLE_C_SCAN_OUT_DEVICE;
        task_notify.p_content = p_content;
        task_notify.handle = xTaskGetCurrentTaskHandle();
        xTaskNotify(ble_c_handle, (uint32_t)&task_notify, eSetBits);
    }
    else
    {
        ble_c_stop_scan();
        memset(&task_notify,0,sizeof(task_notify));
        task_notify.req_type = TASK_REQUEST_BLE_C_SCAN_COMPLETE;
        task_notify.p_content = NULL;
        task_notify.handle = xTaskGetCurrentTaskHandle();
        xTaskNotify(ble_c_handle, (uint32_t)&task_notify, eSetBits);
    }
}
void ble_c_conn_callback(void *arg)
{
    static task_trig_t task_notify;
    memset(&task_notify,0,sizeof(task_trig_t));

    conn = *(uint16_t *)arg;

    task_notify.p_content = NULL;
    task_notify.handle = ble_c_handle;
    task_notify.req_type = TASK_REQUEST_BLE_C_CONNECTED;
    xTaskNotify(ble_c_handle, (uint32_t)&task_notify, eSetBits);
}
void ble_c_rx_callback(ble_data_t * p_ble_data)
{
    static task_trig_t task_notify;
    memset(&task_notify,0,sizeof(task_trig_t));
    task_notify.p_content = p_ble_data;
    task_notify.req_type = TASK_REQUEST_BLE_C_RX;
    xTaskNotify(ble_c_handle, (uint32_t)&task_notify, eSetBits);
    //DBG_I("ble_c_rx_handler:ble received data, give notify TASK_REQUEST_BLE_C_RX");
}
void ble_c_disc_callback(void *arg)
{
    static task_trig_t task_notify;
    memset(&task_notify,0,sizeof(task_trig_t));
    task_notify.p_content = NULL;
    task_notify.handle = xTaskGetCurrentTaskHandle();
    task_notify.req_type = TASK_REQUEST_BLE_C_DISCONNECTED;
    xTaskNotify(ble_c_handle, (uint32_t)&task_notify, eSetBits);
    //DBG_I("ble_c_rx_handler:ble received data, give notify TASK_REQUEST_BLE_C_RX");
}

void setting_ble_c_params(ble_c_params_t * p_params)
{
    //memcpy((uint8_t *)&ble_c_params,(uint8_t *)p_params,sizeof(ble_c_params));
    if(p_params->scan_timeout != 0)
    {
        DBG_I("set ble_c scan_timeout = %d\r\n",p_params->scan_timeout);
        ble_c_params.scan_timeout   = p_params->scan_timeout;
    }
    if(p_params->scan_interval != 0)
    {
        DBG_I("set ble_c scan_interval = %d\r\n",p_params->scan_interval);
        ble_c_params.scan_interval  = p_params->scan_interval;
    }
    if(p_params->conn_timeout != 0)
    {
        DBG_I("set ble_c conn_timeout = %d\r\n",p_params->conn_timeout);
        ble_c_params.conn_timeout   = p_params->conn_timeout;
    }
    if(p_params->recv_target_timeout != 0)
    {
        DBG_I("set ble_c recv_target_timeout = %d\r\n",p_params->recv_target_timeout);
        ble_c_params.recv_target_timeout = p_params->recv_target_timeout;
    }
    if(p_params->recv_net_timeout != 0)
    {
        DBG_E("set ble_c recv_net_timeout = %d\r\n",p_params->recv_net_timeout);
        ble_c_params.recv_net_timeout   = p_params->recv_net_timeout;
    }
}
static bool ble_c_search_target(void)
{
    static int err_cnt;
    scan_target.name = name;
    //scan_target.addr = addr;
    scan_target.scan_params = (ble_gap_scan_params_t *)&m_scan_params;
    scan_target.mode = NEED_ONLY_NAME;
    scan_target.handler = scan_handler;        //若使用回调，则不进入柱塞
    if(ble_c->ops->scan(ble_c,&scan_target,SCAN_TIMEOUT) == HAL_ERR_OK)
    {
        DBG_I("BLE App:Start scan success");
        return true;
    }
    else
    {
        if(err_cnt++ >=3 )
        {
            hal_cpu_reset();
        }
        else
        {
            task_trig_t task_notify;
            task_notify.req_type = TASK_REQUEST_BLE_C_SCAN_COMPLETE;
            task_notify.p_content = NULL;
            task_notify.handle = ble_c_handle;
            xTaskNotify(ble_c_handle, (uint32_t)&task_notify, eSetBits);
        }
        
    }
    DBG_E("BLE App:Start scan failed");
    return false;
}
static bool ble_c_connect_target(void)
{
    //target_addr
    //connect parameters
    conn = BLE_CONN_HANDLE_INVALID;
    memset(&conn_target,0,sizeof(connect_target_t));
    conn_target.scan_params = (ble_gap_scan_params_t *)&m_scan_params;
    conn_target.conn_params = (ble_gap_conn_params_t *)&m_connection_param;
    conn_target.conn_handle = &conn;
    //uint8_t addr[8] = {0x83,0x49,0x5D,0xCD,0xF0,0xDB};
    //memcpy(target_addr,p_addr,6);
    conn_target.addr = max_lvl_node->addr;
    DBG_I("Connecting device %02x:%02x:%02x:%02x:%02x:%02x ......\r\n",
            max_lvl_node->addr[0],
            max_lvl_node->addr[1],
            max_lvl_node->addr[2],
            max_lvl_node->addr[3],
            max_lvl_node->addr[4],
            max_lvl_node->addr[5]);
    if(ble_c->ops->connect(ble_c,&conn_target,CONN_TIMEOUT) == HAL_ERR_OK)
    {
        return true;
    }
    return false;

}
static bool ble_c_trans(task_data_t *arg)
{
    if(!opt_flag.active)
    {
        DBG_E("ble_c_rx_handler: not allow perform\r\n");
        return false;
    }
    if(!arg || !arg->p_data)
    {
        DBG_E("ble_c_trans arg or arg->p_data pointer is null\r\n");
        return false;
    }
    DBG_E("------blec receive from net(0x%x),len=%d-----:\r\n",arg,arg->length);
    ble_c->ops->transmit(ble_c,conn,arg->p_data,arg->length);
    return true;
}
static void ble_c_receive_handler(task_data_t *arg)
{
    if(!opt_flag.active)
    {
        DBG_E("ble_c_rx_handler: not allow perform\r\n");
        return ;
    }
    if(!arg)
    {
        DBG_E("ble_c_rx_handler: pointer is NULL\r\n");
        return;
    }
    pkg_prot_head_t *prot_head = (pkg_prot_head_t *)arg->p_data;
    static task_trig_t task_notify;   
    DBG_I("the cmd_type = %d\r\n",prot_head->cmd_type);

    memset(&task_notify,0,sizeof(task_notify));
    task_notify.req_type = TASK_REQUEST_UPLOAD_VIA_NET;
    task_notify.p_content = (void *)arg;
    task_notify.handle = ble_c_handle;
    //xTaskNotify(get_net_handle(), (uint32_t)&task_notify, eSetBits);
    opt_flag.up_load = 1;
    DBG_I("----blec received from slave(0x%x),len=%d---:\r\n",arg,arg->length);
}
static bool searched_result_handler(void)
{
    if(!opt_flag.active)
    {
        DBG_E("ble_c_rx_handler: not allow perform\r\n");
        return false;
    }
    //do
    //{
        scan_result_list_delete(max_lvl_node);
        max_lvl_node = get_scan_result_max_level();
        if(max_lvl_node == NULL)
        {
            //if no device searched , start next scanning
            ble_c_search_target();
            return false;
        }
        ble_c_connect_target();
    //}while(!ble_c_connect_target());//if connect failed, connect next device
    return true;
}
static void save_device(void *p_content)
{
    target_t * target = (target_t *)p_content;
    ble_data_t *adv_data = (ble_data_t *)target->data;
    if(adv_data->len < (sizeof(slv_adv_t) + 5))
    {
        DBG_W("BLE App: scan out data length invalid<%d>",adv_data->len);
        return;
    }
    slv_adv_t *adv = (slv_adv_t *)(adv_data->p_data+5);
    slv_msg_t dev;
    memcpy(dev.mac, target->peer_addr->addr,6);
    // dev.mac[0] = target->peer_addr->addr[5];
    // dev.mac[1] = target->peer_addr->addr[4];
    // dev.mac[2] = target->peer_addr->addr[3];
    // dev.mac[3] = target->peer_addr->addr[2];
    // dev.mac[4] = target->peer_addr->addr[1];
    // dev.mac[5] = target->peer_addr->addr[0];
    dev.temp = adv->temp;
    dev.rssi = target->rssi;
    dev.vol = 4230;
    dev.timestamp = hal_rtc_get_time();
    dev.mill_secs = (xTaskGetTickCount() % 1000);
    // DBG_D("BLE App: scan dev mac: %02x:%02x:%02x:%02x:%02x:%02x:",
    //             dev.mac[0],dev.mac[1],dev.mac[2],dev.mac[3],dev.mac[4],dev.mac[5]);
    // DBG_D("BLE App: scan dev temp = %d",dev.temp);
    //DBG_D("BLE App: scan dev rssi = %d",dev.rssi);

    
    send_msg_to_cache_queue(&dev);
}
static void ble_c_handle_task(void *arg)
{
    DBG_I("ble_c_handle_task startup\r\n");
    ble_c_cfg_t config;
    memset(&config,0,sizeof(ble_c_cfg_t));
    config.conn_handler = ble_c_conn_callback;
    config.rx_handler = ble_c_rx_callback;
    config.disc_handler = ble_c_disc_callback;
    ble_c = ble_c_get_instance(0);
    if(!(ble_c) || \
       !(ble_c->ops->lock(ble_c) == HAL_ERR_OK) || \
       !(ble_c->ops->init(ble_c,&config) == HAL_ERR_OK) \
       )
    {
        DBG_E("ble_c_handle_task startup failed\r\n");
        return;
    }
    DBG_I("ble_c init ok\r\n");       
    static task_trig_t *notify_value = NULL;
    TickType_t wait_tick = portMAX_DELAY;

    scan_result_list_init();
    ble_c_search_target();
    while(1)
    {
        if(xTaskNotifyWait( 0,ULONG_MAX,(uint32_t *)&notify_value,wait_tick ) == pdPASS)
        {
            switch(notify_value->req_type)
            {
                case TASK_REQUEST_BLE_C_SCAN_COMPLETE:
                    vTaskDelay(5000);
                    ble_c_search_target();
                    break;
                case TASK_REQUEST_BLE_C_SCAN_OUT_DEVICE:
                    save_device(notify_value->p_content);
                    break;
                default:
                    break;
            }
        }
    } 
}

TaskHandle_t create_ble_c_task(void)
{
    if(xTaskCreate( ble_c_handle_task, "BC\0", 256, NULL, 1, &ble_c_handle ) != pdPASS)
    {
        return NULL;
    }
    return ble_c_handle;
}

TaskHandle_t get_ble_c_task_handle(void)
{
    return ble_c_handle;
}