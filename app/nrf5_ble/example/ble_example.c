#include <stdint.h>
#include "app_freertos.h"
#include "nrfx_gpiote.h"
#include "ble_example.h"
#include "ble_init.h"
#include "hal_pin.h"
#include "ble_c_obj.h"
#include "ble_s_obj.h"
#include "app_log.h"
#include "ble_gap.h"

#define S_RUN_INDEX     19
#define RUN_INDEX       20
static  TaskHandle_t    ble_test_handle = NULL;
static ble_c_t         * ble_c = NULL;
static ble_s_t         * ble_s = NULL;
static uint16_t         c_conn_handle = BLE_CONN_HANDLE_INVALID;
//static uint16_t         s_conn_handle = BLE_CONN_HANDLE_INVALID;

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
void test_gpio_init(void)
{
    if(nrfx_gpiote_init() != NRFX_SUCCESS)
    {
        DBG_LOG("test gpio initialized failed.");
        return;
    }
    DBG_LOG("test gpio initialized ok.");
}
void ble_c_scan_callback(target_t const * p_content)
{
    DBG_LOG("scan complete");
    if(p_content != NULL)
    {
        DBG_LOG("target name: %s",p_content->name);
        DBG_LOG("target addr: %02x:%02x:%02x:%02x:%02x:%02x\r\n",
                             p_content->peer_addr->addr[0],
                             p_content->peer_addr->addr[1],
                             p_content->peer_addr->addr[2],
                             p_content->peer_addr->addr[3],
                             p_content->peer_addr->addr[4],
                             p_content->peer_addr->addr[5]
                             );
    }
    ble_c->ops->scan_stop(ble_c);
}
void ble_c_conn_callback(void *arg)
{
    //connected slave device
}
void ble_c_disc_callback(void *arg)
{
    //disconnected
}
void ble_c_rx_callback(ble_data_t * p_ble_data)
{
    //received data, has header of ble protocol
}

void ble_c_test(void)
{
    if(ble_c == NULL)
    {
        ble_c = ble_c_get_instance(0);
    }
    if(!ble_c)
    {
        return;
    }
    //initialize ble central
    ble_c_cfg_t config;
    memset(&config,0,sizeof(ble_c_cfg_t));
    //回调函数配置，若使用回调，则连接时调用回调函数，若不使用回调则进入柱塞直到成功或者超时
    config.conn_handler = ble_c_conn_callback;  
    config.rx_handler = ble_c_rx_callback;
    config.disc_handler = ble_c_disc_callback;
    if(!(ble_c) || \
       !(ble_c->ops->lock(ble_c) == HAL_ERR_OK) || \
       !(ble_c->ops->init(ble_c,&config) == HAL_ERR_OK) \
       )
    {
        return;
    }
    //scan target config
    scan_target_t target = SCAN_TARGET_MSG_DEFAULT;
    static uint8_t addr[] = {0x83,0x49,0x5D,0xCD,0xF0,0xDB};
    static char name[] = "c_s\0";
    target.name = name;
    target.addr = addr;
    //target.mode = NEED_NAME_AND_ADDR;
    target.mode = NEED_ONLY_ADDR;
    //target.handler = ble_c_scan_callback;        //若使用回调，则不进入柱塞
    target.handler = NULL;                  //不使用回调，则进入柱塞等待成功或者超时

    connect_target_t conn_target;
    memset(&conn_target,0,sizeof(connect_target_t));
    conn_target.addr = addr;
    conn_target.scan_params = (ble_gap_scan_params_t *)&m_scan_params;
    conn_target.conn_params = (ble_gap_conn_params_t *)&m_connection_param;
    conn_target.conn_handle = &c_conn_handle;
    
    //若使用了回调函数，则这里不会进入柱塞状态而直接返回
    if(ble_c->ops->scan(ble_c,&target,5000) != HAL_ERR_OK) 
    {
        return;
    }
    if(ble_c->ops->connect(ble_c,&conn_target,5000) != HAL_ERR_OK)
    {
        return;
    }
    uint8_t buf[BLE_RX_BUF_SIZE] = {0};
    int32_t len = 0;
    while(1)
    {
        len = 0;
        len = ble_c->ops->receive(ble_c,c_conn_handle,buf,sizeof(buf),1000);
        if(len > 0)
        {
            //DBG_LOG("ble rx:%s\r\n",data.p_data);
            ble_c->ops->transmit(ble_c,c_conn_handle,buf,len);
            len = 0;
        }
        if(!hal_pin_read(13))
        {
            break;
        }
    }
    ble_c->ops->disconnect(ble_c,&c_conn_handle);
    ble_c->ops->deinit(ble_c);
    ble_c->ops->unlock(ble_c);
}
void ble_s_conn_callback(void *arg)
{
    //connected
}
void ble_s_disc_callback(void *arg)
{
    //disconnected
}
void ble_s_rx_callback(ble_data_t * p_ble_data)
{
    //received data, it has header of ble protocol
}
void ble_s_start_callback(void *arg)
{
    //notified
}
void ble_s_stop_callback(void *arg)
{
    //stopped
}
void ble_s_test(void)
{
    ble_s = ble_s_get_instance();
    //initialize ble central
    ble_s_cfg_t config;
    memset(&config,0,sizeof(ble_s_cfg_t));
    //回调函数配置，若使用回调，则连接时调用回调函数，若不使用回调则进入柱塞直到成功或者超时
    //config.conn_handler = ble_s_conn_callback;  
    //config.rx_handler = ble_s_rx_callback;
    //config.disc_handler = ble_s_disc_callback;
    config.notify_handler = ble_s_start_callback;
    config.unnotify_handler = ble_s_stop_callback;

    if(!(ble_s->ops->lock(ble_s) != HAL_ERR_OK) && \
       !(ble_s->ops->init(ble_s,&config) != HAL_ERR_OK))
    {
        return;
    }
    uint8_t buf[BLE_RX_BUF_SIZE] = {0};
    int32_t len = 0;
    while(1)
    {
        len = 0;
        //如果用了回调函数，则可以在回调函数里面接收数据
        len = ble_s->ops->receive(ble_s,buf,sizeof(buf),1000);
        if(len > 0)
        {
            //DBG_LOG("ble rx:%s\r\n",data.p_data);
            ble_s->ops->transmit(ble_s,buf,len);
            len = 0;
        }
        if(!hal_pin_read(14))
        {
            break;
        }
    }
    ble_s->ops->deinit(ble_s);
    ble_s->ops->unlock(ble_s);
}
void ble_task_handle(void *arg)
{   
    DBG_LOG("+++++++++++++++++++ble central task startup +++++++++++++++++++++");
    test_gpio_init();
    hal_pin_set_mode(RUN_INDEX,HAL_PIN_MODE_OUT);
    hal_pin_set_mode(13,HAL_PIN_MODE_IN_PULL_UP);
    hal_pin_set_mode(14,HAL_PIN_MODE_IN_PULL_UP);
    hal_pin_write(RUN_INDEX,HAL_PIN_LVL_HIGH);
    bool pin;
    while(1)
    {
        pin = hal_pin_read(13);
        if(!pin)
        {
            hal_pin_write(RUN_INDEX,HAL_PIN_LVL_LOW);
            DBG_LOG("the key is pressed");
            ble_c_test();
        }
        pin = hal_pin_read(14);
        if(!pin)
        {
            hal_pin_write(RUN_INDEX,HAL_PIN_LVL_LOW);
            DBG_LOG("the key is pressed");
            ble_s_test();
        }
        vTaskDelay(2000);
        hal_pin_write(RUN_INDEX,HAL_PIN_LVL_LOW);
        vTaskDelay(50);
        hal_pin_write(RUN_INDEX,HAL_PIN_LVL_HIGH);
    }
}

/**
 * @brief 
 * 
 */
void creat_example_task(void)
{
    uint32_t ret = xTaskCreate( ble_task_handle, "test", 1024, NULL, 1, &ble_test_handle );
    if(pdPASS != ret)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}