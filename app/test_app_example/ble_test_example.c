#include <stdint.h>
#include "app_freertos.h"
#include "nrfx_gpiote.h"
#include "ble_test_example.h"
#include "ble_init.h"
#include "hal_pin.h"
#include "ble_c_obj.h"
#include "ble_s_obj.h"
#include "app_log.h"
#include "ble_gap.h"
#include "test_uart.h"
#include "pca10056.h"

#define S_RUN_INDEX     15
#define RUN_INDEX       17
static  TaskHandle_t    ble_test_handle = NULL;
static  TaskHandle_t    ble_s_test_handle = NULL;
static ble_c_t         * ble_c = NULL;

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
void scan_handler(target_t const * p_content)
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
void ble_scan_test(void)
{
    scan_target_t scan_target = SCAN_TARGET_MSG_DEFAULT;
    //static uint8_t addr[] = {0x08,0x24,0x7A,0x1C,0x03,0xCA};
    //static uint8_t addr[] = {0x7D,0xB4,0x91,0x8F,0xEF,0xE4};
    static uint8_t addr[] = {0xFD,0xE7,0xD2,0xFC,0xDF,0xD6};
    static char name[] = "c_s\0";
    scan_target.name = name;
    scan_target.addr = addr;
    scan_target.scan_params = (ble_gap_scan_params_t *)&m_scan_params;
    //target.mode = NEED_NAME_AND_ADDR;
    scan_target.mode = NEED_ONLY_ADDR;
    //target.handler = scan_handler;        //若使用回调，则不进入柱塞
    scan_target.handler = NULL;                  //不使用回调，则进入柱塞等待成功或者超时

    connect_target_t conn_target;
    static uint16_t conn;

    conn = BLE_CONN_HANDLE_INVALID;
    memset(&conn_target,0,sizeof(connect_target_t));
    conn_target.addr = addr;
    conn_target.scan_params = (ble_gap_scan_params_t *)&m_scan_params;
    conn_target.conn_params = (ble_gap_conn_params_t *)&m_connection_param;
    conn_target.conn_handle = &conn;
    if(ble_c == NULL)
    {
        ble_c = ble_c_get_instance(0);
    }
    if(ble_c->ops->lock(ble_c) == HAL_ERR_OK)
    {
        if(ble_c->ops->scan(ble_c,&scan_target,10000) == HAL_ERR_OK)
        {
            if(ble_c->ops->connect(ble_c,&conn_target,10000) == HAL_ERR_OK)
            {
                //uart_string_send("ble connect ok\r\n\0");
                DBG_LOG("central connect slave ok\r\n\0");
                int mtu = ble_c->ops->get_mtu(ble_c,conn);
                DBG_I("the central connection mtu = %d\r\n\0",mtu);
                uint8_t buf[256] = {0};
                int32_t len = 0;
                ble_data_t data;
                data.p_data =buf;
                
                while(1)
                {
                    data.len = sizeof(buf);
                    len = 0;
                    len = ble_c->ops->receive(ble_c,conn,&data,1000);
                    if(len > 0)
                    {
                        DBG_LOG("ble rx:%s\r\n",data.p_data);
                        test_uart_trans(data.p_data,len);
                        len = 0;
                    }

                    len = test_uart_receive(buf,sizeof(buf));
                    if(len > 0)
                    {
                        DBG_LOG("uart rx length = %d\r\n",len);
                        data.len = len;
                        ble_c->ops->transmit(ble_c,conn,&data);
                        DBG_LOG("uart rx:%s\r\n",data.p_data);
                        len = 0;
                    } 
                    if(!hal_pin_read(BUTTON_1))
                    {
                        uart_string_send("disconnect ble\r\n\0");
                        break;
                    }
                }
                ble_c->ops->disconnect(ble_c,conn_target.conn_handle);
            }
        }
        ble_c->ops->unlock(ble_c);
    }
     else
     {
         DBG_E("lock failed, ble is scanning busy");
     }
}
void ble_task_handle(void *arg)
{   
    DBG_LOG("+++++++++++++++++++ble central task startup +++++++++++++++++++++");
    test_gpio_init();
    test_uart_init();
    uart_string_send("ble central start ok\r\n\0");
    hal_pin_set_mode(RUN_INDEX,HAL_PIN_MODE_OUT);
    hal_pin_set_mode(BUTTON_1,HAL_PIN_MODE_IN_PULL_UP);
    hal_pin_write(RUN_INDEX,HAL_PIN_LVL_LOW);
    //ble_scan_test();
    bool pin;
    while(1)
    {
        pin = hal_pin_read(BUTTON_1);
        if(!pin)
        {
            hal_pin_write(RUN_INDEX,HAL_PIN_LVL_LOW);
            DBG_LOG("the key is pressed");
            ble_scan_test();
        }
        
        vTaskDelay(2000);
        hal_pin_write(RUN_INDEX,HAL_PIN_LVL_HIGH);
        vTaskDelay(50);
        hal_pin_write(RUN_INDEX,HAL_PIN_LVL_LOW);
    }
}
void ble_s_task_handle(void *arg)
{
    vTaskDelay(1000);
    static ble_s_t *ble_s;
    DBG_LOG("-------------ble slave task startup ----------");
    uart_string_send("ble slave start ok\r\n\0");
    hal_pin_set_mode(S_RUN_INDEX,HAL_PIN_MODE_OUT);
    hal_pin_write(S_RUN_INDEX,HAL_PIN_LVL_LOW);
    ble_s = ble_s_get_instance();
    if(ble_s->ops->lock(ble_s) == HAL_ERR_OK)
    {
        ble_s->ops->init(ble_s,NULL);
        uint8_t buf[800] = {0};
        int32_t len = 0;
        while(1)
        {
            len = 0;
            len = ble_s->ops->receive(ble_s,buf,sizeof(buf),500);
            if(len > 0)
            {
                DBG_LOG("the slave received len = %d",len);
                test_uart_trans(buf,len);
                ble_s->ops->transmit(ble_s,buf,len);
                len = 0;
            }
            len = test_uart_receive(buf,sizeof(buf));
            if(len > 0)
            {
                DBG_LOG("the uart received len = %d",len);
                //test_uart_trans(buf,len);
                ble_s->ops->transmit(ble_s,buf,len);
                len = 0;
            }
            hal_pin_write(S_RUN_INDEX,HAL_PIN_LVL_HIGH);
            vTaskDelay(50);
            hal_pin_write(S_RUN_INDEX,HAL_PIN_LVL_LOW);
        }
    }
}
/**
 * @brief 
 * 
 */
void creat_task(void)
{
    uint32_t ret = xTaskCreate( ble_task_handle, "test", 512, NULL, 1, &ble_test_handle );
    if(pdPASS != ret)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    if(xTaskCreate( ble_s_task_handle, "test", 512, NULL, 1, &ble_s_test_handle ) != pdPASS)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }  
}