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
#include "hal_uart.h"
#include "data_cache.h"

/*********************************defines***************************************/
char const NET_SET_MODE[] = "AT+WKMOD=\"NET\"\r\n\0";
//char const *NET_SET_MODE[]  =      "AT+WKMOD=\"HTTPD\"\r\n\0";
char const NET_EN_SOCKET_A[]  =   "AT+SOCKAEN=\"on\"\r\n\0";
char const NET_SET_SERVER[]   =  "AT+SOCKA=\"TCP\",\"https://api.xinaitech.com/api/5d10a932ea612\"\r\n\0";
char const NET_SAVE_CFG[]    =    "AT+S\r\n\0";

DBG_SET_LEVEL(DBG_LEVEL_D);

static TaskHandle_t net_handle = NULL;
static QueueHandle_t net_msg_queue = NULL;
static SemaphoreHandle_t    sema = NULL;
static hal_uart_t   *uart = NULL;
static xfer_t  xfer;
/**********************************functions**************************************/
int32_t uart_init(void)
{
    uart = hal_uart_get_instance(0);
    if(uart == NULL)
    {
        return -1;
    }
    const hal_uart_cfg_t cfg = {
        .baudrate = HAL_UART_BAUDRATE_115200,
        .parity = HAL_UART_PARITY_NONE,
        .tx_mode = HAL_UART_TX_MODE_NOCOPY,
        .rx_mode = HAL_UART_RX_MODE_BUFFERED,
        .rx_buf_size = 1000,
        .rx_timeout_ms = 300,
        .tx_timeout_ms = 1000,
        .tx_pin = HAL_CFG_CELL_TXD,
        .rx_pin = HAL_CFG_CELL_RXD,
    };
    if(uart->ops->init(uart, &cfg) != HAL_ERR_OK ||
       uart->ops->set_rx_enable(uart, true) != HAL_ERR_OK)
    {
        DBG_E("UART init failed\r\n");
        return -2;
    }
    return 0;
}
int32_t uart_write(void *buf,int32_t len)
{
    if(!buf)
    {
        DBG_E("UART write: buf is null");
        return -1;
    }
    return uart->ops->write(uart,buf,len);
}
int32_t uart_read(uint8_t *buf, int32_t size)
{
    if(!buf)
    {
        DBG_E("UART read: buf is null");
        return -1;
    }
    return uart->ops->read(uart,buf,size);
}
int32_t net_dev_init(void)
{
    if(uart_init() != 0)
    {
        return -1;
    }
    uart_write((void *)NET_SET_MODE,strlen(NET_SET_MODE));
    uart_write((void *)NET_EN_SOCKET_A,strlen(NET_EN_SOCKET_A));
    uart_write((void *)NET_SET_SERVER,strlen(NET_SET_SERVER));
    uart_write((void *)NET_SAVE_CFG,strlen(NET_SAVE_CFG));

    DBG_I("Net device init over");
    return 0;
}

int32_t net_wait_res(uint8_t *buf, int32_t buf_size)
{
    int32_t time = 10;
    int32_t len = 0;
    //while(!util_timeout(start, xTaskGetTickCount(), 10000))
    while(time-- > 0)
    {
        DBG_D("Net waiting response... %d S\r\n",time);
        len = uart_read(buf,buf_size);
        if(len > 0)
        {
            DBG_D("Net received response len = %d\r\n",len);
            return len ;
        }
        vTaskDelay(1000);
    }
    return -1;
}
int32_t net_upload(uint8_t *p_data, int32_t len)
{
    if(!p_data)
    {
        DBG_E("net upload pointer is NULL\r\n");
        return -1;
    }
    if(uart_write(p_data,len) != len)
    {
        DBG_E("net upload failed\r\n");
    }
    xfer.len = net_wait_res(xfer.data,xfer.max_size);
    if(xfer.len > 0 && net_msg_queue)
    {
        //xQueueSendToBack(net_msg_queue, xfer, 1000);
        //xTaskNotify(get_ble_c_task_handle(), (uint32_t)&task_notify, eSetBits);
        xSemaphoreGive(sema);
        return xfer.len;
    }
    return -2;
}
int32_t net_recv(void)
{
    xfer.len = uart_read(xfer.data,xfer.max_size);
    if(xfer.len > 0)
    {
        DBG_D("Net received <%d bytes>message\r\n",xfer.len);
        //xQueueSendToBack(net_msg_queue, xfer, 1000);
        xSemaphoreGive(sema);
        return xfer.len ;
    }
    return -1;
}
void net_handle_task(void *arg)
{
    net_msg_queue = xQueueCreate(5,sizeof(xfer_t));
    //xfer = (msg_t *)pvPortMalloc(xfer.max_size+sizeof(xfer.len));
    if(!net_msg_queue)
    {
        return;
    }
    while(1)
    {
        if(xQueueReceive(net_msg_queue,&xfer,portMAX_DELAY) == pdPASS)
        {
            net_upload(xfer.data,xfer.len);
        }
        //net_recv();

        // DBG_D("net manage waiting notify, wait_tick = %d\r\n",wait_tick);
        // if(xTaskNotifyWait( 0,ULONG_MAX,(uint32_t *)&notify_value,wait_tick ) == pdPASS)
        // {
        //     //send data
        //     //upload(notify_value->p_content);
        // }
         
    }
}
void create_net_manage_task(void)
{ 
    if(xTaskCreate( net_handle_task, "net", 512, NULL, 1, &net_handle ) != pdPASS)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}
void send_msg_to_net_queue(uint32_t *msg)
{
    xQueueSendToBack(net_msg_queue, msg, 1000);
}
bool send_msg_to_server(void *msg)
{
    xQueueSendToBack(net_msg_queue, (xfer_t *)msg, 1000);
    if(xSemaphoreTake(sema,pdMS_TO_TICKS(10000)) != pdTRUE)
    {
        return -1;
    }
    return 0;
}
TaskHandle_t *get_net_handle(void)
{
    return net_handle;
}
//end
