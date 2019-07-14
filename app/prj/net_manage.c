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
#include "hal_pin.h"
#include "data_cache.h"
#include "data_mgt.h"

/*********************************defines***************************************/
#define B_NET_INDEX     NRF_GPIO_PIN_MAP(1,9)

DBG_SET_LEVEL(DBG_LEVEL_E);

char const NET_SET_MODE[] = "AT+WKMOD=\"NET\"\r\n\0";
//char const *NET_SET_MODE[]  =      "AT+WKMOD=\"HTTPD\"\r\n\0";
char const NET_EN_SOCKET_A[]  =   "AT+SOCKAEN=\"on\"\r\n\0";
char const NET_SET_SERVER[]   =  "AT+SOCKA=\"TCP\",\"https://api.xinaitech.com/api/5d10a932ea612\"\r\n\0";
char const NET_SAVE_CFG[]    =    "AT+S\r\n\0";
char const NET_RES_SUCCESS[] = "{\"code\":1,\"msg\":\"Success\",\"data\":{\"timestamp\":\0";
static TaskHandle_t net_handle = NULL;
static QueueHandle_t net_msg_queue = NULL;
static SemaphoreHandle_t    sema = NULL;
static hal_uart_t   *uart = NULL;
static msg_t  msg_buf;
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
    DBG_E("UART init ok\r\n");
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
int32_t net_string_to_dec(char *str,char endc)
{
    int32_t timestamp = 0;
    while(*str != endc)
    {
        timestamp = timestamp * 10 + (*str - '0');
        str++;
    }
    return timestamp;
}
int32_t net_parse_response_msg(void *msg, int32_t len)
{
    if(!msg || len <= 0)
    {
        DBG_E("Net parse msg=NULL, ort len<= 0");
        return -1;
    }
    char *res;
    res = strstr((char *)msg, NET_RES_SUCCESS);

    if(res)
    {
        int32_t timestamp, rtc;
        timestamp = net_string_to_dec(res+strlen(NET_RES_SUCCESS),'}');
        rtc = hal_rtc_get_time();
        DBG_I("Net: response success, timestamp = %d, and RTC = %d",timestamp,rtc);
        if(abs(timestamp - rtc) > 3 && timestamp > 1562422578)
        {
            hal_rtc_set_time(timestamp);
        }
        return 0;
    }
    return -2;
}
int32_t net_wait_res(uint8_t *buf, int32_t buf_size)
{
    int32_t time = 10;
    int32_t len = 0;
    //while(!util_timeout(start, xTaskGetTickCount(), 10000))
    DBG_I("Net waiting response... 10S\r\n");
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
int32_t net_upload(void *p_data, int32_t len)
{
    if(!p_data)
    {
        DBG_E("net upload pointer is NULL\r\n");
        return -1;
    }
    DBG_I("Net upload: %s\r\n",(char *)p_data);
    int ret = uart_write(p_data,len);
    if(ret != len)
    {
        DBG_E("net upload failed, return %d(%d is ok)\r\n",ret,len);
        return -2;
    }
    len = net_wait_res(p_data,NET_QUEUE_DEFAULT_LENGTH);
    if(len > 0)
    {
        if(net_parse_response_msg(p_data,len) >=0)
        {
            //xQueueSendToBack(net_msg_queue, p_data, 1000);
            //xTaskNotify(get_ble_c_task_handle(), (uint32_t)&task_notify, eSetBits);
            xSemaphoreGive(sema);
            return len;
        }
    }
    else
    {
        DBG_W("Net: Not received server response");
    }
    
    return -2;
}
int32_t net_recv(void)
{
    msg_buf.len = uart_read(msg_buf.data,NET_QUEUE_DEFAULT_LENGTH);
    if(msg_buf.len > 0)
    {
        DBG_I("Net received <%d bytes>message\r\n",msg_buf.len);
        //xQueueSendToBack(net_msg_queue, xfer, 1000);
        xSemaphoreGive(sema);
        return msg_buf.len ;
    }
    return -1;
}
void net_handle_task(void *arg)
{
    DBG_I("Net task startup.");
    net_msg_queue = xQueueCreate(2,NET_QUEUE_DEFAULT_LENGTH);
    if(!net_msg_queue)
    {
        DBG_E("Net: Create queue failed");
        return;
    }
    sema = xSemaphoreCreateBinary();
    if(!sema)
    {
        DBG_E("Net: Create semaphore failed");
        return;
    }
    xSemaphoreTake(sema,pdMS_TO_TICKS(10));
    net_dev_init();
    uint32_t cnt=0;
    // uint8_t msg[NET_QUEUE_DEFAULT_LENGTH];
    // msg_t *p_msg = (msg_t *)msg;
    hal_pin_set_mode(B_NET_INDEX,HAL_PIN_MODE_OUT);
    hal_pin_write(B_NET_INDEX,HAL_PIN_LVL_HIGH);

    while(1)
    {
        memset(&msg_buf,0,sizeof(msg_buf));
        //if(xQueueReceive(net_msg_queue,&xfer,portMAX_DELAY) == pdPASS)
        if(xQueueReceive(net_msg_queue, &msg_buf, pdMS_TO_TICKS(2000)) == pdPASS)
        {
            //DBG_D("Net: p_msg->len=%d",p_msg->len);
            //DBG_D("Net: recv: %s",p_msg->data);
            hal_pin_write(B_NET_INDEX,HAL_PIN_LVL_LOW);
            net_upload(&msg_buf.data,msg_buf.len);
            hal_pin_write(B_NET_INDEX,HAL_PIN_LVL_HIGH);
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
void send_msg_to_net_queue(uint32_t *msg)
{
    xQueueSendToBack(net_msg_queue, msg, 1000);
}
bool send_msg_to_server(void *msg)
{
    if(!net_msg_queue || !sema)
    {
        DBG_E("Net: net_msg_queue || sema is NULL, func:%s,line: %d",__func__,__LINE__);
        return false;
    }
    xQueueSendToBack(net_msg_queue, msg, 1000);
    if(xSemaphoreTake(sema,pdMS_TO_TICKS(10000)) != pdTRUE)
    //if(xQueueReceive(net_msg_queue, msg, pdMS_TO_TICKS(10000)) == pdPASS)
    {
        return false;
    }
    return true;
}
TaskHandle_t *get_net_handle(void)
{
    return net_handle;
}
//end
