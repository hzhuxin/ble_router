/**
 * @brief 
 * 
 * @file test_uart.c
 * @author Chenfei
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.
 * All Rights Reserved.
 * @date 2018-07-12
 */
#include "test_uart.h"
#include "hal_uart.h"
#include "nrfx_uarte.h"
#include "app_log.h"

static  hal_uart_t      *uart = NULL;
//static const uint8_t start[2] = {0xEF,0xEF};
//static const uint8_t stop[3] = {0xFE,0xFE,0xFE};

/**
 * @brief 
 * 
 */
void test_uart_init(void)
{
    if(uart != NULL)
    {
        DBG_LOG("test uart has initialized.");
        return ;
    }
    hal_err_t err_code;
    hal_uart_cfg_t uart_config;

    uart_config.baudrate = HAL_UART_BAUDRATE_115200;
    uart_config.parity   = HAL_UART_PARITY_NONE;
    uart_config.rx_buf_size = UART_RX_BUF;
    uart_config.rx_mode = HAL_UART_RX_MODE_BUFFERED;
    uart_config.rx_pin = UART_RX_PIN;
    uart_config.tx_pin = UART_TX_PIN;
    uart_config.rx_timeout_ms = 100;
    uart_config.tx_buf_size = UART_TX_BUF;
    uart_config.tx_mode = HAL_UART_TX_MODE_NOCOPY;
    uart_config.tx_timeout_ms = 100;

    uart = hal_uart_get_instance(1);
    //nrfx_uarte_t inst = *(nrfx_uarte_t *)uart->inst;
    //DBG_LOG("uart->inst = %d",(*uart->inst)->drv_inst_idx);
    if(uart == NULL)
    {
        DBG_LOG("test uart hal_uart_get_instance failed.");
        return;
    }
    err_code = uart->ops->init(uart,&uart_config);
    if(err_code != HAL_ERR_OK)
    {
        DBG_LOG("test uart init failed, reason %d.",err_code);
        return;
    }
    err_code = uart->ops->set_rx_timeout(uart,UART_RX_TIMEOUT);
    if(err_code != HAL_ERR_OK)
    {
        DBG_LOG("test uart set rx timeout failed, reason %d.",err_code);
        return;
    }
    err_code = uart->ops->set_rx_enable(uart,1);
    if(err_code != HAL_ERR_OK)
    {
        DBG_LOG("test uart set_rx_enable failed, reason 0x%x.",err_code);
        return;
    }
    DBG_LOG("test uart initialized ok.");
    char str[] = "hello\r\n\0";
    int ret = uart->ops->write(uart,str,5);
    DBG_LOG("ret = 0x%x",ret);
    DBG_LOG("test uart transmit %s.",(ret == 5)?"OK":"failed");

}

/**
 * @brief  close uart current instrance
 * 
 */
void test_close_uart(void)
{
    if(uart != NULL)
    {
        uart->ops->deinit(uart);
        uart = NULL;
    }
}
/**
 * @brief 
 * 
 * @param buf 
 * @param length 
 * @return uint32_t 
 */
uint32_t test_uart_trans(void const * const buf, uint16_t length)
{
    if(uart == NULL)
    {
        return 0;
    }
    DBG_LOG("uart transmiting...");
    hal_err_t ret = uart->ops->write(uart,(uint8_t *)buf,length);
    if(ret < 0)
    {
        return 0;
    }
    DBG_LOG("transmit %d bytes data complete",length);
    return length;
}
/**
 * @brief 
 * 
 * @param buf 
 * @param max_size 
 * @return uint32_t 
 */
uint32_t test_uart_receive(void * const buf, uint16_t max_size)
{
    if(uart == NULL)
    {
        return 0;
    }
    if(buf == NULL)
    {
        DBG_LOG("test_uart_receive pointer is null.");
        return false;
    }
    uint8_t *rx_buf = buf;
    return uart->ops->read(uart,rx_buf,max_size);
}

void uart_string_send(char *str)
{
    if(uart == NULL)
    {
        return;
    }
    if(str == NULL)
    {
        return;
    }
    uint8_t buf[256];
    memcpy(buf,str,strlen(str));
    uart->ops->write(uart,buf,strlen(str));
}
//end
