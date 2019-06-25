#ifndef TEST_UART_H
#define TEST_UART_H

#include <stdint.h>
//#include <stdbool.h>

// #define UART_RX_PIN     8
// #define UART_TX_PIN     6
#define UART_RX_PIN     27
#define UART_TX_PIN     26

#define UART_RX_BUF     128
#define UART_TX_BUF     128

#define UART_RX_TIMEOUT 50     //mS

void test_uart_init(void);
void test_close_uart(void);
uint32_t test_uart_trans(void const * const buf, uint16_t length);
uint32_t test_uart_receive(void * const buf, uint16_t max_size);
void uart_string_send(char *str);
#endif //TEST_UART_H
