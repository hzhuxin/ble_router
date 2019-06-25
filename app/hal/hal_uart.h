/**
 * @brief The uart abstract layer
 * 
 * @file uart.h
 * @date 2018-06-22
 * @author Dengjian
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

#ifndef __HAL_UART_H__
#define __HAL_UART_H__

/* Includes ------------------------------------------------------------------*/
#include "hal.h"
#include "hal_pin.h"

/* Typedefs ------------------------------------------------------------------*/

typedef enum hal_uart_baudrate {
  HAL_UART_BAUDRATE_1200 = 0,
  HAL_UART_BAUDRATE_2400,
  HAL_UART_BAUDRATE_4800,
  HAL_UART_BAUDRATE_9600,
  HAL_UART_BAUDRATE_19200,
  HAL_UART_BAUDRATE_38400,
  HAL_UART_BAUDRATE_57600,
  HAL_UART_BAUDRATE_115200,
  HAL_UART_BAUDRATE_MAX,
} hal_uart_baudrate_t;

typedef enum hal_uart_parity {
  HAL_UART_PARITY_NONE = 0,
  HAL_UART_PARITY_EVEN,
  HAL_UART_PARITY_ODD,
  HAL_UART_PARITY_MAX,
} hal_uart_parity_t;

typedef enum hal_uart_tx_mode {
  HAL_UART_TX_MODE_DISABLED = 0,
  HAL_UART_TX_MODE_BUFFERED,
  HAL_UART_TX_MODE_NOCOPY,
  HAL_UART_TX_MODE_MAX,
} hal_uart_tx_mode_t;

typedef enum hal_uart_rx_mode {
  HAL_UART_RX_MODE_DISABLED = 0,
  HAL_UART_RX_MODE_BUFFERED,
  HAL_UART_RX_MODE_NOCOPY,
  HAL_UART_RX_MODE_MAX,
} hal_uart_rx_mode_t;

typedef struct hal_uart_cfg {
  hal_uart_baudrate_t baudrate;
  hal_uart_parity_t parity;
  hal_uart_tx_mode_t tx_mode;
  hal_uart_rx_mode_t rx_mode;
  int16_t tx_timeout_ms;
  int16_t rx_timeout_ms;
  int16_t tx_buf_size;
  int16_t rx_buf_size;
  hal_pin_t tx_pin;
  hal_pin_t rx_pin;
} hal_uart_cfg_t;

struct hal_uart_ops;
typedef struct hal_uart {
  void* inst;
  void* lock;
  void* priv;
  const struct hal_uart_ops* ops;
} hal_uart_t;

typedef struct hal_uart_ops {
  hal_err_t (*init) (hal_uart_t* obj, const hal_uart_cfg_t* cfg);
  hal_err_t (*deinit) (hal_uart_t* obj);

  hal_err_t (*lock) (hal_uart_t* obj);
  hal_err_t (*unlock) (hal_uart_t* obj);

  hal_err_t (*write) (hal_uart_t* obj, void* buf, int len);
  hal_err_t (*read) (hal_uart_t* obj, void* buf, int len);
  hal_err_t (*read_line) (hal_uart_t* obj, void* buf, int buf_len);

  hal_err_t (*set_rx_timeout) (hal_uart_t* obj, uint16_t timeout_ms);
  hal_err_t (*set_rx_enable) (hal_uart_t* obj, bool en);
  hal_err_t (*get_rx_count) (hal_uart_t* obj);
  hal_err_t (*clr_rx_data) (hal_uart_t* obj);
} hal_uart_ops_t;

/* Global variables ----------------------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
hal_uart_t* hal_uart_get_instance(int id);

#endif // #ifndef __HAL_UART_H__

