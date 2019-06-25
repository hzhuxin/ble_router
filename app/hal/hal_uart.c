/**
 * @brief The uart abstract layer
 *
 * @file uart.c
 * @date 2018-06-22
 * @author Dengjian
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "nrfx_uarte.h"
#include "nrfx_log.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "buffer.h"
#include "hal_uart.h"
#include "util.h"

/* Defines -------------------------------------------------------------------*/
#if defined(NRF52840_XXAA) && UART1_ENABLED // check version and UART1
#define HAL_UART1_ENABLED
#endif

#define MAX_XFER_SIZE         255

#define LOCK_TIMEOUT          pdMS_TO_TICKS(2000)

#define CHECK_OBJ(obj)                  \
  if(!obj) {                            \
    return HAL_ERR_PARAM;               \
  }                                     \

#define CHECK_OBJ_PRIV(obj)             \
  if(!obj) {                            \
    return HAL_ERR_PARAM;               \
  }                                     \
  if(!obj->priv) {                      \
    return HAL_ERR_NOT_INITIALIZED;     \
  }                                     \

#define CHECK_OBJ_LOCK(obj)             \
  if(!obj) {                            \
    return HAL_ERR_PARAM;               \
  }                                     \
  if(!obj->lock) {                      \
    return HAL_ERR_MEMORY;              \
  }                                     \

/* Typedefs ------------------------------------------------------------------*/
typedef struct priv_data {
  StaticSemaphore_t tx_semphr_memory;
  StaticSemaphore_t rx_semphr_memory;
  SemaphoreHandle_t tx_semphr;
  SemaphoreHandle_t rx_semphr;
  hal_uart_tx_mode_t tx_mode;
  hal_uart_rx_mode_t rx_mode;
  uint8_t*  tx_buf;
  buffer_t* rx_buf;
  uint16_t tx_timeout;
  uint16_t rx_timeout;
  int16_t tx_bytes;
  int16_t rx_bytes;
  uint8_t rx_buffer[2];
} priv_data_t;

/* Private variables ---------------------------------------------------------*/

static StaticSemaphore_t lock_tab[] = {
  {
  },
#ifdef HAL_UART1_ENABLED
  {
  },
#endif
};

static nrfx_uarte_t inst_tab[] = {
  NRFX_UARTE_INSTANCE(0),
#ifdef HAL_UART1_ENABLED
  NRFX_UARTE_INSTANCE(1),
#endif
};

static hal_uart_t obj_tab[] = {
  {
    .inst = &inst_tab[0],
    .lock = NULL,
    .priv = NULL,
    .ops = NULL,
  },
#ifdef HAL_UART1_ENABLED
  {
    .inst = &inst_tab[1],
    .lock = NULL,
    .priv = NULL,
    .ops = NULL,
  },
#endif
};

/* Private functions ---------------------------------------------------------*/

static nrf_uarte_parity_t get_parity(hal_uart_parity_t parity) {
  nrf_uarte_parity_t ret = -1;

  switch(parity) {
    case HAL_UART_PARITY_NONE:
      ret = NRF_UARTE_PARITY_EXCLUDED;
      break;
    case HAL_UART_PARITY_EVEN:
      ret = NRF_UARTE_PARITY_INCLUDED;
      break;
    default:
      break;
  }

  return ret;
}

static nrf_uarte_baudrate_t get_baudrate(hal_uart_baudrate_t baudrate) {
  nrf_uarte_baudrate_t ret = -1;

  switch(baudrate) {
    case HAL_UART_BAUDRATE_1200:
      ret = NRF_UARTE_BAUDRATE_1200;
      break;
    case HAL_UART_BAUDRATE_2400:
      ret = NRF_UARTE_BAUDRATE_2400;
      break;
    case HAL_UART_BAUDRATE_4800:
      ret = NRF_UARTE_BAUDRATE_4800;
      break;
    case HAL_UART_BAUDRATE_9600:
      ret = NRF_UARTE_BAUDRATE_9600;
      break;
    case HAL_UART_BAUDRATE_19200:
      ret = NRF_UARTE_BAUDRATE_19200;
      break;
    case HAL_UART_BAUDRATE_38400:
      ret = NRF_UARTE_BAUDRATE_38400;
      break;
    case HAL_UART_BAUDRATE_57600:
      ret = NRF_UARTE_BAUDRATE_57600;
      break;
    case HAL_UART_BAUDRATE_115200:
      ret = NRF_UARTE_BAUDRATE_115200;
      break;
    default:
      break;
  }

  return ret;
}

static void uart_handler(nrfx_uarte_event_t const *p_event, void * p_context) {
  hal_uart_t* obj = (hal_uart_t*) p_context;

  if(!obj || !obj->priv) {
      return;
  }

  priv_data_t* priv = (priv_data_t*) obj->priv;
  switch(p_event->type) {
    case NRFX_UARTE_EVT_TX_DONE:
      if(priv->tx_mode == HAL_UART_TX_MODE_NOCOPY) {
        priv->tx_bytes -= p_event->data.rxtx.bytes;
        priv->tx_buf += p_event->data.rxtx.bytes;
        if(priv->tx_bytes > 0) {
          if(nrfx_uarte_tx(obj->inst, priv->tx_buf, priv->tx_bytes > MAX_XFER_SIZE ? MAX_XFER_SIZE : priv->tx_bytes)) {
          }
        } else if(priv->tx_semphr) {
          xSemaphoreGiveFromISR(priv->tx_semphr, NULL);
        }
      }
      break;

    case NRFX_UARTE_EVT_RX_DONE:
      if(priv->rx_mode == HAL_UART_RX_MODE_NOCOPY) {
        if(priv->rx_semphr) {
          priv->rx_bytes = p_event->data.rxtx.bytes;
          xSemaphoreGiveFromISR(priv->rx_semphr, NULL);
        }
      } else if(priv->rx_mode == HAL_UART_RX_MODE_BUFFERED) {
        if(priv->rx_semphr && priv->rx_buf && priv->rx_bytes) {
          uint8_t* data = p_event->data.rxtx.p_data;
          int len = p_event->data.rxtx.bytes;
          // data[len] = 0;
          // NRF_LOG_WARNING("+++++ %s (%d)", data, len);
          if(priv->rx_buf->capacity - priv->rx_buf->count < len) {
            priv->rx_buf->ops->remove(priv->rx_buf, len - (priv->rx_buf->capacity - priv->rx_buf->count));
            NRF_LOG_WARNING("UARTE_ERR_RX_BUFFER");
          }
          if(priv->rx_buf->ops->write(priv->rx_buf, data, len) != len) {
            NRF_LOG_WARNING("UARTE_ERR_RX_WRITE");
          }
          if(len) {
            xSemaphoreGiveFromISR(priv->rx_semphr, NULL);
          }
        }
      }
      break;

    case NRFX_UARTE_EVT_RXSTARTED:
      if(priv->rx_mode == HAL_UART_RX_MODE_BUFFERED && priv->rx_bytes) {
        nrfx_uarte_rx(obj->inst, p_event->data.rxtx.p_data == priv->rx_buffer ? priv->rx_buffer + 1 : priv->rx_buffer, 1);
      }
      break;

    case NRFX_UARTE_EVT_ERROR:
      NRF_LOG_WARNING("UARTE_EVT_ERROR (0x%x)", p_event->data.error.error_mask);
      // if(priv->rx_mode == HAL_UART_RX_MODE_BUFFERED && priv->rx_bytes) {
      //   nrfx_uarte_rx(obj->inst, priv->rx_buffer, 1);
      // }
      break;

    default:
      NRF_LOG_WARNING("UARTE_EVT_UNKNOWN");
      break;
  }
}

/**
 * @brief Init uart
 *
 * @return HAL_ERR_OK if success
 */
static hal_err_t init (hal_uart_t* obj, const hal_uart_cfg_t* cfg) {
  CHECK_OBJ(obj);

  if(obj->priv) {
    return HAL_ERR_ALREADY_INITIALIZED;
  }
  if(cfg->rx_mode == HAL_UART_RX_MODE_BUFFERED && cfg->rx_buf_size <= 0) {
      return HAL_ERR_PARAM;
  }

  nrf_uarte_parity_t parity = get_parity(cfg->parity);
  nrf_uarte_baudrate_t baudrate = get_baudrate(cfg->baudrate);
  if(parity == -1 || baudrate == -1) {
    return HAL_ERR_PARAM;
  }

  obj->priv = pvPortMalloc(sizeof(priv_data_t));
  if(!obj->priv) {
    NRF_LOG_WARNING("Run out of memory at (%s - %s - %d), heap size: %d", __FILE__, __func__, __LINE__, xPortGetFreeHeapSize());
    return HAL_ERR_MEMORY;
  }

  priv_data_t* priv = (priv_data_t*) obj->priv;
  if(cfg->rx_mode == HAL_UART_RX_MODE_BUFFERED) {
    priv->rx_buf = buffer_new(sizeof(char), cfg->rx_buf_size);
    if(!priv->rx_buf) {
      vPortFree(obj->priv);
      obj->priv = NULL;
      NRF_LOG_WARNING("Run out of memory at (%s - %s - %d), heap size: %d", __FILE__, __func__, __LINE__, xPortGetFreeHeapSize());
      return HAL_ERR_MEMORY;
    }
  }

  priv->tx_semphr = xSemaphoreCreateBinaryStatic(&priv->tx_semphr_memory);
  priv->rx_semphr = xSemaphoreCreateBinaryStatic(&priv->rx_semphr_memory);

  priv->tx_mode = cfg->tx_mode;
  priv->rx_mode = cfg->rx_mode;

  priv->rx_bytes = 0;
  priv->tx_bytes = 0;

  priv->tx_timeout = pdMS_TO_TICKS(cfg->tx_timeout_ms);
  priv->rx_timeout = pdMS_TO_TICKS(cfg->rx_timeout_ms);

  nrfx_uarte_config_t nrf_cfg = NRFX_UARTE_DEFAULT_CONFIG;
  nrf_cfg.pseltxd = cfg->tx_pin;
  nrf_cfg.pselrxd = cfg->rx_pin;
  nrf_cfg.p_context = obj;
  nrf_cfg.hwfc = NRF_UARTE_HWFC_DISABLED;
  nrf_cfg.parity = parity;
  nrf_cfg.baudrate = baudrate;
  nrf_cfg.interrupt_priority = APP_IRQ_PRIORITY_HIGH;

  if(nrfx_uarte_init(obj->inst, &nrf_cfg, uart_handler) != NRF_SUCCESS) {
    if(cfg->rx_mode == HAL_UART_RX_MODE_BUFFERED) {
      buffer_delete(priv->rx_buf);
    }
    vPortFree(obj->priv);
    obj->priv = NULL;
    return HAL_ERR_UNLIKELY;
  }

  return HAL_ERR_OK;
}

/**
 * @brief Deinit uart
 *
 * @return HAL_ERR_OK if success
 */
static hal_err_t deinit (hal_uart_t* obj) {
  CHECK_OBJ_PRIV(obj);

  priv_data_t* priv = (priv_data_t*) obj->priv;
  nrfx_uarte_uninit(obj->inst);

  // Fix power consumption according to https://devzone.nordicsemi.com/f/nordic-q-a/26030/how-to-reach-nrf52840-uarte-current-supply-specification/102605#102605
  *(volatile uint32_t *)((uint32_t)((nrfx_uarte_t*)obj->inst)->p_reg + 0xFFC) = 0;
  *(volatile uint32_t *)((uint32_t)((nrfx_uarte_t*)obj->inst)->p_reg + 0xFFC);
  *(volatile uint32_t *)((uint32_t)((nrfx_uarte_t*)obj->inst)->p_reg + 0xFFC) = 1;

  if(priv->rx_mode == HAL_UART_RX_MODE_BUFFERED) {
    if(priv->rx_bytes) {
      priv->rx_bytes = 0;
    }
    buffer_delete(priv->rx_buf);
  }
  vPortFree(obj->priv);
  obj->priv = NULL;

  return HAL_ERR_OK;
}

/**
 * @brief lock
 *
 * @param uart
 *
 * @return HAL_ERR_OK if success
 */
static hal_err_t lock(hal_uart_t* obj) {
  CHECK_OBJ_LOCK(obj);

  if(!xSemaphoreTake(obj->lock, LOCK_TIMEOUT)) {
    return HAL_ERR_BUSY;
  }

  return HAL_ERR_OK;
}

/**
 * @brief Unlock the instance
 *
 * @param uart
 *
 * @return HAL_ERR_OK if success
 */
static hal_err_t unlock(hal_uart_t* obj) {
  CHECK_OBJ_LOCK(obj);

  if(!xSemaphoreGive(obj->lock)) {
    return HAL_ERR_UNLIKELY;
  }

  return HAL_ERR_OK;
}

/**
 * @brief Write bytes
 *
 * @param uart
 * @param buf
 * @param len
 *
 * @return Number of bytes written
 */
static hal_err_t write(hal_uart_t* obj, void* buf, int len) {
  CHECK_OBJ_PRIV(obj);

  if(!buf || len <= 0) {
    return HAL_ERR_PARAM;
  }

  priv_data_t* priv = (priv_data_t*) obj->priv;
  if(!priv->tx_semphr) {
    return HAL_ERR_MEMORY;
  }

  if(priv->tx_mode == HAL_UART_TX_MODE_NOCOPY) {
    priv->tx_buf = buf;
    priv->tx_bytes = len;
    xSemaphoreTake(priv->tx_semphr, 0);

    if(nrfx_uarte_tx(obj->inst, buf, len > MAX_XFER_SIZE ? MAX_XFER_SIZE : len)) {
      return HAL_ERR_DENIAL;
    }

    if(!xSemaphoreTake(priv->tx_semphr, priv->tx_timeout)) {
      priv->tx_bytes = 0;
      nrfx_uarte_tx_abort(obj->inst);
      return HAL_ERR_TIMEOUT;
    }
  } else {
    return HAL_ERR_NOT_SUPPORT;
  }

  return len - priv->tx_bytes;
}

/**
 * @brief Read bytes
 *
 * @param uart
 * @param buf
 * @param len
 *
 * @return Number of bytes read
 */
static hal_err_t read(hal_uart_t* obj, void* buf, int len) {
  CHECK_OBJ_PRIV(obj);

  if(!buf || len <= 0) {
    return HAL_ERR_PARAM;
  }

  priv_data_t* priv = (priv_data_t*) obj->priv;
  if(!priv->rx_semphr) {
    return HAL_ERR_MEMORY;
  }

  if(priv->rx_mode == HAL_UART_RX_MODE_NOCOPY) {
    priv->rx_bytes = 0;
    xSemaphoreTake(priv->rx_semphr, 0);

    if(nrfx_uarte_rx(obj->inst, buf, len)) {
      return HAL_ERR_DENIAL;
    }

    if(!xSemaphoreTake(priv->rx_semphr, priv->rx_timeout)) {
      nrfx_uarte_rx_abort(obj->inst);
      return HAL_ERR_TIMEOUT;
    }
    return priv->rx_bytes;
  } else if(priv->rx_mode == HAL_UART_RX_MODE_BUFFERED) {
    TickType_t start = xTaskGetTickCount();
    TickType_t end = start + priv->rx_timeout;
    do {
      if(priv->rx_buf->count >= len) {
        break;
      }

      TickType_t now = xTaskGetTickCount();
      if(!xSemaphoreTake(priv->rx_semphr, end >= now ? end - now : UINT32_MAX - now + end)) {
        break;
      }
    } while(!util_timeout(start, xTaskGetTickCount(), priv->rx_timeout));

    taskENTER_CRITICAL();
    len = priv->rx_buf->count < len ? priv->rx_buf->count : len;
    if(len) {
      priv->rx_buf->ops->read(priv->rx_buf, buf, len);
      priv->rx_buf->ops->remove(priv->rx_buf, len);
    }
    taskEXIT_CRITICAL();
    return len;
  } else {
    return HAL_ERR_NOT_SUPPORT;
  }

  return HAL_ERR_UNLIKELY;
}

/**
 * @brief Receive a line
 *
 * @param uart
 * @param buf
 * @param len
 *
 * @return Number of bytes read
 */
static int read_line (hal_uart_t* obj, void* buf, int buf_len) {
  CHECK_OBJ_PRIV(obj);

  return HAL_ERR_NOT_SUPPORT;
}

/**
 * @brief Set timeout for recieve
 *
 * @param uart
 * @param timeout_ms
 *
 * @return HAL_ERR_OK if success
 */
static hal_err_t set_rx_timeout (hal_uart_t* obj, uint16_t timeout_ms) {
  CHECK_OBJ_PRIV(obj);

  priv_data_t* priv = (priv_data_t*) obj->priv;
  priv->rx_timeout = pdMS_TO_TICKS(timeout_ms);

  return HAL_ERR_OK;
}

/**
 * @brief Enable or disable receive
 *
 * @param uart
 * @param en
 *
 * @return HAL_ERR_OK if success
 */
static hal_err_t set_rx_enable (hal_uart_t* obj, bool en) {
  CHECK_OBJ_PRIV(obj);

  priv_data_t* priv = (priv_data_t*) obj->priv;
  if(priv->rx_mode != HAL_UART_RX_MODE_BUFFERED) {
    return HAL_ERR_NOT_SUPPORT;
  }

  if(en) {
    priv->rx_bytes = 1;
    nrfx_uarte_rx(obj->inst, priv->rx_buffer, 1);
  } else {
    priv->rx_bytes = 0;
    nrfx_uarte_rx_abort(obj->inst);
  }

  return HAL_ERR_OK;
}

/**
 * @brief Count bytes received
 *
 * @param uart
 *
 * @return bytes received
 */
static int get_rx_count (hal_uart_t* obj) {
  CHECK_OBJ_PRIV(obj);

  priv_data_t* priv = (priv_data_t*) obj->priv;
  if(priv->rx_mode != HAL_UART_RX_MODE_BUFFERED) {
    return HAL_ERR_NOT_SUPPORT;
  }

  if(!priv->rx_buf) {
    return HAL_ERR_MEMORY;
  }

  return priv->rx_buf->count;
}

/**
 * @brief Clear all data received
 *
 * @param uart
 *
 * @return HAL_ERR_OK if success
 */
static hal_err_t clr_rx_data (hal_uart_t* obj) {
  CHECK_OBJ_PRIV(obj);

  priv_data_t* priv = (priv_data_t*) obj->priv;
  if(priv->rx_mode != HAL_UART_RX_MODE_BUFFERED) {
    return HAL_ERR_NOT_SUPPORT;
  }

  if(!priv->rx_buf) {
    return HAL_ERR_MEMORY;
  }

  taskENTER_CRITICAL();
  priv->rx_buf->ops->clear(priv->rx_buf);
  taskEXIT_CRITICAL();

  return HAL_ERR_OK;
}

/* Global functions ----------------------------------------------------------*/

/**
 * @brief Get an instance
 *
 * @param id
 *
 * @return pointer to the instance if success
 */
hal_uart_t* hal_uart_get_instance(int id) {
  static const hal_uart_ops_t ops = {
    .init = init,
    .deinit = deinit,
    .lock = lock,
    .unlock = unlock,
    .write = write,
    .read = read,
    .read_line = read_line,
    .set_rx_timeout = set_rx_timeout,
    .set_rx_enable = set_rx_enable,
    .get_rx_count = get_rx_count,
    .clr_rx_data = clr_rx_data,
  };

  if(id < 0 || id >= sizeof(obj_tab) / sizeof(*obj_tab)) {
    return NULL;
  }

  hal_uart_t* obj = &obj_tab[id];

  if(!obj->lock) {
    obj->lock = xSemaphoreCreateMutexStatic(&lock_tab[id]);
  }
  if(!obj->lock) {
    return NULL;
  }

  obj->ops = &ops;

  return obj;
}

