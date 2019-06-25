/**
 * @brief The spi abstract layer
 *
 * @file spi.c
 * @date 2018-06-22
 * @author Dengjian
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "nrfx_spim.h"
#include "nrfx_log.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "hal_spi.h"
#include "task.h"

/* Defines -------------------------------------------------------------------*/
#define MAX_XFER_SIZE       255
#define LOCK_TIMEOUT        pdMS_TO_TICKS(2000)
#define XFER_TIMEOUT        pdMS_TO_TICKS(100)

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
  StaticSemaphore_t xfer_semphr_memory;
  SemaphoreHandle_t xfer_semphr;
  hal_spi_cfg_t cfg;
  uint8_t* xfer_buf;
  int16_t  xfer_bytes;
} priv_data_t;

/* Private variables ---------------------------------------------------------*/
static StaticSemaphore_t lock_tab[] = {
  {
  },
  {
  }
};

static nrfx_spim_t inst_tab[] = {
  NRFX_SPIM_INSTANCE(1),
  NRFX_SPIM_INSTANCE(2),
};

static hal_spi_t obj_tab[] = {
  {
    .inst = &inst_tab[0],
    .lock = NULL,
    .priv = NULL,
    .ops = NULL,
  },
  {
    .inst = &inst_tab[1],
    .lock = NULL,
    .priv = NULL,
    .ops = NULL,
  },
};


/* Private functions ---------------------------------------------------------*/

static nrf_spim_mode_t get_mode(hal_spi_mode_t mode) {
  nrf_spim_frequency_t ret = -1;

  switch(mode) {
    case HAL_SPI_MODE_0:
      ret = NRF_SPIM_MODE_0;
      break;
    case HAL_SPI_MODE_1:
      ret = NRF_SPIM_MODE_1;
      break;
    case HAL_SPI_MODE_2:
      ret = NRF_SPIM_MODE_2;
      break;
    case HAL_SPI_MODE_3:
      ret = NRF_SPIM_MODE_3;
      break;
    default:
      break;
  }

  return ret;
}

static nrf_spim_frequency_t get_freq(hal_spi_freq_t freq) {
  nrf_spim_frequency_t ret = -1;

  switch(freq) {
    case HAL_SPI_FREQ_125K:
      ret = NRF_SPIM_FREQ_125K;
      break;
    case HAL_SPI_FREQ_250K:
      ret = NRF_SPIM_FREQ_250K;
      break;
    case HAL_SPI_FREQ_500K:
      ret = NRF_SPIM_FREQ_500K;
      break;
    case HAL_SPI_FREQ_1M:
      ret = NRF_SPIM_FREQ_1M;
      break;
    case HAL_SPI_FREQ_2M:
      ret = NRF_SPIM_FREQ_2M;
      break;
    case HAL_SPI_FREQ_4M:
      ret = NRF_SPIM_FREQ_4M;
      break;
    case HAL_SPI_FREQ_8M:
      ret = NRF_SPIM_FREQ_8M;
      break;
    default:
      break;
  }

  return ret;
}

static void spi_handler(nrfx_spim_evt_t const * p_event, void * p_context)
{
  hal_spi_t* obj = (hal_spi_t*) p_context;

  if(!obj || !obj->priv) {
      return;
  }

  priv_data_t* priv = (priv_data_t*) obj->priv;
  if(!priv->xfer_buf || !priv->xfer_semphr) {
    return;
  }

  if(p_event->xfer_desc.p_tx_buffer) {
    priv->xfer_bytes -= p_event->xfer_desc.tx_length;
    if(priv->xfer_bytes > 0) {
      priv->xfer_buf += p_event->xfer_desc.tx_length;
      nrfx_spim_xfer_desc_t xfer = NRFX_SPIM_XFER_TX(priv->xfer_buf, priv->xfer_bytes > MAX_XFER_SIZE ? MAX_XFER_SIZE : priv->xfer_bytes);
      if(nrfx_spim_xfer(obj->inst, &xfer, 0) != NRFX_SUCCESS) {
      }
    } else {
      xSemaphoreGiveFromISR(priv->xfer_semphr, NULL);
    }
  } else {
    priv->xfer_bytes -= p_event->xfer_desc.rx_length;
    if(priv->xfer_bytes > 0) {
      priv->xfer_buf += p_event->xfer_desc.rx_length;
      nrfx_spim_xfer_desc_t xfer = NRFX_SPIM_XFER_RX(priv->xfer_buf, priv->xfer_bytes > MAX_XFER_SIZE ? MAX_XFER_SIZE : priv->xfer_bytes);
      if(nrfx_spim_xfer(obj->inst, &xfer, 0) != NRFX_SUCCESS) {
      }
    } else {
      xSemaphoreGiveFromISR(priv->xfer_semphr, NULL);
    }
  }
}

/**
 * @brief Init
 *
 * @param obj
 * @param cfg
 *
 * @return HAL_ERR_OK if success
 */
 static hal_err_t init (hal_spi_t *obj, hal_spi_cfg_t *cfg) {
  CHECK_OBJ(obj);

  if(!obj->priv) {
    obj->priv = pvPortMalloc(sizeof(priv_data_t));
  }

  if(!obj->priv) {
    return HAL_ERR_MEMORY;
  }

  priv_data_t* priv = (priv_data_t*) obj->priv;
  memcpy(&priv->cfg, cfg, sizeof(priv->cfg));

  return HAL_ERR_OK;
}

/**
 * @brief Deinit
 *
 * @param obj
 *
 * @return HAL_ERR_OK if success
 */
 static hal_err_t deinit (hal_spi_t *obj) {
  CHECK_OBJ_PRIV(obj);

  vPortFree(obj->priv);
  obj->priv = NULL;

  return HAL_ERR_OK;
}

/**
 * @brief Open
 *
 * @param obj
 *
 * @return HAL_ERR_OK if success
 */
 static hal_err_t open (hal_spi_t *obj) {
  CHECK_OBJ_PRIV(obj);

  priv_data_t* priv = (priv_data_t*)obj->priv;
  hal_spi_cfg_t* cfg = &priv->cfg;

  nrf_spim_frequency_t freq = get_freq(cfg->freq);
  nrf_spim_mode_t mode = get_mode(cfg->mode);
  if(freq == -1 || mode == -1) {
    return HAL_ERR_PARAM;
  }

  priv->xfer_semphr = xSemaphoreCreateBinaryStatic(&priv->xfer_semphr_memory);

  nrfx_spim_config_t nrf_cfg;
  nrf_cfg.sck_pin = cfg->sck_pin;;
  nrf_cfg.mosi_pin = cfg->mosi_pin;
  nrf_cfg.miso_pin = cfg->miso_pin;
  nrf_cfg.ss_pin = NRFX_SPIM_PIN_NOT_USED;
  nrf_cfg.irq_priority = APP_IRQ_PRIORITY_HIGH;
  nrf_cfg.orc = 0xff;
  nrf_cfg.mode = mode;
  nrf_cfg.frequency = freq;
  nrf_cfg.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST;

  if(nrfx_spim_init(obj->inst, &nrf_cfg, spi_handler, obj) != NRF_SUCCESS) {
    return HAL_ERR_UNLIKELY;
  }

  return HAL_ERR_OK;
}

/**
 * @brief Close
 *
 * @param obj
 *
 * @return HAL_ERR_OK if success
 */
 static hal_err_t close (hal_spi_t *obj) {
  CHECK_OBJ(obj);

  nrfx_spim_uninit(obj->inst);

  // Fix power consumption according to https://devzone.nordicsemi.com/f/nordic-q-a/26030/how-to-reach-nrf52840-uarte-current-supply-specification/102605#102605
  *(volatile uint32_t *)((uint32_t)((nrfx_spim_t*)obj->inst)->p_reg + 0xFFC) = 0;
  *(volatile uint32_t *)((uint32_t)((nrfx_spim_t*)obj->inst)->p_reg + 0xFFC);
  *(volatile uint32_t *)((uint32_t)((nrfx_spim_t*)obj->inst)->p_reg + 0xFFC) = 1;

  return HAL_ERR_OK;
}

/**
 * @brief Lock
 *
 * @param obj
 *
 * @return HAL_ERR_OK if success
 */
static hal_err_t lock(hal_spi_t *obj) {
  CHECK_OBJ_LOCK(obj);

  if(!xSemaphoreTake(obj->lock, LOCK_TIMEOUT)) {
    return HAL_ERR_BUSY;
  }

  return HAL_ERR_OK;
}

/**
 * @brief Unlock
 *
 * @param obj
 *
 * @return HAL_ERR_OK if success
 */
static hal_err_t unlock(hal_spi_t *obj) {
  CHECK_OBJ_LOCK(obj);

  if(!xSemaphoreGive(obj->lock)) {
    return HAL_ERR_UNLIKELY;
  }

  return HAL_ERR_OK;
}

/**
 * @brief Write bytes
 *
 * @param obj
 * @param buf
 * @param len
 *
 * @return Number of bytes written
 */
static hal_err_t write(hal_spi_t *obj, void* buf, int len) {
  CHECK_OBJ_PRIV(obj);

  if(!buf || len <= 0) {
    return HAL_ERR_PARAM;
  }

  priv_data_t* priv = (priv_data_t*) obj->priv;
  if(!priv->xfer_semphr) {
    return HAL_ERR_MEMORY;
  }

  priv->xfer_buf = buf;
  priv->xfer_bytes = len;
  xSemaphoreTake(priv->xfer_semphr, 0);
  nrfx_spim_xfer_desc_t xfer = NRFX_SPIM_XFER_TX(buf, len > MAX_XFER_SIZE ? MAX_XFER_SIZE : len);

  if(nrfx_spim_xfer(obj->inst, &xfer, 0) != NRFX_SUCCESS) {
    return HAL_ERR_DENIAL;
  }

  if(!xSemaphoreTake(priv->xfer_semphr, XFER_TIMEOUT)) {
    priv->xfer_buf = NULL;
    priv->xfer_bytes = 0;
    return HAL_ERR_TIMEOUT;
  }

  return len - priv->xfer_bytes;
}

/**
 * @brief Read bytes
 *
 * @param obj
 * @param buf
 * @param len
 *
 * @return Number of bytes read
 */
static hal_err_t read(hal_spi_t *obj, void* buf, int len) {
  CHECK_OBJ_PRIV(obj);

  if(!buf || len <= 0) {
    return HAL_ERR_PARAM;
  }

  priv_data_t* priv = (priv_data_t*) obj->priv;
  if(!priv->xfer_semphr) {
    return HAL_ERR_MEMORY;
  }

  priv->xfer_buf = buf;
  priv->xfer_bytes = len;
  xSemaphoreTake(priv->xfer_semphr, 0);
  nrfx_spim_xfer_desc_t xfer = NRFX_SPIM_XFER_RX(buf, len > MAX_XFER_SIZE ? MAX_XFER_SIZE : len);

  if(nrfx_spim_xfer(obj->inst, &xfer, 0) != NRFX_SUCCESS) {
    return HAL_ERR_DENIAL;
  }

  if(!xSemaphoreTake(priv->xfer_semphr, XFER_TIMEOUT)) {
    priv->xfer_buf = NULL;
    priv->xfer_bytes = 0;
    return HAL_ERR_TIMEOUT;
  }

  return len - priv->xfer_bytes;
}

/**
 * @brief Set ss pin high
 *
 * @param obj
 *
 * @return HAL_ERR_OK if success
 */
static hal_err_t ss_high(hal_spi_t *obj) {
  CHECK_OBJ_PRIV(obj);

  priv_data_t* priv = (priv_data_t*) obj->priv;
  if(priv->cfg.ss_pin != HAL_PIN_NOT_USED) {
    hal_pin_set_mode_ex(priv->cfg.ss_pin, HAL_PIN_MODE_OUT, HAL_PIN_MODE_EX_NORMAL, HAL_PIN_LVL_HIGH);
  }

  return HAL_ERR_OK;
}

/**
 * @brief Set ss pin low
 *
 * @param obj
 *
 * @return HAL_ERR_OK if success
 */
static hal_err_t ss_low(hal_spi_t *obj) {
  CHECK_OBJ_PRIV(obj);

  priv_data_t* priv = (priv_data_t*) obj->priv;
  if(priv->cfg.ss_pin != HAL_PIN_NOT_USED) {
    hal_pin_set_mode_ex(priv->cfg.ss_pin, HAL_PIN_MODE_OUT, HAL_PIN_MODE_EX_NORMAL, HAL_PIN_LVL_LOW);
  }

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
hal_spi_t* hal_spi_get_instance(int id) {
  static const hal_spi_ops_t ops = {
    .init = init,
    .deinit = deinit,
    .open = open,
    .close = close,
    .lock = lock,
    .unlock = unlock,
    .read = read,
    .write = write,
    .ss_high = ss_high,
    .ss_low = ss_low,
  };

  if(id < 0 || id >= sizeof(obj_tab) / sizeof(*obj_tab)) {
    return NULL;
  }

  hal_spi_t* obj = &obj_tab[id];

  if(!obj->lock) {
    obj->lock = xSemaphoreCreateMutexStatic(&lock_tab[id]);
  }
  if(!obj->lock) {
    return NULL;
  }

  obj->ops = &ops;

  return obj;
}

