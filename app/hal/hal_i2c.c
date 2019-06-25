/**
 * @brief The i2c abstract layer
 *
 * @file i2c.c
 * @date 2018-06-22
 * @author Dengjian
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "nrfx_twim.h"
#include "nrfx_log.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "hal_i2c.h"

/* Defines -------------------------------------------------------------------*/
#define LOCK_TIMEOUT        pdMS_TO_TICKS(2000)
#define XFER_TIMEOUT        pdMS_TO_TICKS(100)
#define DEV_ADDR            (0x3E >> 1)

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
  hal_i2c_cfg_t cfg;
  bool xfer_done;
} priv_data_t;

/* Private variables ---------------------------------------------------------*/
StaticSemaphore_t lock_tab[] = {
  {
  },
};

static nrfx_twim_t inst_tab[] = {
  NRFX_TWIM_INSTANCE(0),
};

static hal_i2c_t obj_tab[] = {
  {
    .inst = &inst_tab[0],
    .lock = NULL,
    .priv = NULL,
    .ops = NULL,
  },
};

/* Private functions ---------------------------------------------------------*/

static nrf_twim_frequency_t get_freq(hal_i2c_freq_t freq) {
  nrf_twim_frequency_t ret = -1;

  switch(freq) {
    case HAL_I2C_FREQ_100K:
      ret = NRF_TWIM_FREQ_100K;
      break;
    case HAL_I2C_FREQ_400K:
      ret = NRF_TWIM_FREQ_400K;
      break;
    default:
      break;
  }

  return ret;
}

static void twi_handler(nrfx_twim_evt_t const * p_event, void * p_context)
{
  hal_i2c_t* obj = (hal_i2c_t*) p_context;

  if(!obj || !obj->priv) {
      return;
  }

  priv_data_t* priv = (priv_data_t*) obj->priv;

  switch (p_event->type)
  {
    case NRFX_TWIM_EVT_DONE:
      if (p_event->xfer_desc.type == NRFX_TWIM_XFER_RX ||
              p_event->xfer_desc.type == NRFX_TWIM_XFER_TX ||
              p_event->xfer_desc.type == NRFX_TWIM_XFER_TXRX ||
              p_event->xfer_desc.type == NRFX_TWIM_XFER_TXTX) {
        priv->xfer_done = true;
        if(priv->xfer_semphr) {
          xSemaphoreGive(priv->xfer_semphr);
        }
      }
      break;

    case NRFX_TWIM_EVT_ADDRESS_NACK:
    case NRFX_TWIM_EVT_DATA_NACK:
      priv->xfer_done = false;
      if(priv->xfer_semphr) {
        xSemaphoreGive(priv->xfer_semphr);
      }
      break;

    default:
      break;
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
static hal_err_t init (hal_i2c_t *obj, hal_i2c_cfg_t* cfg) {
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
static hal_err_t deinit (hal_i2c_t *obj) {
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
static hal_err_t open (hal_i2c_t *obj) {
  CHECK_OBJ_PRIV(obj);

  priv_data_t* priv = (priv_data_t*) obj->priv;
  hal_i2c_cfg_t* cfg = &priv->cfg;

  nrf_twim_frequency_t freq = get_freq(cfg->freq);
  if(freq == -1) {
    return HAL_ERR_PARAM;
  }

  priv->xfer_semphr = xSemaphoreCreateBinaryStatic(&priv->xfer_semphr_memory);

  nrfx_twim_config_t nrf_cfg;
  nrf_cfg.scl = cfg->scl_pin;
  nrf_cfg.sda = cfg->sda_pin;
  nrf_cfg.frequency = freq;
  nrf_cfg.interrupt_priority = APP_IRQ_PRIORITY_HIGH,
  nrf_cfg.hold_bus_uninit = false;

  if(nrfx_twim_init(obj->inst, &nrf_cfg, twi_handler, priv) != NRF_SUCCESS) {
    return HAL_ERR_UNLIKELY;
  }

  nrfx_twim_enable(obj->inst);

  return HAL_ERR_OK;
}

/**
 * @brief Close
 *
 * @param obj
 *
 * @return HAL_ERR_OK if success
 */
static hal_err_t close (hal_i2c_t *obj) {
  CHECK_OBJ_PRIV(obj);

  nrfx_twim_uninit(obj->inst);

  // Fix power consumption according to https://devzone.nordicsemi.com/f/nordic-q-a/26030/how-to-reach-nrf52840-uarte-current-supply-specification/102605#102605
  *(volatile uint32_t *)((uint32_t)((nrfx_twim_t*)obj->inst)->p_twim + 0xFFC) = 0;
  *(volatile uint32_t *)((uint32_t)((nrfx_twim_t*)obj->inst)->p_twim + 0xFFC);
  *(volatile uint32_t *)((uint32_t)((nrfx_twim_t*)obj->inst)->p_twim + 0xFFC) = 1;

  return HAL_ERR_OK;
}

/**
 * @brief Lock
 *
 * @param obj
 *
 * @return HAL_ERR_OK if success
 */
static hal_err_t lock(hal_i2c_t *obj) {
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
static hal_err_t unlock(hal_i2c_t *obj) {
  CHECK_OBJ_LOCK(obj);

  if(!xSemaphoreGive(obj->lock)) {
    return HAL_ERR_UNLIKELY;
  }

  return HAL_ERR_OK;
}

/**
 * @brief Write bytes
 *
 * @param i2c
 * @param dev_addr
 * @param buf
 * @param len
 *
 * @return Number of byte written
 */
static hal_err_t write(hal_i2c_t *obj, int dev_addr, void* buf, int len) {
  CHECK_OBJ_PRIV(obj);

  if(!buf || len <= 0) {
    return HAL_ERR_PARAM;
  }

  priv_data_t* priv = (priv_data_t*) obj->priv;
  if(!priv->xfer_semphr) {
    return HAL_ERR_MEMORY;
  }

  priv->xfer_done = false;
  xSemaphoreTake(priv->xfer_semphr, 0);

  if(nrfx_twim_tx(obj->inst, dev_addr, buf, len, false) != NRFX_SUCCESS) {
    return HAL_ERR_DENIAL;
  }

  if(xSemaphoreTake(priv->xfer_semphr, XFER_TIMEOUT) != pdTRUE) {
    return HAL_ERR_TIMEOUT;
  }

  if(!priv->xfer_done) {
    return HAL_ERR_FAIL;
  }

  return len;
}

/**
 * @brief Read bytes
 *
 * @param obj
 * @param dev_addr
 * @param buf
 * @param len
 *
 * @return Number of bytes read
 */
static hal_err_t read(hal_i2c_t *obj, int dev_addr, void* buf, int len) {
  CHECK_OBJ_PRIV(obj);

  if(!buf || len <= 0) {
    return HAL_ERR_PARAM;
  }

  priv_data_t* priv = (priv_data_t*) obj->priv;
  if(!priv->xfer_semphr) {
    return HAL_ERR_MEMORY;
  }

  priv->xfer_done = false;
  xSemaphoreTake(priv->xfer_semphr, 0);

  if(nrfx_twim_rx(obj->inst, dev_addr, buf, len) != NRFX_SUCCESS) {
    return HAL_ERR_DENIAL;
  }

  if(xSemaphoreTake(priv->xfer_semphr, XFER_TIMEOUT) != pdTRUE) {
    return HAL_ERR_TIMEOUT;
  }

  if(!priv->xfer_done) {
    return HAL_ERR_FAIL;
  }

  return len;
}

/**
 * @brief Write byte to register
 *
 * @param obj
 * @param dev_addr
 * @param reg_addr
 * @param reg_addr_len
 * @param buf
 * @param len
 *
 * @return Number of bytes written
 */
static hal_err_t write_reg (hal_i2c_t *obj, int dev_addr, int reg_addr, int reg_addr_len, void* buf, int len) {
  CHECK_OBJ_PRIV(obj);

  if(!buf || len <= 0) {
    return HAL_ERR_PARAM;
  }

  priv_data_t* priv = (priv_data_t*) obj->priv;
  if(!priv->xfer_semphr) {
    return HAL_ERR_MEMORY;
  }

  uint8_t* xfer_buf = pvPortMalloc(reg_addr_len + len);
  if(!xfer_buf) {
    return HAL_ERR_MEMORY;
  } else {
    memcpy(xfer_buf, &reg_addr, reg_addr_len);
    memcpy(xfer_buf + reg_addr_len, buf, len);
  }

  priv->xfer_done = false;
  xSemaphoreTake(priv->xfer_semphr, 0);

  nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TX(dev_addr, xfer_buf, reg_addr_len + len);
  if(nrfx_twim_xfer(obj->inst, &xfer, 0) != NRFX_SUCCESS) {
    vPortFree(xfer_buf);
    return HAL_ERR_DENIAL;
  }

  if(xSemaphoreTake(priv->xfer_semphr, XFER_TIMEOUT) != pdTRUE) {
    vPortFree(xfer_buf);
    return HAL_ERR_TIMEOUT;
  }

  if(!priv->xfer_done) {
    vPortFree(xfer_buf);
    return HAL_ERR_FAIL;
  }

  vPortFree(xfer_buf);
  return len;
}

/**
 * @brief Read bytes from register
 *
 * @param obj
 * @param dev_addr
 * @param reg_addr
 * @param reg_addr_len
 * @param buf
 * @param len
 *
 * @return Number of byte read
 */
static hal_err_t read_reg (hal_i2c_t *obj, int dev_addr, int reg_addr, int reg_addr_len, void* buf, int len) {
  CHECK_OBJ_PRIV(obj);

  if(!buf || len <= 0) {
    return HAL_ERR_PARAM;
  }

  priv_data_t* priv = (priv_data_t*) obj->priv;
  if(!priv->xfer_semphr) {
    return HAL_ERR_MEMORY;
  }

  priv->xfer_done = false;
  xSemaphoreTake(priv->xfer_semphr, 0);

  nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TXRX(dev_addr, (uint8_t*)&reg_addr, reg_addr_len, buf, len);
  if(nrfx_twim_xfer(obj->inst, &xfer, 0) != NRFX_SUCCESS) {
    return HAL_ERR_DENIAL;
  }

  if(xSemaphoreTake(priv->xfer_semphr, XFER_TIMEOUT) != pdTRUE) {
    return HAL_ERR_TIMEOUT;
  }

  if(!priv->xfer_done) {
    return HAL_ERR_FAIL;
  }

  return len;
}

/* Global functions ----------------------------------------------------------*/

/**
 * @brief Get an instance
 *
 * @param id
 *
 * @return pointer to the instance if success
 */
hal_i2c_t* hal_i2c_get_instance(int id) {
static const hal_i2c_ops_t ops = {
    .init = init,
    .deinit = deinit,
    .open = open,
    .close = close,
    .lock = lock,
    .unlock = unlock,
    .read = read,
    .write = write,
    .read_reg = read_reg,
    .write_reg = write_reg,
  };

  if(id < 0 || id >= sizeof(obj_tab) / sizeof(*obj_tab)) {
    return NULL;
  }

  hal_i2c_t* obj = &obj_tab[id];

  if(!obj->lock) {
    obj->lock = xSemaphoreCreateMutexStatic(&lock_tab[id]);
  }
  if(!obj->lock) {
    return NULL;
  }

  obj->ops = &ops;

  return obj;
}

