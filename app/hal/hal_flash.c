/**
 * @brief
 *
 * @file hal_flash_nrf52.c
 * @date 2018-09-28
 * @author ChenJiaqi
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "dev_flash.h"
#include "debug.h"
#include "hal_cfg.h"
#include "nrf_fstorage.h"

DBG_SET_LEVEL(DBG_LEVEL_V);

/* Defines -------------------------------------------------------------------*/
#define MAX_PROGRAM_SIZE    1024
#define LOCK_TIMEOUT        pdMS_TO_TICKS(2000)

#define PROG_UNIT           4
#define ERASE_UNIT          4096
#define PROG_MSK            (PROG_UNIT - 1)  // prog size is 4 bytes
#define ERASE_MSK           (ERASE_UNIT - 1) // erase size if 4096 bytes

#define CHECK_ADDR_INVALID(addr, mask) ((addr) & (mask))
#define OPERATION_TIMEOUT          pdMS_TO_TICKS(500)

#if defined(configSUPPORT_STATIC_ALLOCATION) && configSUPPORT_STATIC_ALLOCATION
#define USE_LOCK_STATIC 1
static StaticSemaphore_t _lock;
static StaticSemaphore_t _oper_semphr_memory;
#else
#define USE_LOCK_STATIC 0
#endif

/* Typedefs ------------------------------------------------------------------*/
typedef struct priv_data {
  nrf_fstorage_t *fs;
  SemaphoreHandle_t *oper_sem;
  uint32_t len; // save operation len
  uint32_t chip_size;
  bool result; // save operation result
}priv_data_t;

/* Private variables ---------------------------------------------------------*/
static dev_flash_t _device;
static nrf_fstorage_t _fstorage;

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
  dev_flash_t *obj = (dev_flash_t *)p_evt->p_param;
  priv_data_t *priv = obj->priv;

  priv->result = (p_evt->result == NRF_SUCCESS);
  priv->len = p_evt->len;
  xSemaphoreGiveFromISR(priv->oper_sem, NULL);
}

static hal_err_t flash_nrf52_test (dev_flash_t *obj) {
  if(!obj || !obj->priv) {
    return HAL_ERR_NOT_INITIALIZED;
  }
#if 0
  priv_data_t *priv = obj->priv;
  nrf_fstorage_t *fs = priv->fs;
  DBG_I("========| flash info |========");
  DBG_I("start addr: \t%x",      fs->start_addr);
  DBG_I("total size : \t%dkb",      (fs->end_addr - fs->start_addr)>>10);
  DBG_I("erase unit: \t%d bytes",      fs->p_flash_info->erase_unit);
  DBG_I("program unit: \t%d bytes",    fs->p_flash_info->program_unit);
  DBG_I("==============================");
#endif
  return HAL_ERR_OK;
}

static hal_err_t flash_nrf52_init (dev_flash_t *obj) {
  return HAL_ERR_OK;
}

static hal_err_t flash_nrf52_deinit(dev_flash_t *obj) {
  if(!obj || !obj->priv) {
    return HAL_ERR_PARAM;
  }

  priv_data_t *priv = obj->priv;
  nrf_fstorage_t *fs = priv->fs;
  if(fs) {
    nrf_fstorage_uninit(fs, NULL);
  }

#if USE_LOCK_STATIC
#else
  if(obj->lock) {
    vSemaphoreDelete(obj->lock); // delete lock
    obj->lock = NULL;
  }
  if(priv->oper_sem) {
    vSemaphoreDelete(priv->oper_sem);
  }
#endif

  vPortFree(priv); // delete priv
  obj->priv = NULL;
  return HAL_ERR_OK;
}

static hal_err_t flash_nrf52_get_chipsize (dev_flash_t *obj) {
  if(!obj || !obj->priv) {
    return HAL_ERR_PARAM;
  }

  priv_data_t *priv = obj->priv;

  return priv->chip_size;
}

static hal_err_t flash_nrf52_get_erasesize (dev_flash_t *obj) {

  if(!obj || !obj->priv) {
    return HAL_ERR_PARAM;
  }

  priv_data_t *priv = obj->priv;
  if(!priv->fs) {
    return HAL_ERR_NOT_INITIALIZED;
  }
  return priv->fs->p_flash_info->erase_unit;
}

static hal_err_t flash_nrf52_get_progsize (dev_flash_t *obj) {
  if(!obj || !obj->priv) {
    return HAL_ERR_PARAM;
  }

  return MAX_PROGRAM_SIZE;
}

static hal_err_t flash_nrf52_lock(dev_flash_t *obj) {
  if(!obj || !obj->lock) {
    return HAL_ERR_NOT_INITIALIZED;
  }

  if(!xSemaphoreTake(obj->lock, LOCK_TIMEOUT)) {
    return HAL_ERR_BUSY;
  }

  return HAL_ERR_OK;
}

static hal_err_t flash_nrf52_unlock(dev_flash_t *obj) {
  if(!obj || !obj->lock) {
    return HAL_ERR_NOT_INITIALIZED;
  }

  xSemaphoreGive(obj->lock);

  return HAL_ERR_OK;
}

hal_err_t flash_nrf52_read(dev_flash_t *obj, uint32_t addr, uint8_t *buf, uint32_t nbytes) {
  if(!obj || !obj->priv || CHECK_ADDR_INVALID(addr, PROG_MSK) || CHECK_ADDR_INVALID(nbytes, PROG_MSK)) {
    DBG_E("Inner flash: error %08x %08x %d %d", addr, nbytes, CHECK_ADDR_INVALID(addr, PROG_MSK), CHECK_ADDR_INVALID(nbytes, PROG_MSK));
    return HAL_ERR_PARAM;
  }

  priv_data_t *priv = obj->priv;

  uint32_t chip_size = priv->chip_size;
  if(addr + nbytes > chip_size) {
    return HAL_ERR_PARAM;
  }

  nrf_fstorage_t *fs = priv->fs;
  uint32_t base_addr = fs->start_addr;
  uint32_t phy_addr = base_addr + addr;

  ret_code_t ret = nrf_fstorage_read(fs, phy_addr, buf, nbytes);
  if(ret) {
    DBG_E("Inner flash:ret code is %d", ret);
    return HAL_ERR_FAIL;
  }

  return nbytes;
}

hal_err_t flash_nrf52_write(dev_flash_t *obj, uint32_t addr, const uint8_t *buf, uint32_t nbytes) {
  if(!obj || !obj->priv || CHECK_ADDR_INVALID(addr, PROG_MSK) || CHECK_ADDR_INVALID(nbytes, PROG_MSK)) {
    DBG_E("Inner flash: Write Error");
    return HAL_ERR_PARAM;
  }

  priv_data_t *priv = obj->priv;
  uint32_t chip_size = priv->chip_size;

  if(addr + nbytes > chip_size) {
    return HAL_ERR_PARAM;
  }

  nrf_fstorage_t *fs = priv->fs;
  uint32_t base_addr = fs->start_addr;
  uint32_t phy_addr = base_addr + addr;
  ret_code_t ret;
  uint32_t erase_mask = obj->ops->get_erasesize(obj) - 1;

  uint32_t start_of_next_page = (addr | erase_mask) + 1;
  if (nbytes + addr > start_of_next_page) {
    if (obj->ops->erase(obj, start_of_next_page, erase_mask + 1)) {
      DBG_E("Inner flash: erase failed");
      return HAL_ERR_FAIL;
    }
  }
/*
  if((phy_addr & erase_mask) == 0) {
    if(obj->ops->erase(obj, addr, erase_mask + 1)) {
      DBG_I("Inner flash: erase failed");
      return HAL_ERR_FAIL;
    }
  }
*/
  xSemaphoreTake(priv->oper_sem, 0);
  ret = nrf_fstorage_write(fs, phy_addr, buf, nbytes, obj);
  if(ret) {
    DBG_E("Inner flash: ret code is %d", ret);
    return HAL_ERR_UNLIKELY;
  }

  if(!xSemaphoreTake(priv->oper_sem,OPERATION_TIMEOUT)) {
    return HAL_ERR_TIMEOUT;
  }

  return priv->result ? priv->len : 0;
}

hal_err_t flash_nrf52_erase(dev_flash_t *obj, uint32_t addr, uint32_t len) {
  if(!obj || !obj->priv || CHECK_ADDR_INVALID(addr, ERASE_MSK) || CHECK_ADDR_INVALID(len, ERASE_MSK) || len < ERASE_UNIT) {
    DBG_E("Inner flash: param error");
    return HAL_ERR_PARAM;
  }

  priv_data_t *priv = obj->priv;
  uint32_t chip_size = priv->chip_size;
  if(addr + len > chip_size) {
    return HAL_ERR_PARAM;
  }

  nrf_fstorage_t *fs = priv->fs;
  uint32_t base_addr = fs->start_addr;
  uint32_t phy_addr = base_addr + addr;

  uint32_t pages = len / ERASE_UNIT;

  xSemaphoreTake(priv->oper_sem, 0);

  ret_code_t ret = nrf_fstorage_erase(fs, phy_addr, pages, obj);
  if(ret) {
    DBG_W("Inner flash: ret code is %d, %08x", ret, phy_addr);
    return HAL_ERR_UNLIKELY;
  }

  if(!xSemaphoreTake(priv->oper_sem,OPERATION_TIMEOUT * 10)) {
    return HAL_ERR_TIMEOUT;
  }

  return priv->result ? HAL_ERR_OK : HAL_ERR_FAIL;
}

dev_flash_t * hal_flash_get_instance(uint32_t base_addr, uint32_t size) {

  dev_flash_t *obj = &_device;
  if(obj->priv) {
    return obj;
  }

  if(CHECK_ADDR_INVALID(base_addr, ERASE_MSK) || CHECK_ADDR_INVALID(size, ERASE_MSK) || size < ERASE_UNIT) {
    return NULL;
  }
  // FIXME: we should judge if base_addr and size is valid values
  static const dev_flash_ops_t ops = {
    .init = flash_nrf52_init,
    .deinit = flash_nrf52_deinit,
    .test = flash_nrf52_test,
    .read = flash_nrf52_read,
    .write = flash_nrf52_write,
    .erase = flash_nrf52_erase,
    .get_chipsize = flash_nrf52_get_chipsize,
    .get_programsize = flash_nrf52_get_progsize,
    .get_erasesize = flash_nrf52_get_erasesize,
    .lock = flash_nrf52_lock,
    .unlock = flash_nrf52_unlock,
  };

  nrf_fstorage_t *fs= &_fstorage;
  fs->evt_handler = fstorage_evt_handler;
  fs->start_addr = base_addr;
  fs->end_addr   = base_addr + size; // 256k

  extern nrf_fstorage_api_t nrf_fstorage_sd;
  ret_code_t ret = nrf_fstorage_init(fs, &nrf_fstorage_sd, obj);
  if(ret) {
    DBG_E("Inner flash: nrf_storage_init error %d", ret);
    return NULL;
  }

   if(!obj->lock) {
#if USE_LOCK_STATIC
  obj->lock = xSemaphoreCreateMutexStatic(&_lock);
#else
  obj->lock = xSemaphoreCreateMutex();
#endif
  }

  if(!obj->lock) {
    DBG_E("Inner flash: Create lock failed");
    nrf_fstorage_uninit(fs, NULL);
    return NULL;
  }

 priv_data_t *priv = (priv_data_t *)pvPortMalloc(sizeof(priv_data_t));
  if(!priv) {
    DBG_E("Inner flash: memory error");
    nrf_fstorage_uninit(fs, NULL);
    return NULL;
  }
  memset(priv, 0, sizeof(priv_data_t));

#if USE_LOCK_STATIC
  priv->oper_sem= xSemaphoreCreateBinaryStatic(&_oper_semphr_memory);
#else
  priv->operation = xSemaphoreCreateBinary();
#endif
  if(!priv->oper_sem) {
    DBG_E("Inner flash: create Semaphore Error");
    nrf_fstorage_uninit(fs, NULL);
    vPortFree(priv);
    return NULL;
  }

  priv->fs = fs;
  priv->chip_size = fs->end_addr - fs->start_addr;

  obj->priv = priv;
  obj->ops = &ops;
  xSemaphoreGive(obj->lock);
  xSemaphoreGive(priv->oper_sem);
  return obj;
}
#undef USE_LOCK_STATIC
