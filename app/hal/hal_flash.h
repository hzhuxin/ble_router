/**
 * @brief The flash abstract layer
 *
 * @file dev_flash.h
 * @date 2018-06-26
 * @author ChenJiaqi
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

#ifndef __FLASH_LIGHT_H__
#define __FLASH_LIGHT_H__
/* Includes ------------------------------------------------------------------*/
#include "hal.h"
#include "hal_spi.h"
//#include "dev_interface.h"
#include <stdint.h>


/* Defines -------------------------------------------------------------------*/

/* Typedefs ------------------------------------------------------------------*/
//struct flash_ops;

typedef struct hal_flash {
  const struct hal_flash_ops* ops;
  void *lock;
  void *priv;
} hal_flash_t;

typedef struct hal_flash_ops {
  hal_err_t (*init) (hal_flash_t *obj);
  hal_err_t (*deinit) (hal_flash_t *obj);
  hal_err_t (*test) (hal_flash_t *obj);

  /** Read from flash*/
  hal_err_t(*read) (hal_flash_t *obj, uint32_t addr, uint8_t *buf,uint32_t nbytes); // bytes size
  /** Write to flash*/
  hal_err_t(*write) (hal_flash_t *obj, uint32_t addr, const uint8_t *buf, uint32_t nbytes);
  /** Erase*/
  hal_err_t (*erase) (hal_flash_t *obj,uint32_t addr,uint32_t len);
  /** Get chip capabity(bytes)*/
  hal_err_t (*get_chipsize) (hal_flash_t *obj);
  /** Get max program size (bytes)*/
  hal_err_t (*get_programsize) (hal_flash_t *obj);
  /** Get minimal earse size (bytes)*/
  hal_err_t (*get_erasesize) (hal_flash_t *obj);

  hal_err_t (*lock) (hal_flash_t *obj);
  hal_err_t (*unlock) (hal_flash_t *obj);
} hal_flash_ops_t;

//hal_flash_t * hal_flash_get_instance(hal_interface_t *interface);
hal_flash_t * hal_flash_get_instance(uint32_t addr_start, uint32_t size);
#endif
