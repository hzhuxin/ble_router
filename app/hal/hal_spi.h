/**
 * @brief The spi abstract layer
 * 
 * @file spi.h
 * @date 2018-06-22
 * @author Dengjian
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

#ifndef __SPI_H__
#define __SPI_H__

/* Includes ------------------------------------------------------------------*/
#include "hal.h"
#include "hal_pin.h"

/* Typedefs ------------------------------------------------------------------*/

typedef enum hal_spi_mode {
  HAL_SPI_MODE_0 = 0,
  HAL_SPI_MODE_1,
  HAL_SPI_MODE_2,
  HAL_SPI_MODE_3,
  HAL_SPI_MODE_MAX,
} hal_spi_mode_t;

typedef enum hal_spi_freq {
  HAL_SPI_FREQ_125K = 0,
  HAL_SPI_FREQ_250K,
  HAL_SPI_FREQ_500K,
  HAL_SPI_FREQ_1M,
  HAL_SPI_FREQ_2M,
  HAL_SPI_FREQ_4M,
  HAL_SPI_FREQ_8M,
  HAL_SPI_FREQ_16M,
  HAL_SPI_FREQ_MAX,
} hal_spi_freq_t;

typedef struct hal_spi_cfg {
  hal_spi_mode_t mode;
  hal_spi_freq_t freq;
  hal_pin_t ss_pin; // CAUTION: the ss pin is controled by user
  hal_pin_t sck_pin;
  hal_pin_t miso_pin;
  hal_pin_t mosi_pin;
} hal_spi_cfg_t;

struct hal_spi_ops;
typedef struct hal_spi {
  void* inst;
  void* lock;
  void* priv;
  const struct hal_spi_ops* ops;
} hal_spi_t;

typedef struct hal_spi_ops {
  hal_err_t (*init) (hal_spi_t *obj, hal_spi_cfg_t* cfg);
  hal_err_t (*deinit) (hal_spi_t *obj);

  hal_err_t (*open) (hal_spi_t *obj);
  hal_err_t (*close) (hal_spi_t *obj);

  hal_err_t (*lock) (hal_spi_t *obj);
  hal_err_t (*unlock) (hal_spi_t *obj);

  hal_err_t (*read) (hal_spi_t *obj, void* buf, int len); // CAUTION: you must control the ss pin first
  hal_err_t (*write) (hal_spi_t *obj, void* buf, int len); // CAUTION: you must control the ss pin first

  hal_err_t (*ss_high) (hal_spi_t *obj);
  hal_err_t (*ss_low) (hal_spi_t *obj);
} hal_spi_ops_t;

/* Global variables ----------------------------------------------------------*/

/* Defines -------------------------------------------------------------------*/

hal_spi_t* hal_spi_get_instance(int id);

#endif // #ifndef __SPI_H__

