/**
 * @brief The adc abstract layer
 * 
 * @file adc.h
 * @date 2018-06-22
 * @author Dengjian
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

#ifndef __ADC_H__
#define __ADC_H__

/* Includes ------------------------------------------------------------------*/
#include "hal.h"
#include "hal_pin.h"

/* Typedefs ------------------------------------------------------------------*/
typedef enum hal_adc_reference { // Reference
  HAL_ADC_REFERENCE_INT = 0,
  HAL_ADC_REFERENCE_EXT,
  HAL_ADC_REFERENCE_VDD,
  HAL_ADC_REFERENCE_MAX,
} hal_adc_reference_t;

typedef struct hal_adc_cfg { // ADC configuration
  int16_t resolution;
  int16_t oversample;
} hal_adc_cfg_t;

typedef struct hal_adc_channel { // Channel configuration
  hal_pin_t pin_p;
  hal_pin_t pin_n; // HAL_PIN_NOT_USED for single ended channels
  float gain; // Voltage gain
  int acqtime; // Acquisition time in microsecond
  hal_adc_reference_t reference;
} hal_adc_channel_t;

/* Defines -------------------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/

/* Global functions ----------------------------------------------------------*/

hal_err_t hal_adc_init(hal_adc_cfg_t* cfg, hal_adc_channel_t* channel, int nb_channels);
hal_err_t hal_adc_deinit(void);
hal_err_t hal_adc_update(void);
hal_err_t hal_adc_get_voltage(int channel, float* output);
hal_err_t hal_adc_get_sample(int channel, int16_t* output);
hal_err_t hal_adc_calibrate(void* cali);

#endif // #ifndef __ADC_H__

