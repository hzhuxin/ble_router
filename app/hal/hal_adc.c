/**
 * @brief The adc abstract layer
 *
 * @file adc.c
 * @date 2018-06-22
 * @author Dengjian
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "semphr.h"
#include "nrfx_saadc.h"
#include "nrfx_log.h"
#include "hal_adc.h"

/* Defines -------------------------------------------------------------------*/
#define LOCK_TIMEOUT          pdMS_TO_TICKS(100)
#define CONVERT_TIMEOUT       pdMS_TO_TICKS(10)
#define CALIBRATE_TIMEOUT     pdMS_TO_TICKS(10)

/* Typedefs ------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static int8_t _nb_channels = 0;
static int8_t _oversample = 0;
static int8_t _resolution = 0;
static int16_t _sample[NRF_SAADC_CHANNEL_COUNT] = { 0 };

/* static StaticSemaphore_t lock_memory = { 0 }; */
/* static SemaphoreHandle_t lock = NULL; */
static SemaphoreHandle_t sample_semphr = NULL;
static SemaphoreHandle_t cali_semphr = NULL;

/* Private functions ---------------------------------------------------------*/
/* static bool query_mode(int channel) { */
/*   bool diff = NRF_SAADC->CH[channel].CONFIG & SAADC_CH_CONFIG_MODE_Msk; */

/*   return diff; */
/* } */

static float query_gain(int channel) {
  float gain = -1;

  switch((NRF_SAADC->CH[channel].CONFIG & SAADC_CH_CONFIG_GAIN_Msk) >> SAADC_CH_CONFIG_GAIN_Pos) {
    case NRF_SAADC_GAIN1_6:
      gain = 1 / 6.0;
      break;
    case NRF_SAADC_GAIN1_5:
      gain = 1 / 5.0;
      break;
    case NRF_SAADC_GAIN1_4:
      gain = 1 / 4.0;
      break;
    case NRF_SAADC_GAIN1_3:
      gain = 1 / 3.0;
      break;
    case NRF_SAADC_GAIN1_2:
      gain = 1 / 2.0;
      break;
    case NRF_SAADC_GAIN1:
      gain = 1;
      break;
    case NRF_SAADC_GAIN2:
      gain = 2;
      break;
    case NRF_SAADC_GAIN4:
      gain = 4;
      break;
    default:
      break;
  }

  return gain;
}

static nrf_saadc_oversample_t get_oversample(int osr) {
  nrf_saadc_input_t oversample = -1;

  switch(osr) {
    case 1:
      oversample = NRF_SAADC_OVERSAMPLE_DISABLED;
      break;
    case 2:
      oversample = NRF_SAADC_OVERSAMPLE_2X;
      break;
    case 4:
      oversample = NRF_SAADC_OVERSAMPLE_4X;
      break;
    case 8:
      oversample = NRF_SAADC_OVERSAMPLE_8X;
      break;
    case 16:
      oversample = NRF_SAADC_OVERSAMPLE_16X;
      break;
    case 32:
      oversample = NRF_SAADC_OVERSAMPLE_32X;
      break;
    case 64:
      oversample = NRF_SAADC_OVERSAMPLE_64X;
      break;
    case 128:
      oversample = NRF_SAADC_OVERSAMPLE_128X;
      break;
    case 256:
      oversample = NRF_SAADC_OVERSAMPLE_256X;
      break;
    default:
      break;
  }

  return oversample;
}

static nrf_saadc_input_t get_input(hal_pin_t pin) {
  nrf_saadc_input_t input = -1;

  switch(pin) {
    case HAL_PIN_NOT_USED:
      input = NRF_SAADC_INPUT_DISABLED;
      break;
    case HAL_PIN_ADC_VDD:
      input = NRF_SAADC_INPUT_VDD;
      break;
#ifdef NRF52840_XXAA
    case HAL_PIN_ADC_VDDHDIV5:
      input = SAADC_CH_PSELP_PSELP_VDDHDIV5;
      break;
#endif
    case 2:
      input = NRF_SAADC_INPUT_AIN0;
      break;
    case 3:
      input = NRF_SAADC_INPUT_AIN1;
      break;
    case 4:
      input = NRF_SAADC_INPUT_AIN2;
      break;
    case 5:
      input = NRF_SAADC_INPUT_AIN3;
      break;
    case 28:
      input = NRF_SAADC_INPUT_AIN4;
      break;
    case 29:
      input = NRF_SAADC_INPUT_AIN5;
      break;
    case 30:
      input = NRF_SAADC_INPUT_AIN6;
      break;
    case 31:
      input = NRF_SAADC_INPUT_AIN7;
      break;
    default:
      break;
  }

  return input;
}

static nrf_saadc_resolution_t get_resolution(int bits) {
  nrf_saadc_resolution_t resolution = -1;

  switch(bits) {
    case 8:
      resolution = NRF_SAADC_RESOLUTION_8BIT;
      break;
    case 10:
      resolution = NRF_SAADC_RESOLUTION_10BIT;
      break;
    case 12:
      resolution = NRF_SAADC_RESOLUTION_12BIT;
      break;
    default:
      break;
  }

  return resolution;
}

static nrf_saadc_acqtime_t get_acqtime(int sample_time) {
  nrf_saadc_acqtime_t acqtime = -1;

  if(sample_time <= 3) {
    acqtime = NRF_SAADC_ACQTIME_3US;
  } else if(sample_time <= 5) {
    acqtime = NRF_SAADC_ACQTIME_5US;
  } else if(sample_time <= 10) {
    acqtime = NRF_SAADC_ACQTIME_10US;
  } else if(sample_time <= 15) {
    acqtime = NRF_SAADC_ACQTIME_15US;
  } else if(sample_time <= 20) {
    acqtime = NRF_SAADC_ACQTIME_20US;
  } else if(sample_time <= 40) {
    acqtime = NRF_SAADC_ACQTIME_40US;
  }

  return acqtime;
}

static nrf_saadc_gain_t get_gain(float voltage_gain) {
  nrf_saadc_gain_t gain = -1;

  if(voltage_gain >= 4) {
    gain = NRF_SAADC_GAIN4;
  } else if(voltage_gain >= 2) {
    gain = NRF_SAADC_GAIN2;
  } else if(voltage_gain >= 1) {
    gain = NRF_SAADC_GAIN1;
  } else if(voltage_gain >= 1 / 2.0) {
    gain = NRF_SAADC_GAIN1_2;
  } else if(voltage_gain >= 1 / 3.0) {
    gain = NRF_SAADC_GAIN1_3;
  } else if(voltage_gain >= 1 / 4.0) {
    gain = NRF_SAADC_GAIN1_4;
  } else if(voltage_gain >= 1 / 5.0) {
    gain = NRF_SAADC_GAIN1_5;
  } else if(voltage_gain >= 1 / 6.0) {
    gain = NRF_SAADC_GAIN1_6;
  }

  return gain;
}

static hal_err_t adc_init_channel(int channel, hal_adc_channel_t* cfg) {
  if(cfg->reference != HAL_ADC_REFERENCE_INT) {
    return HAL_ERR_NOT_SUPPORT;
  }

  nrf_saadc_acqtime_t acqtime = get_acqtime(cfg->acqtime);
  nrf_saadc_gain_t gain = get_gain(cfg->gain);
  if(acqtime == -1 || gain == -1) {
    return HAL_ERR_PARAM;
  }

  nrf_saadc_input_t pin_p = get_input(cfg->pin_p);
  nrf_saadc_input_t pin_n = get_input(cfg->pin_n);
  if(pin_p == -1 || pin_n == -1 || pin_p == NRF_SAADC_INPUT_DISABLED) {
    return HAL_ERR_PARAM;
  }

  nrf_saadc_channel_config_t nrf_cfg = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(pin_p);
  nrf_cfg.reference = NRF_SAADC_REFERENCE_INTERNAL;
  nrf_cfg.acq_time = acqtime;
  nrf_cfg.burst = _oversample > 1 ? NRF_SAADC_BURST_ENABLED : NRF_SAADC_BURST_DISABLED;
  nrf_cfg.gain = gain;

  if(cfg->pin_n != HAL_PIN_NOT_USED) {
    nrf_cfg.mode = NRF_SAADC_MODE_DIFFERENTIAL;
    nrf_cfg.pin_n = pin_n;
  }

  if(nrfx_saadc_channel_init(channel, &nrf_cfg) != NRFX_SUCCESS) {
    return HAL_ERR_UNLIKELY;
  }

  return HAL_ERR_OK;
}

static void adc_callback(nrfx_saadc_evt_t const *p_event) {
  if(!p_event) {
    return;
  }

  if(p_event->type == NRFX_SAADC_EVT_CALIBRATEDONE) {
    if(cali_semphr) {
      xSemaphoreGiveFromISR(cali_semphr, NULL);
    }
  } else if(p_event->type == NRFX_SAADC_EVT_DONE) {
    int16_t* buf = p_event->data.done.p_buffer;
    int16_t size = p_event->data.done.size;
    if(buf && size == _nb_channels) {
      for(int ch = 0; ch < _nb_channels; ch++) {
        _sample[ch] = buf[ch];
      }
    }
    if(sample_semphr) {
      xSemaphoreGiveFromISR(sample_semphr, NULL);
    }
  }
}

/* Global functions ----------------------------------------------------------*/

/**
 * @brief Init
 *
 * @param cfg Adc configuration
 * @param channel Channel configuration
 * @param nb_channels Number of channels
 * @param osr Oversample rate
 *
 * @return HAL_ERR_OK if success
 */
hal_err_t hal_adc_init(hal_adc_cfg_t* cfg, hal_adc_channel_t* channel, int nb_channels) {
  if(_nb_channels != 0) {
    return HAL_ERR_ALREADY_INITIALIZED;
  }

  if(!channel || !nb_channels) {
    return HAL_ERR_PARAM;
  }

  nrf_saadc_resolution_t resolution = get_resolution(cfg->resolution);
  if(resolution == -1) {
    return HAL_ERR_PARAM;
  }
  nrf_saadc_oversample_t oversample = get_oversample(cfg->oversample);
  if(oversample == -1) {
    return HAL_ERR_PARAM;
  }

  nrfx_saadc_config_t nrf_cfg = NRFX_SAADC_DEFAULT_CONFIG;
  nrf_cfg.oversample = oversample;
  nrf_cfg.resolution = resolution;
  nrf_cfg.low_power_mode = true;

  if(nrfx_saadc_init(&nrf_cfg, adc_callback) != NRFX_SUCCESS) {
    return HAL_ERR_UNLIKELY;
  }

  _oversample = cfg->oversample;
  _resolution = cfg->resolution;

  for(int i = 0; i < nb_channels; i++) {
    if(adc_init_channel(i, channel++) != HAL_ERR_OK) {
      nrfx_saadc_uninit();
      return HAL_ERR_UNLIKELY;
    }
  }

  _nb_channels = nb_channels;

  return HAL_ERR_OK;
}

/**
 * @brief Deinit
 *
 * @return HAL_ERR_OK if success
 */
hal_err_t hal_adc_deinit(void) {
  if(_nb_channels == 0) {
    return HAL_ERR_NOT_INITIALIZED;
  }

  nrfx_saadc_uninit();
  _nb_channels = 0;

  return HAL_ERR_OK;
}

/**
 * @brief Update data
 *
 * @return HAL_ERR_OK if success
 */
hal_err_t hal_adc_update(void) {
  if(_nb_channels == 0) {
    return HAL_ERR_NOT_INITIALIZED;
  }

  sample_semphr = xSemaphoreCreateBinary();
  if(!sample_semphr) {
    return HAL_ERR_MEMORY;
  }

  int buf_size = _nb_channels;
  int16_t* buf = pvPortMalloc(buf_size * sizeof(int16_t));
  if(!buf) {
    vPortFree(sample_semphr);
    return HAL_ERR_MEMORY;
  }

  if(nrfx_saadc_buffer_convert(buf, buf_size) != NRFX_SUCCESS) {
    vPortFree(sample_semphr);
    vPortFree(buf);
    return HAL_ERR_UNLIKELY;
  }

  if(nrfx_saadc_sample() != NRFX_SUCCESS) {
    vPortFree(sample_semphr);
    vPortFree(buf);
    return HAL_ERR_UNLIKELY;
  }

  if(!xSemaphoreTake(sample_semphr, CONVERT_TIMEOUT)) {
    nrfx_saadc_abort();
    vPortFree(sample_semphr);
    vPortFree(buf);
    return HAL_ERR_TIMEOUT;
  }

  vPortFree(sample_semphr);
  vPortFree(buf);

  return HAL_ERR_OK;
}

/**
 * @brief Read voltage
 *
 * @param channel Channel number to read
 * @param output Pointer to store voltage
 *
 * @return HAL_ERR_OK if success
 */
hal_err_t hal_adc_get_voltage(int channel, float* output) {
  if(_nb_channels == 0) {
    return HAL_ERR_NOT_INITIALIZED;
  }

  if(channel < 0 || channel >= _nb_channels || !output) {
    return HAL_ERR_PARAM;
  }

  *output = 0.6 * _sample[channel] / (1 << _resolution) / query_gain(channel);

  return HAL_ERR_OK;
}

/**
 * @brief Read data
 *
 * @param channel Channel number to read
 * @param output Pointer to store data
 *
 * @return HAL_ERR_OK if success
 */
hal_err_t hal_adc_get_sample(int channel, int16_t* output) {
  if(_nb_channels == 0) {
    return HAL_ERR_NOT_INITIALIZED;
  }

  if(channel < 0 || channel >= _nb_channels || !output) {
    return HAL_ERR_PARAM;
  }

  *output = _sample[channel];

  return HAL_ERR_OK;
}

/**
 * @brief Calibrate
 *
 * @param cali
 *
 * @return HAL_ERR_OK if success
 */
hal_err_t hal_adc_calibrate(void* cali) {
  if(_nb_channels == 0) {
    return HAL_ERR_NOT_INITIALIZED;
  }

  cali_semphr = xSemaphoreCreateBinary();
  if(!cali_semphr) {
    return HAL_ERR_MEMORY;
  }

  if(nrfx_saadc_calibrate_offset() != NRFX_SUCCESS) {
    return HAL_ERR_UNLIKELY;
  }

  if(!xSemaphoreTake(cali_semphr, CALIBRATE_TIMEOUT)) {
    nrfx_saadc_abort();
    vPortFree(cali_semphr);
    return HAL_ERR_TIMEOUT;
  }

  vPortFree(cali_semphr);
  return HAL_ERR_OK;
}


