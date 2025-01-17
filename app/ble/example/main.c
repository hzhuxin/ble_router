/**
 * @brief project for ble-router, unviarnished transmission
 * 
 * @file main.c
 * @date 2018-10-08
 * @author Chenfei
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */
#include "sdk_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "nrf_drv_clock.h"
#include "nrf_pwr_mgmt.h"
#include "app_timer.h"
#include "debug.h"
#include "ble_init.h"
#include "router_init.h"
#include "hal_cpu.h"
#include "hal_rtc.h"
#include "hal_wdt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_bootloader_info.h"
#include "record.h"

#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread;                                /**< Definition of Logger thread. */
#endif

/**@brief Function for initializing the clock.
 */
static void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Callback for watchdog interrupt
 */
static void wdt_cb(void) {
  hal_rtc_get_time();
  hal_cpu_reset();
}
/**@brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}
#if NRF_LOG_ENABLED

static uint32_t get_log_timestamp(void) 
{
  return xTaskGetTickCount();
}
/**@brief Function for initializing logging.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(get_log_timestamp);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Thread for handling the logger.
 *
 * @details This thread is responsible for processing log entries if logs are deferred.
 *          Thread flushes all log entries and suspends. It is resumed by idle task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void logger_thread(void * arg)
{
    UNUSED_PARAMETER(arg);

    while (1)
    {
        NRF_LOG_FLUSH();

        vTaskSuspend(NULL); // Suspend myself
    }
}
#endif //NRF_LOG_ENABLED

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook( void )
{
#if NRF_LOG_ENABLED
     vTaskResume(m_logger_thread);
#endif
}
/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;
    hal_cpu_set_voltage_mode(HAL_CPU_VOLTAGE_REGULATOR_LDO,HAL_CPU_VOLTAGE_1V8);
    // Initialize.   
    clock_init();
    timer_init();

    // hal_cpu_get_reset_reason();
    // hal_wdt_cfg_t cfg = {
    //   .timeout = 12000,
    //   .stop_when_sleep = 0,
    //   .stop_when_debug = 0,
    // };

    // hal_wdt_init(&cfg, wdt_cb);
    hal_rtc_init();

#if NRF_LOG_ENABLED
    log_init();
    // Start execution.
    if (pdPASS != xTaskCreate(logger_thread, "LOGGER", 256, NULL, 1, &m_logger_thread))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
#endif

    creat_init_task();
    // Activate deep sleep mode.
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    power_management_init();
    ble_init(&erase_bonds);   
    // Start FreeRTOS scheduler.
    vTaskStartScheduler();
    // Enter main loop.
    for (;;)
    {
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
}

#if configSUPPORT_STATIC_ALLOCATION
/* static memory allocation for the IDLE task */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[256];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = sizeof(xIdleStack) / sizeof(xIdleStack[0]);
}
#endif

#if configSUPPORT_STATIC_ALLOCATION && configUSE_TIMERS
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

/* If static allocation is supported then the application must provide the
 *    following callback function - which enables the application to optionally
 *       provide the memory that will be used by the timer task as the task's stack
 *          and TCB. */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize) {
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
#endif

void vApplicationMallocFailedHook(void) {
  NRF_LOG_ERROR("!!!! Heap empty(%d) !!!", xPortGetFreeHeapSize());
  NRF_LOG_FLUSH();

  Record_Error(hal_rtc_get_time(), RT_ERR_HEAP, xPortGetFreeHeapSize());
  hal_cpu_reset();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName) {
  NRF_LOG_ERROR("!!!! Stack overflow !!! %s @ %p", pcTaskName, xTask);
  NRF_LOG_FLUSH();

  uint32_t value = 0;
  if(pcTaskName != NULL) {
    for(int i = 0; i < 4; i++) {
      if(*pcTaskName == '\0') {
        break;
      }
      value = (value << 8) | *pcTaskName++;
    }
  }
  if(value == 0) {
    value = (uint32_t)xTask;
  }

  Record_Error(hal_rtc_get_time(), RT_ERR_STACK, value);
  hal_cpu_reset();
}
