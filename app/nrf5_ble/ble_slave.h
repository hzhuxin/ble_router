 /**
 * @brief 
 * 
 * @file ble_slave.h
 * @date 2018-09-27
 * @author Chenfei
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */
#ifndef BLE_SLAVE_H__
#define BLE_SLAVE_H__

#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_link_ctx_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_slv instance.
 *
 * @param     _name            Name of the instance.
 * @param[in] _slv_max_clients Maximum number of SLV clients connected at a time.
 * @hideinitializer
 */
#define BLE_SLV_DEF(_name, _slv_max_clients)                      \
    BLE_LINK_CTX_MANAGER_DEF(CONCAT_2(_name, _link_ctx_storage),  \
                             (_slv_max_clients),                  \
                             sizeof(ble_slv_client_context_t));   \
    static ble_slv_t _name =                                      \
    {                                                             \
        .p_link_ctx_storage = &CONCAT_2(_name, _link_ctx_storage) \
    };                                                            \
    NRF_SDH_BLE_OBSERVER(_name ## _obs,                           \
                         BLE_SLV_BLE_OBSERVER_PRIO,               \
                         ble_slv_on_ble_evt,                      \
                         &_name)

#define SLV_BASE_UUID                  {{0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E}} /**< Used vendor specific UUID. */
//#define SLV_BASE_UUID                   {{0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00 ,0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}} /**< Used vendor specific UUID. */

#define BLE_UUID_SLV_SERVICE 0x0001 /**< The UUID of the Nordic UART Service. */
#define BLE_UUID_SLV_TX_CHARACTERISTIC 0x0002               /**< The UUID of the TX Characteristic. */
#define BLE_UUID_SLV_RX_CHARACTERISTIC 0x0003               /**< The UUID of the RX Characteristic. */

#define BLE_SLV_MAX_RX_CHAR_LEN        BLE_SLV_MAX_DATA_LEN /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_SLV_MAX_TX_CHAR_LEN        BLE_SLV_MAX_DATA_LEN /**< Maximum length of the TX Characteristic (in bytes). */


#define OPCODE_LENGTH        1
#define HANDLE_LENGTH        2

/**@brief   Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_SLV_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
#else
    #define BLE_SLV_MAX_DATA_LEN (BLE_GATT_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif


/**@brief   Nordic UART Service event types. */
typedef enum
{
    BLE_SLV_EVT_RX_DATA,      /**< Data received. */
    BLE_SLV_EVT_TX_RDY,       /**< Service is ready to accept new data to be transmitted. */
    BLE_SLV_EVT_COMM_STARTED, /**< Notification has been enabled. */
    BLE_SLV_EVT_COMM_STOPPED, /**< Notification has been disabled. */
} ble_slv_evt_type_t;


/* Forward declaration of the ble_slv_t type. */
typedef struct ble_slv_s ble_slv_t;


/**@brief   Nordic UART Service @ref BLE_SLV_EVT_RX_DATA event data.
 *
 * @details This structure is passed to an event when @ref BLE_SLV_EVT_RX_DATA occurs.
 */
typedef struct
{
    uint8_t const * p_data; /**< A pointer to the buffer with received data. */
    uint16_t        length; /**< Length of received data. */
} ble_slv_evt_rx_data_t;


/**@brief Nordic UART Service client context structure.
 *
 * @details This structure contains state context related to hosts.
 */
typedef struct
{
    bool is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
} ble_slv_client_context_t;


/**@brief   Nordic UART Service event structure.
 *
 * @details This structure is passed to an event coming from service.
 */
typedef struct
{
    ble_slv_evt_type_t         type;        /**< Event type. */
    ble_slv_t                * p_slv;       /**< A pointer to the instance. */
    uint16_t                   conn_handle; /**< Connection handle. */
    ble_slv_client_context_t * p_link_ctx;  /**< A pointer to the link context. */
    union
    {
        ble_slv_evt_rx_data_t rx_data; /**< @ref BLE_SLV_EVT_RX_DATA event data. */
    } params;
} ble_slv_evt_t;


/**@brief Nordic UART Service event handler type. */
typedef void (* ble_slv_data_handler_t) (ble_slv_evt_t * p_evt);


/**@brief   Nordic UART Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_slv_init
 *          function.
 */
typedef struct
{
    ble_slv_data_handler_t data_handler; /**< Event handler to be called for handling received data. */
} ble_slv_init_t;


/**@brief   Nordic UART Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_slv_s
{
    uint8_t                         uuid_type;          /**< UUID type for Nordic UART Service Base UUID. */
    uint16_t                        service_handle;     /**< Handle of Nordic UART Service (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        tx_handles;         /**< Handles related to the TX characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        rx_handles;         /**< Handles related to the RX characteristic (as provided by the SoftDevice). */
    blcm_link_ctx_storage_t * const p_link_ctx_storage; /**< Pointer to link context storage with handles of all current connections and its context. */
    ble_slv_data_handler_t          data_handler;       /**< Event handler to be called for handling received data. */
};


/**@brief   Function for initializing the Nordic UART Service.
 *
 * @param[out] p_slv      Nordic UART Service structure. This structure must be supplied
 *                        by the application. It is initialized by this function and will
 *                        later be used to identify this particular service instance.
 * @param[in] p_slv_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_slv or p_slv_init is NULL.
 */
uint32_t ble_slv_init(ble_slv_t * p_slv, ble_slv_init_t const * p_slv_init);


/**@brief   Function for handling the Nordic UART Service's BLE events.
 *
 * @details The Nordic UART Service expects the application to call this function each time an
 * event is received from the SoftDevice. This function processes the event if it
 * is relevant and calls the Nordic UART Service event handler of the
 * application if necessary.
 *
 * @param[in] p_ble_evt     Event received from the SoftDevice.
 * @param[in] p_context     Nordic UART Service structure.
 */
void ble_slv_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief   Function for sending a data to the peer.
 *
 * @details This function sends the input string as an RX characteristic notification to the
 *          peer.
 *
 * @param[in]     p_slv       Pointer to the Nordic UART Service structure.
 * @param[in]     p_data      String to be sent.
 * @param[in,out] p_length    Pointer Length of the string. Amount of sent bytes.
 * @param[in]     conn_handle Connection Handle of the destination client.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_slv_data_send(ble_slv_t * p_slv,
                           uint8_t   * p_data,
                           uint16_t  * p_length,
                           uint16_t    conn_handle);


#ifdef __cplusplus
}
#endif

#endif // BLE_SLAVE_H__

/** @} */
