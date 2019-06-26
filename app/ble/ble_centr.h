/**
 * @brief 
 * 
 * @file ble_centr.h
 * @date 2018-09-27
 * @author Chenfei
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */


#ifndef BLE_CENTR_H__
#define BLE_CENTR_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_gatt.h"
#include "ble_db_discovery.h"
#include "nrf_sdh_ble.h"

#include "sdk_config.h"
#include "hal.h"
#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_centr instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_CENTR_DEF(_name)                                                                        \
static ble_centr_t _name;                                                                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_CENTR_BLE_OBSERVER_PRIO,                                                   \
                     ble_centr_on_ble_evt, &_name)

/** @brief Macro for defining multiple ble_centr instances.
 *
 * @param   _name   Name of the array of instances.
 * @param   _cnt    Number of instances to define.
 * @hideinitializer
 */
#define BLE_CENTR_ARRAY_DEF(_name, _cnt)                 \
static ble_centr_t _name[_cnt];                          \
NRF_SDH_BLE_OBSERVERS(_name ## _obs,                     \
                      BLE_CENTR_BLE_OBSERVER_PRIO,       \
                      ble_centr_on_ble_evt, &_name, _cnt)

#define CENTR_BASE_UUID                  {{0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x04, 0x01}}
//#define CENTR_BASE_UUID                  {{0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E}}

#define BLE_UUID_CENTR_SERVICE            0x0001                      /**< The UUID of the Nordic UART Service. */
#define BLE_UUID_CENTR_RX_CHARACTERISTIC  0x0002                      /**< The UUID of the RX Characteristic. */
#define BLE_UUID_CENTR_TX_CHARACTERISTIC  0x0003                      /**< The UUID of the TX Characteristic. */

#define OPCODE_LENGTH 1
#define HANDLE_LENGTH 2

/**@brief   Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_CENTR_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
#else
    #define BLE_CENTR_MAX_DATA_LEN (BLE_GATT_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif


/**@brief CENTR Client event type. */
typedef enum
{
    BLE_CENTR_EVT_DISCOVERY_COMPLETE,   /**< Event indicating that the CENTR service and its characteristics was found. */
    BLE_CENTR_EVT_CENTR_TX_EVT,           /**< Event indicating that the central has received something from a peer. */
    BLE_CENTR_EVT_DISCONNECTED          /**< Event indicating that the CENTR server has disconnected. */
} ble_centr_evt_type_t;

/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
    uint16_t centr_tx_handle;      /**< Handle of the CENTR TX characteristic as provided by a discovery. */
    uint16_t centr_tx_cccd_handle; /**< Handle of the CCCD of the CENTR TX characteristic as provided by a discovery. */
    uint16_t centr_rx_handle;      /**< Handle of the CENTR RX characteristic as provided by a discovery. */
} ble_centr_handles_t;

/**@brief Structure containing the CENTR event data received from the peer. */
typedef struct
{
    ble_centr_evt_type_t evt_type;
    uint16_t             conn_handle;
    uint16_t             max_data_len;
    uint8_t            * p_data;
    uint8_t              data_len;
    ble_centr_handles_t  handles;     /**< Handles on which the Nordic Uart service characteristics was discovered on the peer device. This will be filled if the evt_type is @ref BLE_CENTR_EVT_DISCOVERY_COMPLETE.*/
} ble_centr_evt_t;

// Forward declaration of the ble_centr_t type.
typedef struct ble_centr_s ble_centr_t;

/**@brief   Event handler type.
 *
 * @details This is the type of the event handler that should be provided by the application
 *          of this module to receive events.
 */
typedef void (* ble_centr_evt_handler_t)(ble_centr_t * p_ble_centr, ble_centr_evt_t const * p_evt);

/**@brief CENTR Client structure. */
struct ble_centr_s
{
    uint8_t                 uuid_type;      /**< UUID type. */
    uint16_t                conn_handle;    /**< Handle of the current connection. Set with @ref ble_centr_handles_assign when connected. */
    ble_centr_handles_t     handles;        /**< Handles on the connected peer device needed to interact with it. */
    ble_centr_evt_handler_t evt_handler;    /**< Application event handler to be called when there is an event related to the CENTR. */
};

/**@brief CENTR Client initialization structure. */
typedef struct
{
    ble_centr_evt_handler_t evt_handler;
} ble_centr_init_t;


/**@brief     Function for initializing the Nordic UART client module.
 *
 * @details   This function registers with the Database Discovery module
 *            for the CENTR. Doing so will make the Database Discovery
 *            module look for the presence of a CENTR instance at the peer when a
 *            discovery is started.
 *
 * @param[in] p_ble_centr      Pointer to the CENTR client structure.
 * @param[in] p_ble_centr_init Pointer to the CENTR initialization structure containing the
 *                             initialization information.
 *
 * @retval    NRF_SUCCESS If the module was initialized successfully. Otherwise, an error
 *                        code is returned. This function
 *                        propagates the error code returned by the Database Discovery module API
 *                        @ref ble_db_discovery_evt_register.
 */
uint32_t ble_centr_init(ble_centr_t * p_ble_centr, ble_centr_init_t * p_ble_centr_init);


/**@brief Function for handling events from the database discovery module.
 *
 * @details This function will handle an event from the database discovery module, and determine
 *          if it relates to the discovery of CENTR at the peer. If so, it will
 *          call the application's event handler indicating that CENTR has been
 *          discovered at the peer. It also populates the event with the service related
 *          information before providing it to the application.
 *
 * @param[in] p_ble_centr Pointer to the CENTR client structure.
 * @param[in] p_evt       Pointer to the event received from the database discovery module.
 */
 void ble_centr_on_db_disc_evt(ble_centr_t * p_ble_centr, ble_db_discovery_evt_t * p_evt);


/**@brief     Function for handling BLE events from the SoftDevice.
 *
 * @details   This function handles the BLE events received from the SoftDevice. If a BLE
 *            event is relevant to the CENTR module, it is used to update
 *            internal variables and, if necessary, send events to the application.
 *
 * @param[in] p_ble_evt     Pointer to the BLE event.
 * @param[in] p_context     Pointer to the CENTR client structure.
 */
void ble_centr_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief   Function for requesting the peer to start sending notification of TX characteristic.
 *
 * @details This function enables notifications of the CENTR TX characteristic at the peer
 *          by writing to the CCCD of the CENTR TX characteristic.
 *
 * @param   p_ble_centr Pointer to the CENTR client structure.
 *
 * @retval  NRF_SUCCESS If the SoftDevice has been requested to write to the CCCD of the peer.
 *                      Otherwise, an error code is returned. This function propagates the error
 *                      code returned by the SoftDevice API @ref sd_ble_gattc_write.
 */
uint32_t ble_centr_tx_notif_enable(ble_centr_t * p_ble_centr);


/**@brief Function for sending a string to the server.
 *
 * @details This function writes the RX characteristic of the server.
 *
 * @param[in] p_ble_centr Pointer to the CENTR client structure.
 * @param[in] p_string    String to be sent.
 * @param[in] length      Length of the string.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_centr_send(ble_centr_t * p_ble_centr, uint8_t * p_string, uint16_t length);


/**@brief Function for assigning handles to a this instance of centr.
 *
 * @details Call this function when a link has been established with a peer to
 *          associate this link to this instance of the module. This makes it
 *          possible to handle several link and associate each link to a particular
 *          instance of this module. The connection handle and attribute handles will be
 *          provided from the discovery event @ref BLE_CENTR_EVT_DISCOVERY_COMPLETE.
 *
 * @param[in] p_ble_centr    Pointer to the CENTR client structure instance to associate with these
 *                           handles.
 * @param[in] conn_handle    Connection handle to associated with the given CENTR Instance.
 * @param[in] p_peer_handles Attribute handles on the CENTR server that you want this CENTR client to
 *                           interact with.
 *
 * @retval    NRF_SUCCESS    If the operation was successful.
 * @retval    NRF_ERROR_NULL If a p_centr was a NULL pointer.
 */
uint32_t ble_centr_handles_assign(ble_centr_t *               p_ble_centr,
                                  uint16_t                    conn_handle,
                                  ble_centr_handles_t const * p_peer_handles);


#ifdef __cplusplus
}
#endif

#endif // BLE_CENTR_H__

/** @} */
