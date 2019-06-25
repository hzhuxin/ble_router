/**
 * @brief 
 * 
 * @file ble_centr.c
 * @date 2018-08-30
 * @author Chenfei
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_CENTR)
#include <stdlib.h>
#include "app_freertos.h"
#include "ble.h"
#include "ble_centr.h"
#include "ble_gattc.h"
#include "ble_srv_common.h"
#include "app_error.h"
//#include "ble_init.h"
#include "nrf_log.h"

//static uint8_t *ble_rx_buf = NULL;
//static ble_data_trans_head_t* tx_buf = NULL;
void ble_centr_on_db_disc_evt(ble_centr_t * p_ble_centr, ble_db_discovery_evt_t * p_evt)
{
    ble_centr_evt_t centr_evt;
    memset(&centr_evt,0,sizeof(ble_centr_evt_t));

    ble_gatt_db_char_t * p_chars = p_evt->params.discovered_db.charateristics;

    // Check if the CENTR was discovered.
    if (    (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE)
        &&  (p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_CENTR_SERVICE)
        &&  (p_evt->params.discovered_db.srv_uuid.type == p_ble_centr->uuid_type))
    {
        for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            switch (p_chars[i].characteristic.uuid.uuid)
            {
                case BLE_UUID_CENTR_RX_CHARACTERISTIC:
                    centr_evt.handles.centr_rx_handle = p_chars[i].characteristic.handle_value;
                    break;

                case BLE_UUID_CENTR_TX_CHARACTERISTIC:
                    centr_evt.handles.centr_tx_handle = p_chars[i].characteristic.handle_value;
                    centr_evt.handles.centr_tx_cccd_handle = p_chars[i].cccd_handle;
                    break;

                default:
                    break;
            }
        }
        if (p_ble_centr->evt_handler != NULL)
        {
            centr_evt.conn_handle = p_evt->conn_handle;
            centr_evt.evt_type    = BLE_CENTR_EVT_DISCOVERY_COMPLETE;
            p_ble_centr->evt_handler(p_ble_centr, &centr_evt);
        }
    }
}

/**@brief     Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @details   This function will uses the Handle Value Notification received from the SoftDevice
 *            and checks if it is a notification of the CENTR TX characteristic from the peer. If
 *            it is, this function will decode the data and send it to the
 *            application.
 *
 * @param[in] p_ble_centr Pointer to the CENTR Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_hvx(ble_centr_t * p_ble_centr, ble_evt_t const * p_ble_evt)
{
    if (   (p_ble_centr->handles.centr_tx_handle != BLE_GATT_HANDLE_INVALID)
        && (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_centr->handles.centr_tx_handle)
        && (p_ble_centr->evt_handler != NULL))
    {
        
        ble_centr_evt_t ble_centr_evt;

        ble_centr_evt.evt_type = BLE_CENTR_EVT_CENTR_TX_EVT;
        ble_centr_evt.p_data   = (uint8_t *)p_ble_evt->evt.gattc_evt.params.hvx.data;
        ble_centr_evt.data_len = p_ble_evt->evt.gattc_evt.params.hvx.len;

        p_ble_centr->evt_handler(p_ble_centr, &ble_centr_evt);
        //NRF_LOG_DEBUG("Client sending data.");
    }
}

uint32_t ble_centr_init(ble_centr_t * p_ble_centr, ble_centr_init_t * p_ble_centr_init)
{
    uint32_t      err_code;
    ble_uuid_t    uart_uuid;
    ble_uuid128_t centr_base_uuid = CENTR_BASE_UUID;

    VERIFY_PARAM_NOT_NULL(p_ble_centr);
    VERIFY_PARAM_NOT_NULL(p_ble_centr_init);

    err_code = sd_ble_uuid_vs_add(&centr_base_uuid, &p_ble_centr->uuid_type);
    VERIFY_SUCCESS(err_code);

    uart_uuid.type = p_ble_centr->uuid_type;
    uart_uuid.uuid = BLE_UUID_CENTR_SERVICE;

    p_ble_centr->conn_handle           = BLE_CONN_HANDLE_INVALID;
    p_ble_centr->evt_handler           = p_ble_centr_init->evt_handler;
    p_ble_centr->handles.centr_tx_handle = BLE_GATT_HANDLE_INVALID;
    p_ble_centr->handles.centr_rx_handle = BLE_GATT_HANDLE_INVALID;

    return ble_db_discovery_evt_register(&uart_uuid);
}

void ble_centr_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_centr_t * p_ble_centr = (ble_centr_t *)p_context;

    if ((p_ble_centr == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    if ( (p_ble_centr->conn_handle != BLE_CONN_HANDLE_INVALID)
       &&(p_ble_centr->conn_handle != p_ble_evt->evt.gap_evt.conn_handle)
       )
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_HVX:
            on_hvx(p_ble_centr, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            if (p_ble_evt->evt.gap_evt.conn_handle == p_ble_centr->conn_handle
                    && p_ble_centr->evt_handler != NULL)
            {
                ble_centr_evt_t centr_evt;

                centr_evt.evt_type = BLE_CENTR_EVT_DISCONNECTED;

                p_ble_centr->conn_handle = BLE_CONN_HANDLE_INVALID;
                p_ble_centr->evt_handler(p_ble_centr, &centr_evt);
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for creating a message for writing to the CCCD. */
static uint32_t cccd_configure(uint16_t conn_handle, uint16_t cccd_handle, bool enable)
{
    uint8_t buf[BLE_CCCD_VALUE_LEN];

    buf[0] = enable ? BLE_GATT_HVX_NOTIFICATION : 0;
    buf[1] = 0;

    ble_gattc_write_params_t const write_params =
    {
        .write_op = BLE_GATT_OP_WRITE_REQ,
        .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .handle   = cccd_handle,
        .offset   = 0,
        .len      = sizeof(buf),
        .p_value  = buf
    };

    return sd_ble_gattc_write(conn_handle, &write_params);
}


uint32_t ble_centr_tx_notif_enable(ble_centr_t * p_ble_centr)
{
    VERIFY_PARAM_NOT_NULL(p_ble_centr);

    if ( (p_ble_centr->conn_handle == BLE_CONN_HANDLE_INVALID)
       ||(p_ble_centr->handles.centr_tx_cccd_handle == BLE_GATT_HANDLE_INVALID)
       )
    {
        return NRF_ERROR_INVALID_STATE;
    }
    return cccd_configure(p_ble_centr->conn_handle,p_ble_centr->handles.centr_tx_cccd_handle, true);
}


uint32_t ble_centr_send(ble_centr_t * p_ble_centr, uint8_t * p_data, uint16_t length)
{
    VERIFY_PARAM_NOT_NULL(p_ble_centr);

    if (length > BLE_CENTR_MAX_DATA_LEN)
    {
        NRF_LOG_WARNING("Content too long.");
        return NRF_ERROR_INVALID_PARAM;
    }
    if (p_ble_centr->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        NRF_LOG_WARNING("Connection handle invalid.");
        return NRF_ERROR_INVALID_STATE;
    }

    ble_gattc_write_params_t const write_params =
    {
        .write_op = BLE_GATT_OP_WRITE_CMD,
        .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .handle   = p_ble_centr->handles.centr_rx_handle,
        .offset   = 0,
        .len      = length,
        .p_value  = p_data
    };

    return sd_ble_gattc_write(p_ble_centr->conn_handle, &write_params);
}


uint32_t ble_centr_handles_assign(ble_centr_t               * p_ble_centr,
                                  uint16_t                    conn_handle,
                                  ble_centr_handles_t const * p_peer_handles)
{
    VERIFY_PARAM_NOT_NULL(p_ble_centr);

    p_ble_centr->conn_handle = conn_handle;
    if (p_peer_handles != NULL)
    {
        p_ble_centr->handles.centr_tx_cccd_handle = p_peer_handles->centr_tx_cccd_handle;
        p_ble_centr->handles.centr_tx_handle      = p_peer_handles->centr_tx_handle;
        p_ble_centr->handles.centr_rx_handle      = p_peer_handles->centr_rx_handle;
    }
    return NRF_SUCCESS;
}

#endif // NRF_MODULE_ENABLED(BLE_CENTR)
