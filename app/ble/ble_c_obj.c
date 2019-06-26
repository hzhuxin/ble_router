/**
 * @brief 
 * 
 * @file ble_c_app.c
 * @date 2018-08-31
 * @author Chenfei
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "peer_manager.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_centr.h"
#include "ble_conn_state.h"
#include "nrf_fstorage.h"
#include "fds.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_sdh_freertos.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "debug.h"
#include "ble_pkg.h"
#include "ble_init.h"
#include "ble_c_obj.h"

/*******************************defines**********************************/
#define SCAN_INTERVAL                   100                                      /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                     1200                                      /**< Determines scan window in units of 0.625 millisecond. */

#define SCAN_DURATION                   0x0000                                      /**< Duration of the scanning in units of 10 milliseconds. If set to 0x0000, scanning will continue until it is explicitly disabled. */

#define LOCK_TIMEOUT    pdMS_TO_TICKS(2000)

DBG_SET_LEVEL(DBG_LEVEL_I);

#define CHECK_OBJ(obj)                  \
{                                       \
  if(!obj) {                            \
    return HAL_ERR_PARAM;               \
  }                                     \
}
#define CHECK_OBJ_PRIV(obj)             \
{                                       \
  if(!obj) {                            \
    return HAL_ERR_PARAM;               \
  }                                     \
  if(!obj->priv) {                      \
    return HAL_ERR_NOT_INITIALIZED;     \
  }                                     \
}
#define CHECK_OBJ_LOCK(obj)             \
{                                       \
  if(!obj) {                            \
    return HAL_ERR_PARAM;               \
  }                                     \
  if(!obj->lock) {                      \
    return HAL_ERR_MEMORY;              \
  }                                     \
}
static StaticSemaphore_t lock_tab[1] = 
{
  {
  },
};

static ble_c_t obj_tab[] = 
{
  {
    .lock = NULL,
    .priv = NULL,
    .ops = NULL,
  },
};

//static ble_centr_t m_centr;
static SemaphoreHandle_t    sema_scan = NULL;
static scan_target_t        m_target_msg = SCAN_TARGET_MSG_DEFAULT;
static uint16_t m_conn_handle_c  = BLE_CONN_HANDLE_INVALID;
static SemaphoreHandle_t    sema_c_connect = NULL;
static SemaphoreHandle_t    sema_c_tx_complete = NULL;
static QueueHandle_t        c_rx_queue = NULL;
static ble_c_cfg_t          m_c_cfg;
static TickType_t         scan_start_time = 0;
static TickType_t         scan_timeout = 0;
static uint8_t            ble_c_mtu_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH;
//NRF_BLE_GATT_DEF(m_gatt_c);            /**< GATT module instance. */
//BLE_CENTR_ARRAY_DEF(m_centr,NRF_SDH_BLE_TOTAL_LINK_COUNT);
BLE_CENTR_DEF(m_centr);
//BLE_DB_DISCOVERY_ARRAY_DEF(m_db_discovery, NRF_SDH_BLE_CENTRAL_LINK_COUNT);  /**< Database discovery module instances. */
BLE_DB_DISCOVERY_DEF(m_db_discovery);                                    /**< DB discovery module instance. */

/*******************************functions**********************************/

static hal_err_t lock(ble_c_t* obj) 
{
  CHECK_OBJ_LOCK(obj);

  if(!xSemaphoreTake(obj->lock, LOCK_TIMEOUT)) 
  {
    return HAL_ERR_BUSY;
  }

  return HAL_ERR_OK;
}

/**
 * @brief Unlock the instance
 *
 * @param uart
 *
 * @return HAL_ERR_OK if success
 */
static hal_err_t unlock(ble_c_t* obj) 
{
  CHECK_OBJ_LOCK(obj);

  if(!xSemaphoreGive(obj->lock)) 
  {
    return HAL_ERR_UNLIKELY;
  }

  return HAL_ERR_OK;
}
/**@brief Parameters used when scanning. */
static ble_gap_scan_params_t const m_scan_params =
{
    .extended      = 0,
    .active        = 1,
    .interval      = SCAN_INTERVAL,
    .window        = SCAN_WINDOW,
    .timeout       = SCAN_DURATION,
    .scan_phys     = BLE_GAP_PHY_1MBPS,
    .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
};

static uint8_t m_scan_buffer_data[BLE_GAP_SCAN_BUFFER_EXTENDED_MIN]; /**< buffer where advertising reports will be stored by the SoftDevice. */

/**@brief Pointer to the buffer where advertising reports will be stored by the SoftDevice. */
static ble_data_t m_scan_buffer =
{
    m_scan_buffer_data,
    BLE_GAP_SCAN_BUFFER_EXTENDED_MIN
};

/**@brief Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param =
{
    MIN_CONNECTION_INTERVAL,
    MAX_CONNECTION_INTERVAL,
    SLAVE_LATENCY,
    SUPERVISION_TIMEOUT
};
void set_ble_c_mtu_len(uint8_t mtu_len)
{
    if(mtu_len > NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
    {
        return;
    }
    if(mtu_len < 20)
    {
        return;
    }
    ble_c_mtu_len = mtu_len;
    DBG_I("\r\nCentral data len is set to 0x%X(%d)\r\n", mtu_len, mtu_len);
}
/*
void ble_c_on_ble_evt(ble_evt_t const*p_ble_evt)
{
    ble_centr_on_ble_evt(p_ble_evt, &m_centr);
}*/
/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    DBG_I("db_disc_handler:p_evt->conn_handle = %d",p_evt->conn_handle);
    //ble_centr_on_db_disc_evt(&m_centr[p_evt->conn_handle], p_evt);
    ble_centr_on_db_disc_evt(&m_centr, p_evt);
}

/**
 * @brief Database discovery initialization.
 */
void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}
/**
 * @brief Get the central connect status object
 * 
 * @return true 
 * @return false 
 */
bool ble_c_connect_status(void)
{
    if(m_conn_handle_c != BLE_CONN_HANDLE_INVALID)
    {
        return true;
    }
    return false;
}
bool scan_start(void)
{
    ret_code_t err_code;

    (void) sd_ble_gap_scan_stop();
    if(m_target_msg.scan_params == NULL)
    {
        err_code = sd_ble_gap_scan_start(&m_scan_params, &m_scan_buffer);
    }
    else
    {
        err_code = sd_ble_gap_scan_start(m_target_msg.scan_params, &m_scan_buffer);
    }
    // It is okay to ignore this error since we are stopping the scan anyway.
    if (err_code != NRF_SUCCESS)
    {
        DBG_I("sd_ble_gap_scan_start failed, reason %d",err_code);
        return false;
    }
    return true;
    //DBG_I("start scanning...");
}
bool scan_stop(void)
{
    ret_code_t err_code;
    err_code = sd_ble_gap_scan_stop();
    if (err_code != NRF_SUCCESS)
    {
        DBG_I("sd_ble_gap_scan_stop failed, reason %d",err_code);
        return false;
    }
    memset(&m_target_msg,0,sizeof(m_target_msg));
    return true;
}

/**
 * @brief check_scan_advdata
 * 
 * @param p_encoded_data 
 * @param data_len 
 * @param peer_addr 
 * @return true 
 * @return false 
 */
static bool check_scan_advdata( char    **const p_name,
                                uint8_t * p_encoded_data,
                                uint16_t        data_len,
                                ble_gap_addr_t const* peer_addr)
{
    uint16_t   parsed_name_len;
    uint8_t  * p_parsed_name = NULL;
    uint16_t   data_offset = 0;
    bool ret = false;

    parsed_name_len = ble_advdata_search(p_encoded_data,
                                         data_len,
                                         &data_offset,
                                         BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME);

    p_parsed_name = &p_encoded_data[data_offset];

    if ((data_offset != 0)&& (parsed_name_len != 0))
    {
        *(p_parsed_name+parsed_name_len)= 0;
        // DBG_I("\r\n%s",p_parsed_name);
        // DBG_I("%02x:%02x:%02x:%02x:%02x:%02x\r\n",
        //                      peer_addr->addr[0],
        //                      peer_addr->addr[1],
        //                      peer_addr->addr[2],
        //                      peer_addr->addr[3],
        //                      peer_addr->addr[4],
        //                      peer_addr->addr[5]
        //                      );
        char *p_target = m_target_msg.name;
        char *p_scan = (char *)p_parsed_name;
        *p_name = (char *)p_parsed_name;
        switch(m_target_msg.mode)
        {
            case NEED_NAME_AND_ADDR:
                ret = ( strstr(p_scan,p_target) && \
                    (peer_addr->addr[0] == m_target_msg.addr[0]) && \
                    (peer_addr->addr[1] == m_target_msg.addr[1]) && \
                    (peer_addr->addr[2] == m_target_msg.addr[2]) && \
                    (peer_addr->addr[3] == m_target_msg.addr[3]) && \
                    (peer_addr->addr[4] == m_target_msg.addr[4]) && \
                    (peer_addr->addr[5] == m_target_msg.addr[5]))?  \
                    true:false;
                break;
            case NEED_ONLY_NAME:
                ret = strstr(p_scan,p_target)? true:false;
                break;

            case NEED_ONLY_ADDR:
                ret=((peer_addr->addr[0] == m_target_msg.addr[0]) && \
                    (peer_addr->addr[1] == m_target_msg.addr[1]) && \
                    (peer_addr->addr[2] == m_target_msg.addr[2]) && \
                    (peer_addr->addr[3] == m_target_msg.addr[3]) && \
                    (peer_addr->addr[4] == m_target_msg.addr[4]) && \
                    (peer_addr->addr[5] == m_target_msg.addr[5]))?  \
                    true:false;
                break;
            case NEED_NONE:
                //as long as any device has been scaned
                ret = true;
                break;
            default:
                ret = false;
                //DBG_I("invalid search mode.");
                break;
        }
    }
    return ret;
}

/**@brief Function for handling the advertising report BLE event.
 *
 * @param[in] p_adv_report  Advertising report from the SoftDevice.
 */
static void on_adv_report(ble_gap_evt_adv_report_t const * p_adv_report)
{
    char *target_name = NULL;
    bool ret = check_scan_advdata(&target_name,\
                                  p_adv_report->data.p_data, \
                                  p_adv_report->data.len, \
                                  &p_adv_report->peer_addr);
    if(ret)
    {
        DBG_I("\r\n%s",target_name);
        DBG_I("%02x:%02x:%02x:%02x:%02x:%02x\r\n",
                             p_adv_report->peer_addr.addr[0],
                             p_adv_report->peer_addr.addr[1],
                             p_adv_report->peer_addr.addr[2],
                             p_adv_report->peer_addr.addr[3],
                             p_adv_report->peer_addr.addr[4],
                             p_adv_report->peer_addr.addr[5]
                             );
        if(m_target_msg.handler != NULL)
        {
            target_t target;
            target.name         = (char *)target_name;
            target.peer_addr    = (ble_gap_addr_t*)&p_adv_report->peer_addr;
            m_target_msg.handler(&target);
        }
        else if(sema_scan != NULL)
        {
             xSemaphoreGive(sema_scan);
        }
    }
    if((scan_timeout != 0) && \
        (xTaskGetTickCount() - scan_start_time >= scan_timeout))
    {
        if(m_target_msg.handler != NULL)
        {
            m_target_msg.handler(NULL);
        }
    }
    else
    {
        scan_start();
    }
    //DBG_I("\r\nscanning\r\n");
}
/**@brief   Function for handling BLE events from central applications.
 *
 * @details This function parses scanning reports and initiates a connection to peripherals when a
 *          target UUID is found. It updates the status of LEDs used to report central applications
 *          activity.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
void on_ble_central_evt(ble_evt_t const * p_ble_evt)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        {
            DBG_I("+++++++++++++++++++ Central connected ++++++++++++++++++++");
            m_conn_handle_c = p_gap_evt->conn_handle;
            DBG_I("on_ble_central_evt:p_gap_evt->conn_handle =%d",p_gap_evt->conn_handle);
            APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < NRF_SDH_BLE_CENTRAL_LINK_COUNT);
            err_code = ble_centr_handles_assign(&m_centr, p_ble_evt->evt.gap_evt.conn_handle, NULL);
            //err_code = ble_centr_handles_assign(&m_centr[p_ble_evt->evt.gap_evt.conn_handle], p_ble_evt->evt.gap_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            // start discovery of services. The SLV Client waits for a discovery result
            //err_code = ble_db_discovery_start(&m_db_discovery[p_ble_evt->evt.gap_evt.conn_handle], p_ble_evt->evt.gap_evt.conn_handle);
            err_code = ble_db_discovery_start(&m_db_discovery, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
            if(c_rx_queue == NULL)
            //if(c_rx_queue[p_ble_centr_evt->conn_handle] == NULL)
            {
                c_rx_queue = xQueueCreate(1, sizeof(ble_data_t) );
                xQueueReset(c_rx_queue);
                //c_rx_queue[p_ble_centr_evt->conn_handle] = xQueueCreate(1, sizeof(ble_data_t));
                //xQueueReset(c_rx_queue[p_ble_centr_evt->conn_handle]);
            }
            if(sema_c_tx_complete == NULL)
            {
                //sema_c_tx_complete = ble_c_create_sema();
                sema_c_tx_complete = xSemaphoreCreateBinary();
                xSemaphoreTake( sema_c_tx_complete, 0 );
            }
        } break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
        {
            DBG_I("The central disconnected (reason: %d)",
                             p_gap_evt->params.disconnected.reason);
                             
            m_conn_handle_c = BLE_CONN_HANDLE_INVALID;
            vSemaphoreDelete( sema_c_tx_complete );
            sema_c_tx_complete = NULL;
            // if(sema_c_connect != NULL)
            // {
            //     //will not be perform after connected slave
            //     xSemaphoreGive(sema_c_connect);
            // }
            
        } break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_ADV_REPORT:
        {
            on_adv_report(&p_gap_evt->params.adv_report);
        } break; // BLE_GAP_ADV_REPORT

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                DBG_I("Connection Request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);

        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            DBG_I("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            DBG_I("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            DBG_I("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE:
            DBG_I("BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE");
            xSemaphoreGive(sema_c_tx_complete);
            break;

        default:
            // No implementation needed.
            break;
    }
}
/**
 * @brief ble_centr_evt_handler
 * 
 * @param p_ble_centr 
 * @param p_ble_centr_evt 
 */
static void ble_centr_evt_handler(ble_centr_t * p_ble_centr, ble_centr_evt_t const * p_ble_centr_evt)
{
    static uint8_t *c_rx_buf = NULL;
    static ble_data_t ble_data; 
    ret_code_t err_code;

    switch (p_ble_centr_evt->evt_type)
    {
        case BLE_CENTR_EVT_DISCOVERY_COMPLETE:
            DBG_I("Discovery complete, conn_handle = %d",p_ble_centr_evt->conn_handle);
            m_conn_handle_c = p_ble_centr_evt->conn_handle;
            err_code = ble_centr_handles_assign(p_ble_centr, p_ble_centr_evt->conn_handle, &p_ble_centr_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_centr_tx_notif_enable(p_ble_centr);
            APP_ERROR_CHECK(err_code);
            if(sema_c_connect != NULL)
            {
                //connect to slave
                xSemaphoreGive(sema_c_connect);
            }
            vPortFree(c_rx_buf);
            c_rx_buf = pvPortMalloc(BLE_RX_BUF_SIZE);
            if(c_rx_buf == NULL)
            {
                DBG_E("Malloced c_rx_buf is NULL");
                DBG_E("remain stack size %d",uxTaskGetStackHighWaterMark(NULL));  
            }
            if(m_c_cfg.conn_handler != NULL)
            {
                m_c_cfg.conn_handler(&m_conn_handle_c);
            }
            break;           
        case BLE_CENTR_EVT_CENTR_TX_EVT:
            ble_data.p_data = p_ble_centr_evt->p_data;
            ble_data.len = p_ble_centr_evt->data_len;
            DBG_D("c received a pkg data len = %d",ble_data.len);
            ble_data.len = ble_combine_sub_frame((uint8_t *)c_rx_buf,ble_data.p_data,ble_data.len);
            if(ble_data.len > 0)
            {    
                DBG_I("blec received complete data len = %d",ble_data.len);
                ble_pkg_head_t *p = (ble_pkg_head_t *)c_rx_buf;
                ble_data.len = p->data_len;
                ble_data.p_data = c_rx_buf+sizeof(ble_pkg_head_t);
                if(m_c_cfg.rx_handler != NULL)
                {                   
                    m_c_cfg.rx_handler(&ble_data);
                }
                else if(c_rx_queue != NULL)
                {
                    xQueueSendToBackFromISR(c_rx_queue, &ble_data, NULL);
                }
            }    
            break;

        case BLE_CENTR_EVT_DISCONNECTED:
            DBG_I("Central Disconnected slave.");
            vPortFree(c_rx_buf);
            c_rx_buf = NULL;
            m_conn_handle_c = BLE_CONN_HANDLE_INVALID;
            // if(sema_c_connect != NULL)
            // {
            //     xSemaphoreGive(sema_c_connect);
            // }
            if(m_c_cfg.disc_handler != NULL)
            {
                m_c_cfg.disc_handler(p_ble_centr);
            }
            break;
    }
}

static int32_t ble_c_send(void * const buffer, uint16_t length,uint16_t conn_handle)
{
    if(length <= 0 || buffer == NULL)
    {
        return false;
    }
    uint8_t     max_mtu     = get_conn_gatt_mtu(m_conn_handle_c);
    uint16_t    len_cnt     = length;   
    uint32_t    err_code    = NRF_SUCCESS;      
    uint8_t     send_buff[NRF_SDH_BLE_GATT_MAX_MTU_SIZE]  ;   
    uint8_t     sub_pkg_len = 0;
    uint8_t    *p_data      = (uint8_t *)buffer;
    //vPortFree(send_buff);
    //send_buff = pvPortMalloc(max_mtu);

    if(send_buff == NULL || buffer == NULL)
    {
        DBG_E("ble_c_send:remain stack size %d",uxTaskGetStackHighWaterMark(NULL));
        DBG_E("ble_c_send pointer buffer or send buff is NULL");
        //vPortFree(send_buff);
        return 0;
    }
    if(sema_c_tx_complete == NULL)
    {
        DBG_E("ble_c_send pointer sema_c_tx_complete is NULL");
        //vPortFree(send_buff);
        return 0;
    }
    xSemaphoreTake( sema_c_tx_complete, 0 );
    DBG_I("blec will send %d bytes, max MTU = %d, connid %d\r\n",length, max_mtu,conn_handle);  
    do
    {            
        //split the datas to sub package  to send
        get_sub_split(send_buff,&p_data[length-len_cnt],len_cnt,length,max_mtu-1);
        if(len_cnt < max_mtu)
        {            
            sub_pkg_len = len_cnt;
            //err_code = ble_centr_send(&m_centr[conn_handle],send_buff,len_cnt+1);
            err_code = ble_centr_send(&m_centr,send_buff,len_cnt+1);
            len_cnt = 0;
        }
        else    //len_cnt > m_ble_rts_max_data_len
        {
            sub_pkg_len = max_mtu - 1;
            //err_code = ble_centr_send(&m_centr[conn_handle],send_buff,max_mtu);
            err_code = ble_centr_send(&m_centr,send_buff,max_mtu);
            len_cnt -= (max_mtu - 1);
        }
        if(err_code != NRF_SUCCESS)
        {
            if(err_code == NRF_ERROR_INVALID_STATE)
            {
               DBG_I("ble transmit failed: NRF_ERROR_INVALID_STATE\r\n");
               //vPortFree(send_buff);
                return false;
            }
            if(err_code == NRF_ERROR_INVALID_PARAM)
            {
                DBG_I("ble transmit failed with NRF_ERROR_INVALID_PARAM\r\n");
                //vPortFree(send_buff);
                return false;
            }
            //ble send buffer is full, waiting for tx complete
            len_cnt += sub_pkg_len;
            if(xSemaphoreTake( sema_c_tx_complete, BLE_TX_COMPLETE_WAIT ) != pdPASS)
            {
                DBG_I(">>>>>>>>>>>>>>  ble transmit timeout  <<<<<<<<<<<<<< %d\r\n");
                //vPortFree(send_buff);
                return false;
            }
            //DBG_I("received BLE_GATTS_EVT_HVN_TX_COMPLETE\r\n");
        }
        DBG_I("\r\nblec send splitlen = %d",sub_pkg_len);
    }while(len_cnt);      
    DBG_I("ble send data complete\r\n");
    //vPortFree(send_buff);
    return true;
}
static int32_t ble_c_pkg_send(uint8_t *buf,uint16_t length, uint16_t conn_handle)
{
    if(buf == NULL)
    {
        DBG_E("ble_c_pkg_send pointer buf is NULL");
        return 0;
    }   
    if(conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        DBG_E("ble_c_pkg_send no connection");
        return 0;
    }    
    uint32_t len = 0;
    uint8_t tx_buf[BLE_TX_BUF_SIZE];
    // uint8_t *tx_buf = NULL;
    //vPortFree(tx_buf);
    //tx_buf = pvPortMalloc(length+sizeof(ble_pkg_head_t)+2);
    len = ble_package(tx_buf,buf,BLE_TX_BUF_SIZE,length);

    if(len > 0)
    {
        len = ble_c_send(tx_buf,len,conn_handle);
    }
    else
    {
        DBG_E("ble_c_pkg_send data package failed");
    }
    //vPortFree(tx_buf);
    return len;
}
/**@brief Function for initializing the SLV Client. */
void centr_init(void)
{
    ret_code_t       err_code;
    ble_centr_init_t init;;

    init.evt_handler = ble_centr_evt_handler;
    // for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    // {
    //     err_code = ble_centr_init(&m_centr[i], &init);
    //     APP_ERROR_CHECK(err_code);
    // }
    err_code = ble_centr_init(&m_centr, &init);
    APP_ERROR_CHECK(err_code);
}
static hal_err_t ble_c_init(ble_c_t *obj,ble_c_cfg_t * config)
{
    CHECK_OBJ(obj);
    if(config != NULL)
    {   
        //if has a call back user given
        memcpy(&m_c_cfg,config,sizeof(ble_c_cfg_t));
        if(config->rx_handler != NULL)
        {
            return HAL_ERR_OK;
        }
    }
    else
    {
        memset(&m_c_cfg,0,sizeof(ble_c_cfg_t));
    }  
    //if no callback, the queue will be used
    sema_c_tx_complete = xSemaphoreCreateBinary();
    c_rx_queue = xQueueCreate(3, sizeof(ble_data_t) );
    if((sema_c_tx_complete == NULL) || \
    (c_rx_queue == NULL))
    {
        return HAL_ERR_MEMORY;
    } 
    return HAL_ERR_OK;
}
static hal_err_t ble_c_deinit(ble_c_t *obj)
{
    CHECK_OBJ(obj);
    vSemaphoreDelete( sema_c_tx_complete );
    vQueueDelete(c_rx_queue);
    sema_c_tx_complete = NULL;
    c_rx_queue = NULL;
    memset(&m_c_cfg,0,sizeof(ble_c_cfg_t));
    return HAL_ERR_OK;
}
hal_err_t ble_centr_scan(ble_c_t *obj, scan_target_t const *p_target, int32_t timeout)
{
    CHECK_OBJ(obj);
    if(p_target == NULL)
    {
        DBG_I("scan pointer target is NULL.");
        return HAL_ERR_PARAM;
    }
    memset(&m_target_msg,0,sizeof(m_target_msg));

    m_target_msg.scan_params = p_target->scan_params;    
    m_target_msg.mode  = p_target->mode;
    m_target_msg.handler = p_target->handler;
    
    switch(p_target->mode)
    {
        case NEED_NAME_AND_ADDR:
            DBG_I("scanning base on name and addr.");
            if((p_target->name == NULL) || (p_target->addr == NULL))
            {
                DBG_I("invalid scan name or addr.");
                return HAL_ERR_FAIL;
            }
            //strcpy(m_target_msg.name,p_target->name);
            //memcpy(m_target_msg.addr,p_target->addr,BLE_GAP_ADDR_LEN);
            m_target_msg.name  = p_target->name;
            m_target_msg.addr  = p_target->addr;
            break;
        case NEED_ONLY_NAME:
            DBG_I("scanning base on only name.");
            if(p_target->name == NULL)
            {
                DBG_I("invalid search name.");
                return HAL_ERR_FAIL;
            }
            //strcpy(m_target_msg.name,p_target->name);
             m_target_msg.name  = p_target->name;
            break;

        case NEED_ONLY_ADDR:
            DBG_I("scanning base on only addr.");
            if(p_target->addr == NULL)
            {
                DBG_I("invalid search name.");
                return HAL_ERR_FAIL;
            }
            //memcpy(m_target_msg.addr,p_target->addr,BLE_GAP_ADDR_LEN);
            m_target_msg.addr  = p_target->addr;
            break;
        case NEED_NONE: 
            DBG_I("scanning base on none.");
            break;
        default:
            DBG_I("invalid search mode.");
            return HAL_ERR_FAIL;
            //break;
    }
    if(!scan_start())
    {
        DBG_I("scan start failed.");
        return HAL_ERR_FAIL;
    }
    scan_start_time = xTaskGetTickCount();
    scan_timeout = 0;
    if(m_target_msg.handler != NULL)
    {
        //if has the call back , not need to waiting
        scan_timeout = timeout;
        return HAL_ERR_OK;
    }
    if(sema_scan == NULL)
    {
        sema_scan = xSemaphoreCreateBinary();
        xSemaphoreTake(sema_scan,0);
    }
    hal_err_t ret;
    uint32_t time = (timeout < 1000)?SCAN_TIME_DEFAULT:timeout;

    ret = (xSemaphoreTake(sema_scan,time) == pdPASS)?HAL_ERR_OK:HAL_ERR_FAIL;

    scan_stop();
    vSemaphoreDelete(sema_scan);
    sema_scan = NULL;
    DBG_I("scan over, it's %s",(ret == HAL_ERR_OK)?"successed":"failed");
    return ret;
}
hal_err_t ble_centr_scan_stop(ble_c_t *obj)
{
    CHECK_OBJ(obj);
    if(!scan_stop())
    {
        return HAL_ERR_FAIL;
    }
    return HAL_ERR_OK;
}
hal_err_t ble_centr_connect(ble_c_t *obj, connect_target_t const *p_target, int32_t timeout)
{
    CHECK_OBJ(obj);

    if(*p_target->conn_handle != BLE_CONN_HANDLE_INVALID )
    //if((m_conn_handle_c != BLE_CONN_HANDLE_INVALID))
    {
        DBG_I("The device %x:%x:%x:%x:%x:%x connected already",
                    p_target->addr[0],
                    p_target->addr[1],
                    p_target->addr[2],
                    p_target->addr[3],
                    p_target->addr[4],
                    p_target->addr[5]);
        return HAL_ERR_ALREADY_INITIALIZED;
    }
    uint32_t err_code;
    ble_gap_addr_t ble_gap_addr;
    memset(&ble_gap_addr,0,sizeof(ble_gap_addr));
    if(p_target->addr != NULL)
    {
        memcpy(ble_gap_addr.addr,p_target->addr,BLE_GAP_ADDR_LEN);
    }
    else
    {
        DBG_I("Connection give mac is NULL, so use the default");
        return HAL_ERR_FAIL;
    }
    hal_err_t ret = HAL_ERR_FAIL;
    if(sema_c_connect == NULL)
    {
        sema_c_connect = xSemaphoreCreateBinary();
        xSemaphoreTake(sema_c_connect,0);
    }
    sd_ble_gap_connect_cancel();
    m_conn_handle_c = BLE_CONN_HANDLE_INVALID ;
    ble_gap_addr.addr_id_peer = 0;
    ble_gap_addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;

    ble_gap_scan_params_t * scan_params;
    ble_gap_conn_params_t * conn_params;

    if(p_target->scan_params != NULL)
    {
        scan_params = p_target->scan_params;
    }
    else
    {
        scan_params = (ble_gap_scan_params_t *)&m_scan_params;
    }
    if(p_target->conn_params != NULL)
    {
        conn_params = p_target->conn_params;
    }
    else
    {
        conn_params = (ble_gap_conn_params_t *)&m_connection_param;
    }
    err_code = sd_ble_gap_connect(&ble_gap_addr,
                                    scan_params,
                                    conn_params,
                                    APP_BLE_CONN_CFG_TAG);        
    if (err_code != NRF_SUCCESS)
    {
        DBG_E("Connection Request Failed, reason %d", err_code);
        return HAL_ERR_FAIL;
    }
    DBG_I("Waiting to connect slave device,%02x:%02x:%02x:%02x:%02x:%02x",
                    p_target->addr[0],
                    p_target->addr[1],
                    p_target->addr[2],
                    p_target->addr[3],
                    p_target->addr[4],
                    p_target->addr[5]);
    if(m_c_cfg.conn_handler != NULL)
    {
        //if has the callback , not need to waiting
        return HAL_ERR_OK;
    }
    if(xSemaphoreTake(sema_c_connect,timeout) == pdPASS)
    {
        if(m_conn_handle_c != BLE_CONN_HANDLE_INVALID)
        {
            *p_target->conn_handle = m_conn_handle_c;
            ret = HAL_ERR_OK;
        }
        else
        {
            ret = HAL_ERR_FAIL;
            DBG_E("Connect failed.");
            sd_ble_gap_connect_cancel();
        }
    }
    else
    {
        ret = HAL_ERR_FAIL;
        m_conn_handle_c = BLE_CONN_HANDLE_INVALID;
        *p_target->conn_handle = BLE_CONN_HANDLE_INVALID;
        sd_ble_gap_connect_cancel();
        DBG_E("Connection timeout");
    }
    DBG_I("Connect slave %s.",(ret==HAL_ERR_OK)?"successed":"failed");
    vSemaphoreDelete(sema_c_connect);
    sema_c_connect = NULL;

    return ret;
}
hal_err_t ble_centr_disconnect(ble_c_t *obj, uint16_t *conn_handle)
{
    CHECK_OBJ(obj);

    uint32_t err_code;
    DBG_I("central voluntary to disconnect slave.");
    if(*conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        DBG_I("The device has disconnected");
        return HAL_ERR_OK;
    }
    err_code = sd_ble_gap_disconnect(*conn_handle,
                                    BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if(err_code != NRF_SUCCESS)
    {
        return HAL_ERR_FAIL;
    }
    *conn_handle = BLE_CONN_HANDLE_INVALID;

    return HAL_ERR_OK;
}
hal_err_t ble_centr_transmit(ble_c_t *obj, uint16_t conn_handle,uint8_t *buf,uint16_t length)
{
    CHECK_OBJ(obj);

    if(conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        DBG_E("No connection");
        return HAL_ERR_NOT_INITIALIZED;
    }

    if(!ble_c_pkg_send(buf,length,conn_handle))
    {
        DBG_E("blec pkg send failed");
        return HAL_ERR_FAIL;
    }
    return HAL_ERR_OK;
}
hal_err_t ble_centr_recieve(ble_c_t *obj, 
                            uint16_t conn_handle,
                            uint8_t *buf, 
                            uint16_t size,
                            uint32_t timeout)
{
    CHECK_OBJ(obj);

    if((buf == NULL) || (size == 0))
    {
        DBG_I("ble c receive pointer is null");
        return HAL_ERR_DENIAL;
    }
    ble_data_t rx_data;
    if(c_rx_queue == NULL)
    {
        return HAL_ERR_MEMORY;
    }
    if(xQueueReceive(c_rx_queue, &rx_data , timeout) != pdPASS)
    {
        return HAL_ERR_FAIL;
    } 
    DBG_I("ble rx len = %d",rx_data.len);
    rx_data.len = (rx_data.len > size)? size : rx_data.len;
    memcpy(buf,rx_data.p_data,rx_data.len) ;
    return rx_data.len;
}
hal_err_t ble_c_get_mtu(ble_c_t *obj, uint16_t conn_handle)
{
    int mtu=0;
    mtu = get_conn_gatt_mtu(conn_handle);
    if((mtu < 20) || (mtu > (NRF_SDH_BLE_GATT_MAX_MTU_SIZE- OPCODE_LENGTH - HANDLE_LENGTH)))
    {
        return HAL_ERR_FAIL;
    }
    return mtu;
}
ble_c_t* ble_c_get_instance(int id) 
{
    static const ble_c_ops_t ops = 
    {
        .lock       = lock,
        .unlock     = unlock,
        .init       = ble_c_init,
        .deinit     = ble_c_deinit,
        .scan       = ble_centr_scan,
        .scan_stop  = ble_centr_scan_stop,
        .connect    = ble_centr_connect,
        .disconnect = ble_centr_disconnect,
        .transmit   = ble_centr_transmit,
        .receive    = ble_centr_recieve,
        .get_mtu    = ble_c_get_mtu,
    };

    if(id < 0 || id >= sizeof(obj_tab) / sizeof(*obj_tab)) 
    {
        return NULL;
    }
    ble_c_t* obj = &obj_tab[id];
    if(!obj->lock)
    {
        obj->lock = xSemaphoreCreateMutexStatic(&lock_tab[id]);
    }
    if(!obj->lock) 
    {
        return NULL;
    }

    obj->ops = &ops;

    return obj;
}