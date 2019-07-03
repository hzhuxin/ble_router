/**
 * @brief 
 * 
 * @file ble_s_app.c
 * @date 2018-09-05
 * @author Chenfei
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_SLV)
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
#include "ble_slave.h"
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
#include "ble_s_obj.h"
#include "ble_gap.h"

/*******************************defines**********************************/

#define LOCK_TIMEOUT    pdMS_TO_TICKS(2000)
DBG_SET_LEVEL(DBG_LEVEL_D);

#define CHECK_OBJ(obj)                  \
{                                       \
    if(!obj){return HAL_ERR_PARAM;}     \
}
#define CHECK_OBJ_PRIV(obj)             \
{                                       \
    if(!obj) {return HAL_ERR_PARAM;}    \
    if(!obj->priv){return HAL_ERR_NOT_INITIALIZED;}\
}
#define CHECK_OBJ_LOCK(obj)             \
{                                       \
  if(!obj) {return HAL_ERR_PARAM;}      \
  if(!obj->lock) {return HAL_ERR_MEMORY;}\
}
static StaticSemaphore_t lock_tab;

static ble_s_t obj_tab = {.lock = NULL,
                          .priv = NULL,
                          .ops = NULL,};

static ble_s_cfg_t          m_s_cfg;
static uint16_t             m_conn_handle_s     = BLE_CONN_HANDLE_INVALID;
static QueueHandle_t        queue_s_rx          = NULL;
static SemaphoreHandle_t    sema_s_tx_complete  = NULL;
static SemaphoreHandle_t    sema_s_connect = NULL;
static uint8_t            * slv_rx_buf = NULL;
static uint8_t              ble_s_mtu_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH;
NRF_BLE_GATT_DEF(m_gatt_s); 
BLE_SLV_DEF(m_slave, NRF_SDH_BLE_TOTAL_LINK_COUNT); /**< BLE SLV service instance for peripheral. */
/*******************************functions**********************************/

void set_ble_s_mtu_len(uint8_t mtu_len)
{
    if(mtu_len > NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
    {
        return;
    }
    if(mtu_len < 20)
    {
        return;
    }
    ble_s_mtu_len = mtu_len;
    DBG_I("\r\nSlave data len is set to 0x%X(%d)\r\n", mtu_len, mtu_len);
}
/**@brief   Function for handling BLE events from peripheral applications.
 * @details Updates the status LEDs used to report the activity of the peripheral applications.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
void on_ble_peripheral_evt(ble_evt_t const * p_ble_evt)
{
    ret_code_t err_code;
    static SemaphoreHandle_t   sema_s_tx_complete = NULL;
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            DBG_I("Peripheral connected");
            m_conn_handle_s = p_ble_evt->evt.gap_evt.conn_handle;
            // Assing connection handle to the QWR module.
            multi_qwr_conn_handle_assign(p_ble_evt->evt.gap_evt.conn_handle);
            if(sema_s_tx_complete == NULL)
            {
                sema_s_tx_complete = xSemaphoreCreateBinary();
            }
            if(m_s_cfg.conn_handler != NULL)
            {
                m_s_cfg.conn_handler(NULL);
            }
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            DBG_I("Peripheral disconnected");
            m_conn_handle_s = BLE_CONN_HANDLE_INVALID;
            vPortFree(slv_rx_buf);
            //slv_rx_buf = NULL;
            //vQueueDelete(queue_s_rx);
            //queue_s_rx = NULL;
            vSemaphoreDelete(sema_s_tx_complete);
            sema_s_tx_complete = NULL;
            if(m_s_cfg.disc_handler != NULL)
            {
                m_s_cfg.disc_handler(NULL);
            }
            break;

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
        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            DBG_I("BLE_GATTS_EVT_HVN_TX_COMPLETE");
            if(sema_s_tx_complete != NULL)
            {
                xSemaphoreGive(sema_s_tx_complete);
            }
            break;
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling the data from the master.
 *
 * @param[in] p_evt       BLE Peripheral Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void slv_data_handler(ble_slv_evt_t * p_evt)
{
    static ble_data_t ble_data;
    switch(p_evt->type)
    {
        case BLE_SLV_EVT_RX_DATA:
        {
            //DBG_I("Received data from BLE SLV.");
            ble_data.p_data = (uint8_t *)p_evt->params.rx_data.p_data;
            ble_data.len = p_evt->params.rx_data.length;
            DBG_I("act slave received data len = %d",ble_data.len);
            ble_data.len = ble_combine_sub_frame((uint8_t *)slv_rx_buf,ble_data.p_data,ble_data.len);
            if((ble_data.len > 0))
            {                
                if(m_s_cfg.rx_handler != NULL)
                {
                    m_s_cfg.rx_handler(&ble_data);
                }
                else if(queue_s_rx != NULL)
                {
                    ble_data.p_data = (uint8_t *)slv_rx_buf;
                    xQueueSendToBackFromISR(queue_s_rx, &ble_data, NULL);
                    DBG_I("received total data len = %d",ble_data.len);                  
                    DBG_I("data address = 0x%x",slv_rx_buf);
                }
            }
        }
        case BLE_SLV_EVT_COMM_STARTED:
            DBG_I("BLE_SLV_EVT_COMM_STARTED");
            //rx memeory
            vPortFree(slv_rx_buf);
            slv_rx_buf = pvPortMalloc(BLE_RX_BUF_SIZE);
            if(m_s_cfg.notify_handler != NULL)
            {
                m_s_cfg.notify_handler(NULL);
            }
            break;
        case BLE_SLV_EVT_COMM_STOPPED:
            DBG_I("BLE_SLV_EVT_COMM_STOPPED");
            if(m_s_cfg.unnotify_handler != NULL)
            {
                m_s_cfg.unnotify_handler(NULL);
            }
            break;
        default : 
            break;
    }

}
/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
void services_init(void)
{
    uint32_t           err_code;
    ble_slv_init_t     slv_init;

    // Initialize SLV.
    memset(&slv_init, 0, sizeof(slv_init));

    slv_init.data_handler = slv_data_handler;

    err_code = ble_slv_init(&m_slave, &slv_init);
    APP_ERROR_CHECK(err_code);
}

static hal_err_t ble_s_send_one_split(void const * p_buf, uint16_t length)
{
    uint8_t *buf = (uint8_t *)p_buf;
    if(buf == NULL)
    {
        return false;
    }
    if(length <= 0 || length > ble_s_mtu_len)
    {
        return false;
    }

    uint32_t err_code = ble_slv_data_send(&m_slave,buf,&length,m_conn_handle_s);

    if(err_code != NRF_SUCCESS)
    {
        if(err_code == NRF_ERROR_BUSY)
        {
            //ble send buffer is full, waiting for tx complete
            uint32_t time = BLE_TX_COMPLETE_WAIT;
            if(m_s_cfg.tx_timeout != 0)
            {
                time = m_s_cfg.tx_timeout;
            }
            if(xSemaphoreTake(sema_s_tx_complete, time ) == pdPASS)
            {
                return HAL_ERR_BUSY;
            }
            DBG_I("-----  ble transmit timeout  %dS -----\r\n",BLE_TX_COMPLETE_WAIT);
            return HAL_ERR_TIMEOUT;
        }
        else
        {
            DBG_I("BLE of slave transmit sub split failed\r\n");
            return HAL_ERR_FAIL;
        }       
    }
#if 0
    DBG_I("send one split");
    for(uint8_t i=0;i<length;i++)
    {
        DBG_B("%x ",buf[i]);
    }
    DBG_B("\r\n");
#endif
    return HAL_ERR_OK;
}
/**
 * @brief 
 * 
 * @param p_buf 
 * @param length 
 * @return uint32_t 
 */
static hal_err_t ble_s_send(void const * buffer, int16_t const length)
{
    if(length <= 0 || buffer == NULL)
    {
        return HAL_ERR_PARAM;
    }
    if(sema_s_tx_complete == NULL)
    {
        return HAL_ERR_NOT_INITIALIZED;
    }
    uint16_t    len_cnt     = length;      
    uint8_t     *send_buff = NULL ;   
    uint8_t     sub_pkg_len = 0;
    uint8_t    *p_data      = (uint8_t *)buffer;
    hal_err_t ret = 0;

    ble_s_mtu_len = get_conn_gatt_mtu(m_conn_handle_s);
    DBG_I("BLE_S MTU length = %d",ble_s_mtu_len);
    vPortFree(send_buff);
    send_buff = pvPortMalloc(ble_s_mtu_len+1);
    do
    {            
        sub_pkg_len = get_sub_split(send_buff,&p_data[length-len_cnt],len_cnt,length,ble_s_mtu_len-1);
        ret = ble_s_send_one_split(send_buff,(sub_pkg_len+1));
        if(ret == HAL_ERR_OK)
        {
            DBG_I("send success length = %d",sub_pkg_len);
            len_cnt = (len_cnt < ble_s_mtu_len)? 0 : (len_cnt - sub_pkg_len);
        }
        else if(ret == HAL_ERR_BUSY)
        {
            //len_cnt = len_cnt;
            DBG_I("because of busy at previous, now resend last split, length = %d",sub_pkg_len);
        }
        else
        {
            //re-send here
            DBG_I("send failed length = %d",sub_pkg_len);
            vPortFree(send_buff);
            send_buff = NULL;
            return HAL_ERR_DENIAL;
        }
        DBG_I("remanin package len = %d\r\n",len_cnt);
    }while(len_cnt);
    vPortFree(send_buff);
    send_buff = NULL;
    DBG_I("ble send data complete\r\n");
    return HAL_ERR_OK;
}
static hal_err_t ble_s_init(ble_s_t *obj,ble_s_cfg_t * config)
{
    CHECK_OBJ(obj);
    if(config != NULL)
    {   
        //if has a call back user given
        memcpy((uint8_t *)&m_s_cfg,(uint8_t *)config,sizeof(ble_s_cfg_t));
        if(config->rx_handler != NULL)
        {
            return HAL_ERR_OK;
        }
    }
    else
    {
        memset(&m_s_cfg,0,sizeof(ble_s_cfg_t));
    }  
    //if no callback, the queue will be used
    sema_s_tx_complete = xSemaphoreCreateBinary();
    queue_s_rx = xQueueCreate(3, sizeof(ble_data_t) );
    if((sema_s_tx_complete == NULL) || \
    (queue_s_rx == NULL))
    {
        return HAL_ERR_MEMORY;
    } 

    return HAL_ERR_OK;
}
static hal_err_t ble_s_deinit(ble_s_t *obj)
{
    CHECK_OBJ(obj);
    vSemaphoreDelete( sema_s_tx_complete );
    vQueueDelete(queue_s_rx);
    sema_s_tx_complete = NULL;
    queue_s_rx = NULL;
    memset(&m_s_cfg,0,sizeof(ble_s_cfg_t));
    return HAL_ERR_OK;
}
static hal_err_t lock(ble_s_t* obj) 
{
    CHECK_OBJ_LOCK(obj);
    if(!xSemaphoreTake(obj->lock, LOCK_TIMEOUT)) 
    {
        return HAL_ERR_BUSY;
    }
    return HAL_ERR_OK;
}

static hal_err_t unlock(ble_s_t* obj) 
{
    CHECK_OBJ_LOCK(obj);

    if(!xSemaphoreGive(obj->lock)) 
    {
        return HAL_ERR_UNLIKELY;
    }
    return HAL_ERR_OK;
}
static hal_err_t ble_s_get_status(ble_s_t *obj,bool *status)
{
    CHECK_OBJ_LOCK(obj);
    if(status == NULL)
    {
        return HAL_ERR_PARAM;
    }
    *status = (m_conn_handle_s == BLE_CONN_HANDLE_INVALID)? false:true;

    return HAL_ERR_OK;
}
static hal_err_t ble_s_disconnect(ble_s_t *obj)
{
    CHECK_OBJ(obj);
    
    if(m_conn_handle_s == BLE_CONN_HANDLE_INVALID)
    {
        DBG_I("The device has disconnected");
        return HAL_ERR_ALREADY_INITIALIZED;
    }
    if(sema_s_connect == NULL)
    {
        sema_s_connect = xSemaphoreCreateBinary();
        xSemaphoreTake(sema_s_connect,0);
    }
    hal_err_t ret;
    uint32_t error_code;
    error_code = sd_ble_gap_disconnect(m_conn_handle_s,
                          BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if(error_code != NRF_SUCCESS)
    {
        vSemaphoreDelete(sema_s_connect);
        sema_s_connect = NULL;
        DBG_I("sd_ble_gap_disconnect failed, reson %d.",error_code);
        return HAL_ERR_FAIL;
    }
    ret = (xSemaphoreTake(sema_s_connect,5000) == pdPASS)?HAL_ERR_OK:HAL_ERR_FAIL;

    vSemaphoreDelete(sema_s_connect);
    sema_s_connect = NULL;
    DBG_I("disconnect slave.");
    return ret;
}
static hal_err_t ble_s_transmit(ble_s_t *obj, uint8_t *buf, int16_t len)
{
    CHECK_OBJ(obj);
    if(buf == NULL || len <= 0)
    {
        DBG_I("the ble data pointer is NULL.");
        return HAL_ERR_PARAM;
    }   
    if(m_conn_handle_s == BLE_CONN_HANDLE_INVALID)
    {
        return HAL_ERR_NOT_INITIALIZED;
    }
    uint8_t *tx_buf = NULL;
    vPortFree(tx_buf);
    tx_buf = pvPortMalloc(BLE_TX_BUF_SIZE);
    if(tx_buf == NULL)
    {
        DBG_I("malloc failed");
        vPortFree(tx_buf);
        return HAL_ERR_MEMORY;
    }
    len = ble_package(tx_buf,buf,BLE_TX_BUF_SIZE,len);
    DBG_I("ble packaged total len = %d\r\n",len);
    if(len <= 0)
    {        
        vPortFree(tx_buf);
        return HAL_ERR_MEMORY;
    }
    hal_err_t ret = ble_s_send(tx_buf,len);
    vPortFree(tx_buf);
    return ret;
}
static hal_err_t ble_s_recieve(ble_s_t *obj,uint8_t *buf, int16_t size,uint32_t timeout)
{
    CHECK_OBJ(obj);
    if((buf == NULL)|| (size <= 0))
    {
        DBG_I("ble s receive memeory invalid");
        return HAL_ERR_PARAM;
    }
    ble_data_t rx_data;
    if(queue_s_rx == NULL)
    {
        return HAL_ERR_NOT_INITIALIZED;
    }
    if(xQueueReceive(queue_s_rx, &rx_data , timeout) != pdPASS)
    {
        return HAL_ERR_TIMEOUT;
    } 
    if(rx_data.len > 0 && rx_data.len <= size)
    {
        rx_data.len = ble_get_valid_field(buf,rx_data.p_data,rx_data.len);       
        return rx_data.len;
    }
    else
    {
        if(rx_data.len > size)
        {
            DBG_I("rx_len = %d > max size = %d",rx_data.len,size);
        }
        else
        {
            DBG_I("rx_len = 0");
        }
        return HAL_ERR_MEMORY;
    } 
}
static hal_err_t ble_s_get_mac(ble_s_t * obj, uint8_t *buf,uint8_t size)
{
    CHECK_OBJ(obj);
    uint32_t err_code;
    ble_gap_addr_t gap_addr;
    if((buf == NULL) || (size < BLE_GAP_ADDR_LEN))
    {
        return HAL_ERR_PARAM;
    }
    err_code = sd_ble_gap_addr_get(&gap_addr);
    if(err_code != NRF_SUCCESS)
    {
        return HAL_ERR_DENIAL;
    }
    memcpy(buf,gap_addr.addr,BLE_GAP_ADDR_LEN);
    return BLE_GAP_ADDR_LEN;
}
hal_err_t ble_s_get_mtu(ble_s_t *obj)
{
    int mtu=0;
    mtu = get_conn_gatt_mtu(m_conn_handle_s);
    if((mtu < 20) || (mtu > (NRF_SDH_BLE_GATT_MAX_MTU_SIZE- OPCODE_LENGTH - HANDLE_LENGTH)))
    {
        return HAL_ERR_FAIL;
    }
    return mtu;
}


ble_s_t* ble_s_get_instance(void) 
{
    static const ble_s_ops_t ops = 
    {
        .lock       = lock,
        .unlock     = unlock,
        .init       = ble_s_init,
        .deinit     = ble_s_deinit,
        .disconnect = ble_s_disconnect,
        .transmit   = ble_s_transmit,
        .receive    = ble_s_recieve,
        .status     = ble_s_get_status,
        .get_mac    = ble_s_get_mac,
        .get_mtu    = ble_s_get_mtu,
    };

    ble_s_t* obj = &obj_tab;
    if(!obj->lock)
    {
        obj->lock = xSemaphoreCreateMutexStatic(&lock_tab);
    }
    if(!obj->lock) 
    {
        return NULL;
    }
    obj->ops = &ops;

    return obj;
}

#endif //NRF_MODULE_ENABLED(BLE_SLV_ENABLED)
