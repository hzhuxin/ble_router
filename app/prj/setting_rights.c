/**
 * @brief 
 * 
 * @file setting_limits.c
 * @date 2018-09-28
 * @author Chenfei
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */

/************************************includes************************************************/
#include "setting_rights.h"
//#include "init.h"
#include "debug.h"
//#include "app_log.h"
#include "router_setting.pb.h"
#include "HexStr.h"
#include "ble_gap.h"
#include "pb_callback.h"
#include "crc16.h"
#include "Define.pb.h"
#include "execute_state.h"
#include "hal.h"
#include "hal_rtc.h"
#include "net_manage.h"
#include "ble_c_app.h"

/************************************defines************************************************/
#define BUF_SIZE        500
DBG_SET_LEVEL(DBG_LEVEL_D);
//DBG_LOG_ENABLE(true);
static TaskHandle_t     setting_rights_handle = NULL;
//static QueueHandle_t    task_reult_queue = NULL;
static uint8_t          slave_settings_buf[512];
static uint16_t         slave_settings_len = 0;
static uint16_t         mac_addr_num = 0;
static struct mac_list_t *m_mac_list = NULL;
static struct mac_list_t *p_read_mac = NULL;
static struct mac_list_t *p_write_mac = NULL;
//static uint32_t         router_id;
//static router_params_t        m_router = {0};
static bool              rights_mode = 0;
/************************************functions************************************************/

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool mac_list_init(void)
{
    //m_mac_list.next = NULL;
    if(m_mac_list != NULL)
    {
        return true;
    }
    vPortFree(m_mac_list);
    m_mac_list = (mac_list_t *)pvPortMalloc(sizeof(mac_list_t));
    if(m_mac_list == NULL)
    {
        DBG_I("link list init failed\r\n");
        return false;
    }
    m_mac_list->next = NULL;
    p_read_mac = m_mac_list;
    p_write_mac = m_mac_list;
    DBG_D("link list init successed\r\n");
    return true;
}
/**
 * @brief 
 * 
 * @param mac 
 * @return mac_list_t* 
 */
static mac_list_t * mac_list_search(uint8_t const *mac)
{
    if(m_mac_list == NULL)
    {
        return NULL;
    }
    mac_list_t *p = m_mac_list;
    for(;p != NULL;p=p->next)
    {
        if(memcmp(mac,p->addr,6) == 0)
        {
            return p;
        }
    }
    return NULL;
}
/**
 * @brief 
 * 
 * @param mac 
 * @return true 
 * @return false 
 */
bool mac_list_back_insert(uint8_t const *mac)
{
    if(m_mac_list == NULL)
    {
        if(mac_list_init() != true)
        {
            return false;
        }
    }
    if(mac_list_search(mac) != NULL)
    {
        DBG_I("This scanned device is already existed \r\n");
        return true;
    }
    mac_list_t *node = NULL;
    mac_list_t *p = NULL;
    for(p = m_mac_list;p->next != NULL;p=p->next)
    {
        ;
        //DBG_I("linklist address p = 0x%x, p->next = 0x%x\r\n",p,p->next);
    }
    node = (mac_list_t *)pvPortMalloc(sizeof(mac_list_t));;
    if(node == NULL)
    {
        DBG_I("apply memory for new list node failed\r\n");
        return false;
    }
    //node->mac.addr_id_peer = mac->addr_id_peer;
    //node->mac.addr_type    = mac->addr_type;
    memcpy(node->addr,mac,6);

    p->next = node;
    node->next = NULL;
    mac_addr_num++;
    DBG_D("\r\nInsert No. %d device successed:",mac_addr_num);
    DBG_D("%02x:%02x:%02x:%02x:%02x:%02x\r\n",\
            mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    return true;
}
static bool mac_list_delete(mac_list_t *node)
{
    if(node == NULL)
    {
        DBG_E("list_delete: The node pointer is NULL\r\n");
        return false;
    }
    mac_list_t *pos_front = m_mac_list;
    for(;!pos_front->next;pos_front = pos_front->next)
    {
        if(pos_front->next == node)
        {
            break;
        }
    }
    if(pos_front->next != node)
    {
        DBG_E("list_delete: not found the node to delete\r\n");
        return false;       
    }
    pos_front->next = node->next;
    DBG_I("list_delete: delete node %02x:%02x:%02x:%02x:%02x:%02x succeed\r\n",
            node->addr[0],node->addr[1],node->addr[2],node->addr[3],node->addr[4],node->addr[5]);
    vPortFree(node);
    node = NULL;
    mac_addr_num--;
    return true;
}
static int mac_list_all_delete(mac_list_t *node)
{
    if(node == NULL)
    {
        return 0;
    }
    if(!mac_list_all_delete(node->next))
    {
        memset(node,0,sizeof(mac_list_t));
        vPortFree(node);
        node = NULL;
    }
    return 0;
}
/*
static bool mac_list_all_delete(void)
{
    mac_addr_num = 0;
    if(m_mac_list->next == NULL)
    {
        return true;
    }
    mac_list_t *node = m_mac_list->next;
    mac_list_t *temp = m_mac_list->next;
    m_mac_list->next = NULL;
    do
    {
        temp = temp->next;  //save next node
        DBG_I("delete dev %02x:%02x:%02x:%02x:%02x:%02x\r\n",
        node->addr[0],node->addr[1],node->addr[2],node->addr[3],node->addr[4],node->addr[5]);
        memset(node,0,sizeof(mac_list_t));
        vPortFree(node);
        //node->next = NULL;
        node = temp;        //point to next node
    }while(temp);
    return true;
}*/
mac_list_t * get_next_addr(void)
{
    if((m_mac_list == NULL) || (m_mac_list->next == NULL))
    {
        DBG_I("get_next_addr: not init or no addr\r\n");
        return NULL;
    }
    if(p_read_mac == NULL)
    {
        DBG_I("get_next_addr: p_read_mac is invalid state NULL\r\n");
        return NULL;
    }
    p_read_mac = p_read_mac->next;
    return p_read_mac;
}
mac_list_t * get_addr_behind(uint8_t const *addr)
{
    if(addr == NULL)
    {
        DBG_E("get_addr_behind: addr pointer is NULL\r\n");
        return NULL;
    }
    mac_list_t *p_prev_addr;

    p_prev_addr = mac_list_search(addr);
    if(p_prev_addr == NULL)
    {
        DBG_E("get_addr_behind: no previous addr given searched\r\n");
        return NULL;
    }
    if(p_prev_addr->next == NULL)
    {
        DBG_E("get_addr_behind: behind of given addr is no target\r\n");
        return NULL;
    }

    return p_prev_addr->next;
}

task_data_t * get_slave_settings(void)
{   
    if(slave_settings_len == 0)
    {
        return NULL;
    }
    static task_data_t settings;
    settings.p_data = slave_settings_buf;
    settings.length = slave_settings_len;
    return &settings;
}
static void setting_response(proto_router_execute_state_t status, TaskHandle_t task_handle)
{
    static uint8_t buf[200];
    pkg_prot_head_t *prot_head = (pkg_prot_head_t *)buf;
    pkg_prot_frame_t *prot_frame = (pkg_prot_frame_t *)buf;
    uint16_t len = 0;
    char mac_addr[15];

    memset(mac_addr,0,sizeof(mac_addr));
    uint8_t addr[8];
    //ble_s->ops->get_mac(ble_s,addr,sizeof(addr));
    HexToStr(mac_addr,addr,6);

    len = opt_status_encode(prot_frame->p_prot_data,sizeof(buf)-sizeof(pkg_prot_head_t),status,mac_addr);

    prot_head->prot_version = 1;
    prot_head->vendor_id = 1;
    prot_head->option_encryption = 0;
    prot_head->reserve = 0;
    prot_head->cmd_type = PROTOCOL_V2_HEADER_TYPE_TYPE_ROUTER_RSP;
    prot_head->crc16 = crc16_compute((const uint8_t *)prot_head,sizeof(pkg_prot_head_t)-2,NULL);
    prot_head->crc16 = crc16_compute((const uint8_t *)prot_frame->p_prot_data,len,&prot_head->crc16);
    len += sizeof(pkg_prot_head_t);

    task_data_t app_data;     
    task_trig_t task_notify;
    app_data.p_data = buf;
    app_data.length = len;
    task_notify.req_type = TASK_REQUEST_RESPONSE;
    task_notify.p_content = &app_data;
    task_notify.handle = NULL;

    xTaskNotify(task_handle, (uint32_t)&task_notify, eNoAction);
}
bool judge_rights(void * arg)
{    
    if(!arg)
    {
        DBG_E("judge_rights: pointer arg is NULL");
        return false;
    }
    if(rights_mode == 0)
    {
        //not need rights
        return true;
    }
    uint8_t *addr = (uint8_t *)arg;
    mac_list_t *slave = mac_list_search(addr);
    if(!slave)
    {
        DBG_E("judge_rights: %02x:%02x:%02x:%02x:%02x:%02x no rights to be managed",\
                addr[0],addr[1],addr[2],addr[3],addr[4],addr[5]);
        return false;
    }
    DBG_D("judge_rights: %02x:%02x:%02x:%02x:%02x:%02x rights to be managed",\
            addr[0],addr[1],addr[2],addr[3],addr[4],addr[5]);
    return true;
}

/**
* @functionname : user_app_setting_parse 
* @description  : function for update the application perform parameters, @ref user_setting_info_t
* @input        : p_set_rsp_s: decoded message,  
*               : p_set_info_t: all perform parameter data
* @output       : none 
* @return       : true if successed
*/
// static void setting_apply(proto_router_setting_req_t *p_proto,setting_params_t *p_setting)
// {
//     if(p_proto == NULL)
//     {
//         return;
//     }
//     if(p_proto.has_TimeStamp)
//     {
//         hal_rtc_set_time(p_proto.TimeStamp);
//     }
    
//     m_router.id = p_proto.RouterId;

//     //m_router.requester = p_setting->Requester;     //not attention the requester is phone or net
// }

static bool setting_decode( proto_router_setting_req_t * const p_set_cmd, 
                            setting_params_t    * const p_set_out,
                            uint8_t         const  *p_buf, 
                            int32_t         const len)
{
    pb_istream_t    i_stream ;  
    bool    ret;    

    p_set_cmd->TargetMac.funcs.decode          = decode_repeated_var_string;
    p_set_cmd->TargetMac.arg                   = p_set_out->mac;

    p_set_cmd->TargetName.funcs.decode          = decode_repeated_var_string;
    p_set_cmd->TargetName.arg                   = p_set_out->name;

    p_set_cmd->Ip.funcs.decode          = decode_repeated_var_string;
    p_set_cmd->Ip.arg                   = p_set_out->ip;
       
    i_stream = pb_istream_from_buffer(p_buf,len);
    ret = pb_decode(&i_stream,proto_router_setting_req_fields,p_set_cmd); 
    DBG_I("decode settings is %s\r\n", (ret? "successed":"failed"));
    return ret;
}
/**
* @functionname : app_setting_rsp_parse_handle 
* @description  : function for parsing the received message, include decode, and enforce the settings for parameters
* @input        : p_buf: received protocal-buffer message,  
*               : len: message length
* @output       : none 
* @return       : true if successed
*/
bool setting_parse(void const * const p_buf, int32_t const len)
{
    setting_params_t            setting_params;
    proto_router_setting_req_t      p_proto;
    //char slave_mac[13];
    //char dev_name[32];

    //memset(&dev_name,0, sizeof(dev_name));
    //memset(&slave_mac,0, sizeof(slave_mac));
    memset(&setting_params,0, sizeof(setting_params));
    memset(&p_proto,0, sizeof(p_proto));
    
    //bool ret = true;
    DBG_I("decoding setting message\r\n");
    //setting_params.slv_mac = slave_mac;
    if(setting_decode(&p_proto, &setting_params, p_buf, len) != true)
    {
        DBG_I("decode setting is failed\r\n");
        return false;
    }
#if ROUTER_ID
    char device_id[13] = {0};
    uint32_t msgtoken = 0;
    app_iden_get_device_id_char(device_id);
    uint16_t give_random = asrp_pbd_set_rsp_t.Iden.MsgToken & 0x0ffff;
    
    if(asrp_set_info_t.iden.dev_id[0] != '\0')
    {
        if(memcmp(asrp_set_info_t.iden.dev_id,device_id,12) != 0)    
        {
            DBG_I("\r\norigin parse: device id is error\r\n");
            return 0;
        }    
        msgtoken = app_get_msg_token(asrp_set_info_t.iden.dev_id,give_random);
    }
    else
    {
        msgtoken = app_get_msg_token(device_id,give_random);
    }
    if(msgtoken != asrp_pbd_set_rsp_t.Iden.MsgToken)
    {
        DBG_I("\r\norigin parse: check msgtoken is error\r\n");
        return 0;
    }
#endif //verify router id or token
    DBG_I("setting parameters apply\r\n");
    bool ret = true;
    if(p_proto.has_TimeStamp)
    {
        ret &= hal_rtc_set_time(p_proto.TimeStamp);
    }
    if(p_proto.has_AddOrDel)
    {
        uint8_t addr[8];
        mac_list_t *node = NULL;
        StrToHex(addr,setting_params.mac,12);
        switch(p_proto.AddOrDel)
        {
            case SETTING_ADD_ONE_DEV:
                ret &= mac_list_back_insert(addr);
                break;
            case SETTING_DEL_ONE_DEV:
                node = mac_list_search(addr);
                if(node)
                {
                    ret &= mac_list_delete(node);
                }
                else
                {
                    ret = false;
                }
                break;
            case SETTING_DEL_ALL_DEV:
                ret &= mac_list_all_delete(m_mac_list->next);
                break;
            default:
                DBG_E("setting invalid add or delete command\r\n");
                break;
        }
    }
    if(setting_params.name[0] != 0)
    {
        ret &= set_dev_name(setting_params.name);
    }
    if(p_proto.has_Mode)
    {
        if((p_proto.Mode > SETTING_RIGHTS_MIN) && (p_proto.Mode < SETTING_RIGHTS_MAX))
        {
            rights_mode = p_proto.Mode;
            ret &= true;
        }
        else
        {
            ret = false;
        }
    }
    if(p_proto.has_Port)
    {
        ret &= net_set_ip_port(setting_params.ip,p_proto.Port);
    }
    return true;
}
void setting_handler(void *arg, TaskHandle_t task_handle)
{
    task_data_t *p_param = (task_data_t *)arg;
    pkg_prot_head_t *prot_head = (pkg_prot_head_t *)p_param->p_data;
    pkg_prot_frame_t *p_proto = (pkg_prot_frame_t *)p_param->p_data;

    uint16_t give_crc16 = prot_head->crc16;
    uint16_t cacul_crc16 = crc16_compute((const uint8_t *)prot_head,sizeof(pkg_prot_head_t)-2,NULL);
    cacul_crc16 = crc16_compute((const uint8_t *)p_proto->p_prot_data,prot_head->frame_len,&cacul_crc16);
    if(cacul_crc16 != give_crc16)
    {
        return;
    }
    if(prot_head->cmd_type == PROTOCOL_V2_HEADER_TYPE_TYPE_SETTING_RSP)
    {
        slave_settings_len = p_param->length;
        memcpy(slave_settings_buf,p_param->p_data,slave_settings_len);
    }
    else if(prot_head->cmd_type == PROTOCOL_V2_HEADER_TYPE_TYPE_ROUTER_REQ)
    {
        if(setting_parse(p_proto->p_prot_data,p_proto->prot_head.frame_len))
        {
            setting_response(PROTO_ROUTER_EXECUTE_STATE_SUCCESSED,task_handle);
        }
        else
        {
            setting_response(PROTO_ROUTER_EXECUTE_STATE_FAILED,task_handle);
        }
    }
}
bool rights_identity(void * arg, TaskHandle_t task_handle)
{
    task_data_t *p_param = (task_data_t *)arg;
    static task_trig_t task_trig;
    mac_list_t * slave = NULL;

    slave = mac_list_search(p_param->p_data);

    task_trig.req_type = (slave!=NULL)? TASK_REQUEST_IS_RIGHTS: TASK_REQUEST_NO_RIGHTS;
    task_trig.p_content = NULL;
    xTaskNotify(task_handle, (uint32_t)&task_trig, eSetBits);
    return true;
}
void setting_rights_handle_task(void *arg)
{
    DBG_I("setting_rights_handle_task startup");
    static task_trig_t *notify_value;
    //static task_data_t app_data;
    //static buf[BUF_SIZE];
    //static uint32_t len;
    //app_data.p_data = buf;
    mac_list_init();
    /////////////////////////////////////////////////for test
    uint8_t addr1[8] = {0x83,0x49,0x5D,0xCD,0xF0,0xDB};
    uint8_t addr2[8] = {0x25,0x8C,0x5D,0xEF,0x88,0xE3};
    uint8_t addr3[8] = {0x7F,0x55,0x40,0x47,0x63,0xDB};
    mac_list_back_insert(addr1);
    mac_list_back_insert(addr2);
    mac_list_back_insert(addr3);

    mac_list_all_delete(m_mac_list->next);
    m_mac_list->next = NULL;
    ////////////////////////////////////////////////
    while(1)
    {
        if(xTaskNotifyWait( 0,0,(uint32_t*)&notify_value,portMAX_DELAY ) == pdPASS)
        {
            DBG_I("st-rt task notitied type %d from task \"%s\"",notify_value->req_type,pcTaskGetName(notify_value->handle));
            switch(notify_value->req_type)
            {
                case TASK_REQUEST_SETTING:
                    DBG_I("TASK_REQUEST_SETTING");
                    //app_data.length = ((task_data_t *)notify_value.p_content)->length;
                    //app_data.length = (app_data.length > sizeof(buf))?sizeof(buf):app_data.length;
                    //memcpy(buf,((task_data_t *)notify_value.p_content)->p_data,app_data.length);
                    setting_handler(notify_value->p_content,*(TaskHandle_t*)notify_value->handle);
                    break;
                case TASK_REQUEST_RITGHTS_IDENTITY:
                    DBG_I("TASK_REQUEST_RITGHTS_IDENTITY");
                    rights_identity(notify_value->p_content,*(TaskHandle_t*)notify_value->handle);
                    break;
                default :
                    DBG_I("setting & rights: invalid req type %d",notify_value->req_type);
                    break;
            }
        }
    }
}
void create_setting_rights_task(void)
{
    if(xTaskCreate(setting_rights_handle_task, "strt\0", 512, NULL, 1, &setting_rights_handle ) != pdPASS)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }  
}

TaskHandle_t get_setting_handle(void)
{
    return setting_rights_handle;
}

//end
