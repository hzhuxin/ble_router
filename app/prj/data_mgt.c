/**
 * 
 */

#include "app_freertos.h"
#include "string.h"
#include "net_manage.h"
#include "init.h"
#include "hal_cfg.h"
#include "hal_pin.h"
#include "hal_cpu.h"
#include "debug.h"
#include "ble_c_app.h"
#include "HexStr.h"
#include "hal_rtc.h"
#include "hal_uart.h"
#include "data_mgt.h"
#include "data_cache.h"

#define XFER_JSON_MAX_CHAR_LEN  (NET_QUEUE_DEFAULT_LENGTH)
#define XFER_JSON_UNIT_CHAR_LEN (strlen(gateway)+strlen(timestamp)+strlen(vol)+strlen(devices)+\
                                 strlen(device)+strlen(temp)+strlen(bat)+strlen(rssi)+strlen(timestamp2)+\
                                 60)
#define XFER_SLV_DEFAULT_NUM    (XFER_JSON_MAX_CHAR_LEN/sizeof(slv_msg_t)-1)
#define XFER_SLV_BYTES          (XFER_SLV_DEFAULT_NUM* sizeof(slv_msg_t))  

//for test
#define TEST 0

DBG_SET_LEVEL(DBG_LEVEL_D);

static const char gateway[] = "{\"gateway\":\"\0";
static const char timestamp[] = "\",\"timestamp\":\0";
static const char vol[] = ",\"battery\":\0";
static const char devices[] = ",\"devices\":\0";
static const char device[] = "\"device\":\"\0";
static const char temp[] = "\",\"temperature\":\0";
//static const char bat[] = "\0";
static const char rssi[] = ",\"rssi\":\0";
static const char timestamp2[] = ",\"timestamp\":\0";
/*{"gateway":"112233332211","timestamp":12345678912,"battery":28,
"devices":[{"device":"112233445566","temperature":3926,"battery":35,"rssi":109,"timestamp":12345678912}]}*/
static TaskHandle_t     data_manage_task_handle = NULL; 
static QueueHandle_t    msg_queue = NULL;

static int32_t int_bits(int32_t dt)
{
    int32_t i=0;

    i = (dt < 0)? (1):(0);

    while(dt)
    {
        dt /= 10;
        i++;
    }
    return i;
}
static int32_t uint_bits(uint32_t dt)
{
    int32_t i=0;
    while(dt)
    {
        dt /= 10;
        i++;
    }
    return i;
}
#if TEST
static const router_msg_t router  = 
{
    .mac = {0x11,0x11,0x11,0x11,0x11,0x11},
    .vol = 3970,
    .timestamp = 123456789,
};
static int32_t update_router_msg(router_msg_t *msg)
{
    memcpy(msg,&router,sizeof(router_msg_t));
    return sizeof(router_msg_t);
}
#else //TEST
static int32_t update_router_msg(router_msg_t *msg)
{
    memcpy(&msg->mac,hal_cpu_get_mac(),6);
    msg->timestamp = hal_rtc_get_time();
    msg->vol = 4000;
    return sizeof(router_msg_t);
}
#endif //TEST
static int32_t router_msg_head_to_json(char *json,int32_t size)
{
    if(!json || size < 120)
    {
        return -1;
    }
    char *p = json;
    //uint32_t mses;
    router_msg_t router;
    update_router_msg(&router);
    router_msg_t *msg = &router;
    strcpy(p,gateway);
    p += strlen(gateway);
    sprintf(p,"%02x%02x%02x%02x%02x%02x",msg->mac[0],msg->mac[1],msg->mac[2],msg->mac[3],msg->mac[4],msg->mac[5]);
    p += 12;
    strcpy(p,timestamp);
    p += strlen(timestamp);
    sprintf(p,"%u",msg->timestamp);
    p += int_bits(msg->timestamp);
    //mses = xTaskGetTickCount() % 1000;
    sprintf(p,"%u",(xTaskGetTickCount() % 1000));
    p += 3;
    strcpy(p,vol);
    p += strlen(vol);
    sprintf(p,"%u",msg->vol);
    p += int_bits(msg->vol);
    strcpy(p,devices);
    p += strlen(devices);

    return (p-json);
}
static int32_t router_msg_end_to_json(char *json,int32_t size)
{
    if(!json || size < 1)
    {
        DBG_E("Dgt: router_msg_end_to_json pointer is NULL");
        return -1;
    }
    *json = '}';
    return 1;
}
static int32_t dev_msg_to_json(char *json,int32_t size,slv_msg_lst_t *dev_msg[], int32_t devs)
{
    if(!json || !dev_msg || devs <= 0 || size < 120)
    {
        DBG_E("Dgt: json = 0x%x\r\n",json);
        return -1;
    }
    
    int32_t i=0,cnt=0;
    char *p = json;
    p[i++] = '[';
    while(cnt < devs)
    {
        slv_msg_t *msg = &dev_msg[cnt]->data;
        if(cnt > 0)
        {
            p[i++] = ',';
        }
        p[i++] = '{';
        //device mac
        strcpy(&p[i],device);
        i += strlen(device);
        sprintf(&p[i],"%02x%02x%02x%02x%02x%02x",msg->mac[0],msg->mac[1],msg->mac[2],msg->mac[3],msg->mac[4],msg->mac[5]);
        i += 12;
        //temptrue
        strcpy(&p[i],temp);
        i += strlen(temp);
        sprintf(&p[i],"%d",msg->temp);
        i += int_bits(msg->temp);
        //battary
        strcpy(&p[i],vol);
        i += strlen(vol);
        sprintf(&p[i],"%u",msg->vol);
        i += int_bits(msg->vol);
        //rssi
        strcpy(&p[i],rssi);
        i += strlen(rssi);
        sprintf(&p[i],"%d",msg->rssi);
        i += int_bits(msg->rssi);
        //timestamp
        strcpy(&p[i],timestamp2);
        i += strlen(timestamp2);
        sprintf(&p[i],"%u",msg->timestamp);
        //i += uint_bits(msg->timestamp);
        i += 10;
        //mill second
        sprintf(&p[i],"%03u",msg->mill_secs%1000);
        //i += uint_bits(msg->mill_secs%1000);
        i += 3;

        p[i++] = '}';
        cnt++;
    }
    p[i++] = ']';
    return (i);
}
int32_t get_dev_data(void * buf,int32_t size,slv_msg_lst_t *dev_node[], int32_t dev_num)
{
    if(!buf || dev_node || !dev_num)
    {
        return -1;
    }
    slv_msg_t *p_dev = (slv_msg_t *)buf;
    while(dev_num > 0)
    {
        if(size < sizeof(slv_msg_t))
        {
            break;
        }
        memcpy(p_dev->mac,(*dev_node)->data.mac,6);

        p_dev->vol = (*dev_node)->data.vol;
        p_dev->temp = (*dev_node)->data.temp;
        p_dev->timestamp = (*dev_node)->data.timestamp;
        p_dev->rssi = (*dev_node)->data.rssi;

        dev_num--;
        p_dev += sizeof(slv_msg_t);
        dev_node ++;

    }
    return ((uint8_t *)p_dev - (uint8_t *)buf);
}

#if TEST
static slv_msg_lst_t cache[] = 
{
    {.upload_time=0,
     .update_flag=0,
     .data.mac={0x22,0x22,0x22,0x22,0x22,0x22},
     .data.temp = 253,
     .data.vol=4036,
     .data.rssi=105,
     .data.timestamp = 1562422899,
    },
    {.upload_time=0,
     .update_flag=0,
     .data.mac={0x22,0x22,0x22,0x22,0x22,0x22},
     .data.temp = 183,
     .data.vol=3936,
     .data.rssi=124,
     .data.timestamp = 1562422899,
    },
};
static int32_t test_cache_read_list_by_time(slv_msg_lst_t *node[], int32_t number)
{
    if(!node)
    {
        DBG_E("Dgt: need to read number = %d",number);
        return -1;
    }
    DBG_D("Dgt: need to read record number = %d",number);
    int32_t i=0;
    for(;i<number;i++)
    {
        node[i] = &cache[i];
        node[i]->update_flag = 1;
        node[i]->upload_time = hal_rtc_get_time();
        // DBG_D("Dgt:buf[i].mac=%02x:%02x:%02x:%02x:%02x:%02x",
        //               node[i]->data.mac[0],
        //               node[i]->data.mac[1],
        //               node[i]->data.mac[2],
        //               node[i]->data.mac[3],
        //               node[i]->data.mac[4],
        //               node[i]->data.mac[5]);
        // DBG_D("Dgt: buf[%d].temp=%d",i,node[i]->data.temp);
        // DBG_D("Dgt: buf[%d].vol=%d,",i,node[i]->data.vol);
        // DBG_D("Dgt: buf[%d].rssi=%d",i,node[i]->data.rssi);
        // DBG_D("Dgt: buf[%d].time=%d",i,node[i]->data.timestamp);
    }
    return i;
}
#endif
void data_mgt_task_main(void *arg)
{
    DBG_D("Dgt task startup.");
    cache_init();
    int32_t records = 0;

    static slv_msg_lst_t *node[XFER_SLV_DEFAULT_NUM];
    static slv_msg_t dev;
    static msg_t *p_msg = NULL;
    TickType_t upload_time = 0;
    //static xfer_pkg_t *xp_buf = NULL;
    //static slv_msg_t *dev_msgs = NULL;

    p_msg = (msg_t *)pvPortMalloc(XFER_JSON_MAX_CHAR_LEN);
    if(!p_msg)
        {DBG_E("Dgt task startup fail at malloc p_msg.");return;}

    msg_queue = xQueueCreate(10,sizeof(slv_msg_t));
    if(!msg_queue)
        {DBG_E("Dgt task startup faile at create queue.");return;}
    DBG_I("Dgt task startup ok.");
    vTaskDelay(2000);
    while(1)
    {
        memset(p_msg, 0, XFER_JSON_MAX_CHAR_LEN);
        p_msg->len = 0;
        while(xQueueReceive(msg_queue,&dev,pdMS_TO_TICKS(10000)) == pdPASS)
        {
            // DBG_D("Dgt: recieved device msg");
            // DBG_D("Dgt: recv: %02x:%02x:%02x:%02x:%02x:%02x:",
            //     dev.mac[0],dev.mac[1],dev.mac[2],dev.mac[3],dev.mac[4],dev.mac[5]);
            // DBG_D("Dgt: recv dev temp = %d",dev.temp);
            // DBG_D("Dgt: recv dev rssi = %d",dev.rssi);
            cache_insert_list(&dev);
        }
#if TEST
        cache_insert_dev_for_test();
#endif //TEST
        records = cache_read_list_by_time(node,XFER_SLV_DEFAULT_NUM);
        DBG_I("Dgt: read out <%d> devices",records);
        if(records > 0 && records <= XFER_SLV_DEFAULT_NUM)
        {
#if __HTTPS_ENABLE__
            p_msg->len = router_msg_head_to_json((char *)p_msg->data,XFER_JSON_MAX_CHAR_LEN-sizeof(p_msg->len));
            p_msg->len += dev_msg_to_json((char *)&p_msg->data[p_msg->len],XFER_JSON_MAX_CHAR_LEN-sizeof(p_msg->len)-p_msg->len,node,records);
            p_msg->len += router_msg_end_to_json((char *)&p_msg->data[p_msg->len],XFER_JSON_MAX_CHAR_LEN-sizeof(p_msg->len)-p_msg->len);
#endif
            DBG_D("%s",p_msg->data);
            DBG_I("Dgt: uploading...");
            //NRF_LOG_HEXDUMP_INFO(p_msg,p_msg->len+4);
            upload_time = xTaskGetTickCount();
            if(send_msg_to_server(p_msg))
            //if(0)
            {
                DBG_I("Dgt: upload data success within %dS\r\n",(xTaskGetTickCount()-upload_time)/1000);
                cache_list_update_upload_state(node,records);
            }
            else
            {
                DBG_W("Dgt:Upload data fail with timeout %d!",(xTaskGetTickCount()-upload_time)/1000);
            }
            
        }
    }
}
void create_data_mgt_task(void)
{
    if(xTaskCreate( data_mgt_task_main, "DGT", 512, NULL, 1, &data_manage_task_handle ) != pdPASS)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}
void send_msg_to_cache_queue(slv_msg_t *slv)
{
    xQueueSend(msg_queue, (void *)slv, (TickType_t)1);
}
//end
