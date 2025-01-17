/**
 * 
 */
#include "app_freertos.h"
#include "net_manage.h"
#include "init.h"
#include "hal_cfg.h"
#include "hal_pin.h"
#include "debug.h"
#include "ble_c_app.h"
#include "HexStr.h"
#include "hal_rtc.h"
#include "hal_uart.h"
#include "data_mgt.h"
#include "data_cache.h"

#define CACHE_DEFAULT_SIZE      100000
#define MAX_LIST_NUMBER         500
#define DEALAY_UPLOAD_TIME      (15*60)

#define CHECK_LOCK()    \
    do                      \
    {                       \
        if(!cache_lock())   \
        {                   \
            DBG_E("Cache data lock failed");\
            return -1;      \
        }                   \
    } while (0);

#define TEST    0

DBG_SET_LEVEL(DBG_LEVEL_I);

static SemaphoreHandle_t    lock = NULL;
typedef struct
{
    slv_msg_lst_t * head;
    int32_t         cnt;
    slv_msg_lst_t * end;
}cache_slv_list_t;

static cache_slv_list_t    cache_list;

bool cache_lock(void)
{
    if(lock && (xSemaphoreTake(lock,pdMS_TO_TICKS(2000)) == pdTRUE))
    {
        return true;
    }
    return false;
}
bool cache_unlock(void)
{
    //xSemaphoreGive(lock);
    return true;
}
int32_t cache_init(void)
{
    lock = xSemaphoreCreateBinary();
    if(!lock)
    {
        DBG_E("Data cache init failed");
        return -1;
    }
    memset(&cache_list,sizeof(cache_list),0);
    cache_list.head = NULL;
    cache_list.end = NULL;
    cache_list.cnt = 0;
    xSemaphoreGive(lock);
    return 0;
}

#include <stdlib.h>
void cache_generate_dev_for_test(slv_msg_t *dev)
{
    static int dt = 1;
    dev->mac[0] = (rand() * dt & 0xff) + 1;
    dev->mac[1] = (rand() * dt & 0xff) + 2;
    dev->mac[2] = (rand() * dt & 0xff) + 3;
    dev->mac[3] = (rand() * dt & 0xff) + 4;
    dev->mac[4] = (rand() * dt & 0xff) + 5;
    dev->mac[5] = (rand() * dt & 0xff) + 6;
    dev->temp = (dt * 299) % 33;
    dev->rssi = (dt * 666) % 200;
    dev->vol = (dt * 4267) % 4200;
    dev->timestamp = hal_rtc_get_time();
    dt++;
}
void cache_insert_dev_for_test(void)
{
    static slv_msg_t dev;
    for(uint8_t i=0;i<10;i++)
    {
        cache_generate_dev_for_test(&dev);
        cache_insert_list(&dev);
    }
}
int32_t cache_insert_list(slv_msg_t *new_node)
{
    if(cache_list.cnt >= MAX_LIST_NUMBER)
    {
        DBG_W("Cached slave messages are reached max!!!\r\n");
        return -1;
    }
    if(!new_node)
    {
        DBG_E("Cached insert list failure of NULL\r\n");
        return -2;
    }
    //CHECK_LOCK();
    slv_msg_lst_t *node = NULL;
    node = cache_search_list(new_node);
    if(node)
    {
        memcpy(&node->data,new_node,sizeof(slv_msg_t));
        node->update_flag = 1;
        //DBG_I("Cache exist cnt = %d",cache_list.cnt);
        DBG_D("Cache update node[%02x:%02x:%02x:%02x:%02x:%02x] is exit, update flag=1",
                new_node->mac[0],
                new_node->mac[1],
                new_node->mac[2],
                new_node->mac[3],
                new_node->mac[4],
                new_node->mac[5]);
        return 0;
    }
    node = (slv_msg_lst_t *)pvPortMalloc(sizeof(slv_msg_lst_t));
    if(!node)
    {
        DBG_E("Cache malloc node fail\r\n");
        //cache_unlock();
        return -2;
    }
    memcpy(&node->data,new_node,sizeof(slv_msg_t));
    node->update_flag = 1;      //set update flag, clear after upload
    if(cache_list.head == NULL)
    {
        cache_list.head = node;
        cache_list.end = node;
        cache_list.cnt = 1;
    }
    else
    {
        cache_list.end->next = node;
        cache_list.end = node;
        cache_list.cnt++;
    }
    DBG_I("Cache insert device[%02x:%02x:%02x:%02x:%02x:%02x]",
                node->data.mac[0],
                node->data.mac[1],
                node->data.mac[2],
                node->data.mac[3],
                node->data.mac[4],
                node->data.mac[5]);
    DBG_I("Cache list cnt = %d",cache_list.cnt);

    return 0;
}
slv_msg_lst_t * cache_search_list(slv_msg_t *msg)
{
    //CHECK_LOCK();
    slv_msg_lst_t *p_node;
    slv_msg_lst_t *temp = NULL;
    for(p_node = cache_list.head;p_node;p_node = p_node->next)
    {
        if(memcmp(p_node->data.mac,msg->mac,6) == 0)
        {
            temp = p_node;
        }
    }
    //cache_unlock();
    return temp;
}
int32_t cache_replace_list(slv_msg_t *msg)
{
    //CHECK_LOCK();
    slv_msg_lst_t *p_node = NULL;
    p_node = cache_search_list(msg);
    if(p_node)
    {
        if((msg->timestamp - p_node->data.timestamp) < DEALAY_UPLOAD_TIME)
        {
            return -2;
        }
        memcpy(&p_node->data,msg,sizeof(slv_msg_t));
    }
    cache_unlock();
    return 1;
}
int32_t cache_list_update_upload_state(slv_msg_lst_t *node[],int node_num)
{
    if(!node)
    {
        DBG_E("Cache:node is NULL,func: %s line: %d\r\n",__func__,__LINE__);
        return -1;
    }

    if(!cache_list.head)
    {
        DBG_E("Cache:There is no data in cache list,func: %s line: %d\r\n",__func__,__LINE__);
        return -2;
    }
    slv_msg_lst_t *p_list = cache_list.head;
    int32_t i=0;
    for(;(i<node_num && p_list); p_list = p_list->next)
    {
        //if(memcmp(p_list->data.mac,node->data.mac,6) == 0)
        if(p_list == node[i])
        {
            p_list->upload_time = hal_rtc_get_time();
            p_list->update_flag = 0;
            i++;
        }
    }
    DBG_I("Cache update upload time over, %d devices update",i);
    return 0;
}
int32_t cache_read_list_by_time(slv_msg_lst_t *buf[], int32_t number)
{
    if(number <=0)
    {
        DBG_W("Cache read number is illegal or 0\r\n");
        return -1;
    }
    number = (number > cache_list.cnt)? cache_list.cnt:number;

    slv_msg_lst_t *p_list = cache_list.head;
    int32_t i=0;
    for(;(i<number && p_list); p_list = p_list->next)
    {
        DBG_D("Dgt: read dev msg by time: update_timestamp-upload time= %d",(p_list->data.timestamp-p_list->upload_time));
        if((p_list->data.timestamp > p_list->upload_time)&&
           (p_list->data.timestamp - p_list->upload_time) >= DEALAY_UPLOAD_TIME)
        {
            buf[i] = p_list;
            i++;
        }
    }
    //cache_unlock();
    return i;
}
int32_t cache_delete_all_list(void)
{
    //CHECK_LOCK();
    slv_msg_lst_t *p_del;
    for(;cache_list.head;cache_list.head = cache_list.head->next)
    {
        p_del = cache_list.head;
        p_del->next = 0;
        vPortFree(p_del);
    }
    cache_list.head = NULL;
    cache_list.end = NULL;
    cache_list.cnt = 0;
    //cache_unlock();
    return 0;
}
int32_t cache_delete_list(int32_t nodes)
{
    if(!cache_list.head)
    {
        return 0;
    }
    //CHECK_LOCK();
    nodes = (nodes > cache_list.cnt)? cache_list.cnt:nodes;
    slv_msg_lst_t *p_del;
    int32_t i=0;
    for(;i<nodes && cache_list.head;i++)
    {
        p_del = cache_list.head;
        cache_list.head = cache_list.head->next;
        p_del->next = 0;
        vPortFree(p_del);
    }
    if(!cache_list.head)
    {
        cache_list.cnt = 0;
        cache_list.end = 0;
    }
    //cache_unlock();
    return i;
}

//end

