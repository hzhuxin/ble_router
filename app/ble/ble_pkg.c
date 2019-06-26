/**
 * @brief 
 * 
 * @file ble_pkg.c
 * @author Chenfei
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.
 * All Rights Reserved.
 * @date 2018-07-12
 */
#include "stdlib.h"
#include "ble_pkg.h"
#include "debug.h"
#include "crc16.h"

DBG_SET_LEVEL(DBG_LEVEL_N);

static uint32_t ble_get_new_sub_frame(uint8_t **p_dst_final, 
                                           uint8_t const *p_src_new_frame, 
                                           uint8_t frame_len)
{
    if(!*p_dst_final || !p_src_new_frame)
    {
        DBG_I("NULL Pointer\r\n");
        return 0;
    }
    memcpy(*p_dst_final,p_src_new_frame,frame_len);
    *p_dst_final += frame_len;
    return  frame_len;
}
uint32_t ble_get_valid_field(uint8_t *p_dst, uint8_t const *p_src, uint32_t length)
{
    if(!p_dst|| !p_src)
    {
        DBG_I("\r\np_dest or p_data is NULL\r\n");
        return 0;
    }
    
    ble_pkg_head_t *ble_pkg_head = (ble_pkg_head_t *) p_src;
    if(length < (sizeof(ble_pkg_head_t)+ble_pkg_head->data_len))
    {
        DBG_I("\r\ncommand length is not enough\r\n");
        return 0;
    }
    //ble_data_trans_head_t *p_head = (ble_data_trans_head_t *)p_frame;
    uint16_t *p_giv_crc = (uint16_t *)((void *)(p_src + sizeof(ble_pkg_head_t) + ble_pkg_head->data_len));
    uint16_t crc_cal = crc16_compute(p_src, sizeof(ble_pkg_head_t) + ble_pkg_head->data_len,NULL);
    if( crc_cal != *p_giv_crc )
    {
        DBG_I("\r\nCrc is wrong, given crc = 0x%x , calcrc = 0x%x \r\n",*p_giv_crc,crc_cal);
        DBG_I("\r\nCrc caculate len = %d \r\n",sizeof(ble_pkg_head_t) + ble_pkg_head->data_len);
        return 0;
    }
    memcpy(p_dst, p_src + sizeof(ble_pkg_head_t),ble_pkg_head->data_len);
    return ble_pkg_head->data_len;
}
uint16_t ble_combine_sub_frame(uint8_t *p_dest_frame, const uint8_t *p_split_frame, int16_t split_len)
{
    if(split_len == 0)
    {
        DBG_I("ble length == 0\r\n");
        return 0;
    }
    if(!p_split_frame || !p_dest_frame)
    {
        DBG_I("ble_combine_sub_frame pointer is NULL\r\n");
        return 0;
    }
    int16_t frame_len = 0;
    split_head_t *p_split_head = (split_head_t *)(p_split_frame);
    static construct_mode_t current_construct_mode = CONSTRUCT_MODE_INVALID;
    static uint8_t  *p_current_pos = NULL;

    DBG_I("p_split_head->type is: %d\r\n", p_split_head->type);

    switch(p_split_head->type)
    {
        case SPLIT_FRAME_TYPE_MIDDLE:
        {
            //DBG_I("this is middle frame type\r\n");

            if(current_construct_mode == CONSTRUCT_MODE_CONSTRUCTING)
            {
                // calucate current positon to story
                // copy data to dest
                ble_get_new_sub_frame(&p_current_pos,
                                                  p_split_frame + sizeof(split_head_t),
                                                  split_len - sizeof(split_head_t)
                                                 );
                DBG_I("middle frame len is:%d\r\n",p_current_pos - p_dest_frame );
            }
        }
            break;
        case SPLIT_FRAME_TYPE_START:
        {
            //DBG_I("this is first frame type\r\n");
            current_construct_mode = CONSTRUCT_MODE_CONSTRUCTING;
            p_current_pos = p_dest_frame;
            ble_get_new_sub_frame(&p_current_pos,
                                                  p_split_frame + sizeof(split_head_t),
                                                  split_len - sizeof(split_head_t)
                                                 );            
            DBG_I("first frame len is:%d\r\n", p_current_pos - p_dest_frame);
            // update destination
            break;
        }
        
        case SPLIT_FRAME_TYPE_LAST:
        {
            //DBG_I("this is last frame type\r\n");
            if(current_construct_mode == CONSTRUCT_MODE_CONSTRUCTING)
            {
                // calucate current positon to story
                // copy data to dest
                ble_get_new_sub_frame(&p_current_pos,
                                                    p_split_frame + sizeof(split_head_t),
                                                    split_len - sizeof(split_head_t)
                                                    );
                current_construct_mode = CONSTRUCT_MODE_CONSTRUCTED;                            
                frame_len = p_current_pos - p_dest_frame;
                DBG_I("lase frame len is:%d\r\n", frame_len);
            }
        }
            break;
        case SPLIT_FRAME_TYPE_SINGLE:
        {
            //DBG_I("this is total frame type\r\n");
            p_current_pos = p_dest_frame;
            memcpy(p_current_pos, p_split_frame + sizeof(split_head_t), split_len - sizeof(split_head_t));
            current_construct_mode = CONSTRUCT_MODE_CONSTRUCTED;
            frame_len = split_len - sizeof(split_head_t);           
            break;
        }
        default:
            break;
    }
    DBG_I("total frame len is:%d\r\n", frame_len);
    return frame_len;
}
uint32_t ble_package(uint8_t * const p_dst,
                    uint8_t const *p_src, 
                     uint32_t const dst_size,
                     uint32_t const src_len)
{
    if(p_dst == NULL)
    {
        DBG_I("ble package out pointer is NULL\r\n");
        return 0;
    }
    if((sizeof(ble_pkg_head_t)+src_len+2) > dst_size)
    {       
        DBG_I("ble package length is more than out size\r\n");
        return 0;
    }
    ble_pkg_t   ble_pkg;
    static uint8_t seq = 0;
    seq++;
    if(seq == 0)
    {
        seq++;
    }
    ble_pkg.head.sequence  = seq;
    ble_pkg.head.func_code = CMD;
    ble_pkg.head.data_len  = src_len;    
    ble_pkg.frame          = (uint8_t *)p_src;

    memcpy(p_dst,(uint8_t *)&ble_pkg.head,sizeof(ble_pkg.head));
    memcpy(p_dst+sizeof(ble_pkg.head),(uint8_t *)ble_pkg.frame,src_len);

    uint16_t crc16 = crc16_compute(p_dst,src_len + sizeof(ble_pkg.head),NULL);
    memcpy(p_dst+sizeof(ble_pkg.head)+src_len,(uint8_t *)&crc16,2);

    uint32_t out_len = (src_len + sizeof(ble_pkg.head) + 2);
    DBG_I("ble package out len = %d\r\n",out_len);
    return out_len;
    //return (src_len + sizeof(ble_pkg.head) + 2);
}
/**
* @functionname : user_data_pb_bhv_encode 
* @description  : function for splitting the datas to sub package  to send
* @input        : buffer: pointor to data buf, length: data length will be sent
* @output       : none 
* @return       : true if successed
*/
uint16_t get_sub_split( uint8_t *const dst_buf,  
                        uint8_t const * const src_buf,
                        uint16_t const curren_len,
                        uint16_t const total_len,
                        uint16_t const split_len)
{
    uint16_t valid_len;
    
    if(curren_len > total_len || \
       curren_len == 0 || \
       total_len  == 0 || \
       split_len == 0)
    {
        return 0;
    }
    if(src_buf == NULL || dst_buf == NULL)
    {
        return 0;
    }
    //add header and copy data
    if(curren_len > split_len)
    {
        dst_buf[0] = (curren_len == total_len)? SPLIT_FRAME_TYPE_START : SPLIT_FRAME_TYPE_MIDDLE;
        valid_len = split_len;
    }
    else
    {        
        //only (or remain) signle package
        dst_buf[0] = (curren_len == total_len)? SPLIT_FRAME_TYPE_SINGLE : SPLIT_FRAME_TYPE_LAST;
        valid_len = curren_len;        
    }
    memcpy(&dst_buf[1], src_buf, valid_len); 
    return valid_len;
    
}