/**
 * @brief 
 * 
 * @file pb_callback.c
 * @date 2018-10-12
 * @author Chenfei
 * @copyright (c) 2016-2018 by Druid Technology Co., Ltd.  All Rights Reserved.
 */
#include <stdint.h>
#include "pb_callback.h"
#include "debug.h"

DBG_SET_LEVEL(DBG_LEVEL_N);
/***********************************************defined functions**********************************************/
 /**
* @functionname : encode_repeated_var_string 
* @description  : function for encode a strings data
* @input        : stream, encode stream,  
*               : fields: the origin fields
*               : str: the string data will be encoded
* @output       : none 
* @return       : true if successed
*/
bool encode_repeated_var_string(pb_ostream_t *stream, const pb_field_t *fields,  void * const *str)
{
    bool res;
    res = pb_encode_tag_for_field(stream, fields);
    //DBG_I("pb encode tag for string is  %s", (res? "successed":"failed"));
    res = pb_encode_string(stream,*str,strlen(*str));
    //DBG_I("encode string \"%s\" is %s\r\n", *str, (res? "successed":"failed"));
    //PRINT_S("pb encode string over\r\n\0");
    return res;
}
/**
* @functionname : app_decode_repeated_var_string 
* @description  : function for decode a strings data
* @input        : stream, encode stream,  
*               : fields: the origin fields
*               : str: the string data will be encoded
* @output       : none 
* @return       : true if successed
*/
bool decode_repeated_var_string(pb_istream_t *stream, const pb_field_t *field, void **str)
{
    pb_byte_t   cache_buf[stream->bytes_left];
    uint8_t     byte_len = stream->bytes_left;
    bool res = false;
    res = pb_read(stream,cache_buf, stream->bytes_left);
    DBG_I("\r\ndecode string \"%s\" %s\r\n",cache_buf,(res? "successed":"failed"));
    if(res == true)
    {
        memcpy(*str, cache_buf, byte_len);
        return true;
    }
    else
    {
        *str = NULL;
        return false;
    }
}
 /**
* @functionname : encode_repeated_var_int 
* @description  : function for encode a int data
* @input        : stream, encode stream,  
*               : fields: the origin fields
*               : str: the string data will be encoded
* @output       : none 
* @return       : true if successed
*/
bool encode_repeated_var_int(pb_ostream_t *stream, const pb_field_t *fields,  void * const *str)
{
    bool res;
    res = pb_encode_tag_for_field(stream, fields);
    DBG_I("pb encode tag for string is  %s", (res? "successed":"failed"));
    res = pb_encode_string(stream,*str,strlen(*str));
    DBG_I("encode string \"%s\" is %s\r\n", *str, (res? "successed":"failed"));
    return res;
}