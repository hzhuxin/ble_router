#ifndef BLE_PKG_H
#define BLE_PKG_H
#include <stdint.h>
#include <stdbool.h>

//ble head default
//#define     SEQ         1       //0x01-0xff
#define     CMD         1       //0x01-0x7f

enum split_frame_type
{
    SPLIT_FRAME_TYPE_MIDDLE = 0x00,
    SPLIT_FRAME_TYPE_START= 0x01,
    SPLIT_FRAME_TYPE_LAST = 0x02,
    SPLIT_FRAME_TYPE_SINGLE = 0x03
};

typedef enum 
{
    CONSTRUCT_MODE_INVALID =0,
    CONSTRUCT_MODE_CONSTRUCTING ,
    CONSTRUCT_MODE_CONSTRUCTED
}construct_mode_t;

typedef struct 
{
    uint8_t type:4; // low 4 bit is type
    uint8_t version:4;// high 4 bit is version
}split_head_t;

typedef struct
{
    uint16_t len;
    uint8_t  *dt;
}ble_rsp_t;

typedef struct
{
    uint8_t     sequence;                        //the sequence filed in the data struct, it will be change continous with adding 1 of next complet data
    uint8_t     func_code;                        //the command filed in the data struct, it is fixed.
    uint16_t    data_len;                   //the datafiled length, not including seq, cmd and crc
}ble_pkg_head_t;
typedef struct 
{
    ble_pkg_head_t  head;
    uint8_t        *frame;                  //include data and crc16
}ble_pkg_t;

uint32_t ble_get_valid_field(uint8_t *p_dst, uint8_t const *p_src, uint32_t length);
uint16_t ble_combine_sub_frame(uint8_t *p_dest_frame, 
                               const uint8_t *p_split_frame, 
                               int16_t split_len);
uint32_t ble_package(uint8_t * const p_dst,uint8_t const *p_src, 
                     uint32_t const dst_size,uint32_t const src_len);

uint16_t get_sub_split( uint8_t *const dst_buf,  
                        uint8_t const * const src_buf,
                        uint16_t const curren_len,
                        uint16_t const total_len,
                        uint16_t const split_len);

#endif //BLE_PKG_H

