/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: Estrus.proto */

#ifndef PROTOBUF_C_Estrus_2eproto__INCLUDED
#define PROTOBUF_C_Estrus_2eproto__INCLUDED

#include <protobuf-c/protobuf-c.h>

PROTOBUF_C__BEGIN_DECLS

#if PROTOBUF_C_VERSION_NUMBER < 1000000
# error This file was generated by a newer version of protoc-c which is incompatible with your libprotobuf-c headers. Please update your headers.
#elif 1003000 < PROTOBUF_C_MIN_COMPILER_VERSION
# error This file was generated by an older version of protoc-c which is incompatible with your libprotobuf-c headers. Please regenerate this file with a newer version of protoc-c.
#endif

#include "IdentityMsg.pb-c.h"

typedef struct _Protocol__EstrusReq Protocol__EstrusReq;
typedef struct _Protocol__Estrus Protocol__Estrus;


/* --- enums --- */


/* --- messages --- */

struct  _Protocol__EstrusReq
{
  ProtobufCMessage base;
  Protocol__IdentityMsg *iden;
  size_t n_estrusinfo;
  Protocol__Estrus **estrusinfo;
};
#define PROTOCOL__ESTRUS_REQ__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&protocol__estrus_req__descriptor) \
    , NULL, 0,NULL }


struct  _Protocol__Estrus
{
  ProtobufCMessage base;
  /*
   * unix timestamp, 只取小时，服务器忽略分钟，秒
   */
  protobuf_c_boolean has_timestamp;
  uint32_t timestamp;
  /*
   * 活动百分比, 每小时
   */
  protobuf_c_boolean has_activity;
  int32_t activity;
  /*
   * odba, 每小时60个ODBA的平均值
   */
  protobuf_c_boolean has_odba;
  int32_t odba;
};
#define PROTOCOL__ESTRUS__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&protocol__estrus__descriptor) \
    , 0, 0u, 0, 0, 0, 0 }


/* Protocol__EstrusReq methods */
void   protocol__estrus_req__init
                     (Protocol__EstrusReq         *message);
size_t protocol__estrus_req__get_packed_size
                     (const Protocol__EstrusReq   *message);
size_t protocol__estrus_req__pack
                     (const Protocol__EstrusReq   *message,
                      uint8_t             *out);
size_t protocol__estrus_req__pack_to_buffer
                     (const Protocol__EstrusReq   *message,
                      ProtobufCBuffer     *buffer);
Protocol__EstrusReq *
       protocol__estrus_req__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   protocol__estrus_req__free_unpacked
                     (Protocol__EstrusReq *message,
                      ProtobufCAllocator *allocator);
/* Protocol__Estrus methods */
void   protocol__estrus__init
                     (Protocol__Estrus         *message);
size_t protocol__estrus__get_packed_size
                     (const Protocol__Estrus   *message);
size_t protocol__estrus__pack
                     (const Protocol__Estrus   *message,
                      uint8_t             *out);
size_t protocol__estrus__pack_to_buffer
                     (const Protocol__Estrus   *message,
                      ProtobufCBuffer     *buffer);
Protocol__Estrus *
       protocol__estrus__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   protocol__estrus__free_unpacked
                     (Protocol__Estrus *message,
                      ProtobufCAllocator *allocator);
/* --- per-message closures --- */

typedef void (*Protocol__EstrusReq_Closure)
                 (const Protocol__EstrusReq *message,
                  void *closure_data);
typedef void (*Protocol__Estrus_Closure)
                 (const Protocol__Estrus *message,
                  void *closure_data);

/* --- services --- */


/* --- descriptors --- */

extern const ProtobufCMessageDescriptor protocol__estrus_req__descriptor;
extern const ProtobufCMessageDescriptor protocol__estrus__descriptor;

PROTOBUF_C__END_DECLS


#endif  /* PROTOBUF_C_Estrus_2eproto__INCLUDED */
