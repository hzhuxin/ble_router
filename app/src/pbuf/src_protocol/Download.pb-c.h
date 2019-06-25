/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: Download.proto */

#ifndef PROTOBUF_C_Download_2eproto__INCLUDED
#define PROTOBUF_C_Download_2eproto__INCLUDED

#include <protobuf-c/protobuf-c.h>

PROTOBUF_C__BEGIN_DECLS

#if PROTOBUF_C_VERSION_NUMBER < 1000000
# error This file was generated by a newer version of protoc-c which is incompatible with your libprotobuf-c headers. Please update your headers.
#elif 1003000 < PROTOBUF_C_MIN_COMPILER_VERSION
# error This file was generated by an older version of protoc-c which is incompatible with your libprotobuf-c headers. Please regenerate this file with a newer version of protoc-c.
#endif

#include "IdentityMsg.pb-c.h"

typedef struct _Protocol__DownloadReq Protocol__DownloadReq;
typedef struct _Protocol__DownloadRsp Protocol__DownloadRsp;


/* --- enums --- */


/* --- messages --- */

struct  _Protocol__DownloadReq
{
  ProtobufCMessage base;
  Protocol__IdentityMsg *iden;
  /*
   * firmware id
   */
  char *fileid;
  /*
   * 固件开始位置
   */
  protobuf_c_boolean has_begin;
  int32_t begin;
  /*
   * 回传固件长度限制
   */
  protobuf_c_boolean has_length;
  int32_t length;
};
extern char protocol__download_req__file_id__default_value[];
#define PROTOCOL__DOWNLOAD_REQ__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&protocol__download_req__descriptor) \
    , NULL, protocol__download_req__file_id__default_value, 0, 0, 0, 1400 }


struct  _Protocol__DownloadRsp
{
  ProtobufCMessage base;
  Protocol__IdentityMsg *iden;
  /*
   * 固件总长度
   */
  protobuf_c_boolean has_total;
  int32_t total;
  /*
   * 固件开始位置
   */
  protobuf_c_boolean has_begin;
  int32_t begin;
  /*
   * 回传固件的长度
   */
  protobuf_c_boolean has_length;
  int32_t length;
  /*
   * 固件数据
   */
  protobuf_c_boolean has_data;
  ProtobufCBinaryData data;
};
extern uint8_t protocol__download_rsp__data__default_value_data[];
#define PROTOCOL__DOWNLOAD_RSP__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&protocol__download_rsp__descriptor) \
    , NULL, 0, 0, 0, 0, 0, 0, 0, { 0, protocol__download_rsp__data__default_value_data } }


/* Protocol__DownloadReq methods */
void   protocol__download_req__init
                     (Protocol__DownloadReq         *message);
size_t protocol__download_req__get_packed_size
                     (const Protocol__DownloadReq   *message);
size_t protocol__download_req__pack
                     (const Protocol__DownloadReq   *message,
                      uint8_t             *out);
size_t protocol__download_req__pack_to_buffer
                     (const Protocol__DownloadReq   *message,
                      ProtobufCBuffer     *buffer);
Protocol__DownloadReq *
       protocol__download_req__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   protocol__download_req__free_unpacked
                     (Protocol__DownloadReq *message,
                      ProtobufCAllocator *allocator);
/* Protocol__DownloadRsp methods */
void   protocol__download_rsp__init
                     (Protocol__DownloadRsp         *message);
size_t protocol__download_rsp__get_packed_size
                     (const Protocol__DownloadRsp   *message);
size_t protocol__download_rsp__pack
                     (const Protocol__DownloadRsp   *message,
                      uint8_t             *out);
size_t protocol__download_rsp__pack_to_buffer
                     (const Protocol__DownloadRsp   *message,
                      ProtobufCBuffer     *buffer);
Protocol__DownloadRsp *
       protocol__download_rsp__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   protocol__download_rsp__free_unpacked
                     (Protocol__DownloadRsp *message,
                      ProtobufCAllocator *allocator);
/* --- per-message closures --- */

typedef void (*Protocol__DownloadReq_Closure)
                 (const Protocol__DownloadReq *message,
                  void *closure_data);
typedef void (*Protocol__DownloadRsp_Closure)
                 (const Protocol__DownloadRsp *message,
                  void *closure_data);

/* --- services --- */


/* --- descriptors --- */

extern const ProtobufCMessageDescriptor protocol__download_req__descriptor;
extern const ProtobufCMessageDescriptor protocol__download_rsp__descriptor;

PROTOBUF_C__END_DECLS


#endif  /* PROTOBUF_C_Download_2eproto__INCLUDED */