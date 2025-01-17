/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: Behavior2.proto */

/* Do not generate deprecated warnings for self */
#ifndef PROTOBUF_C__NO_DEPRECATED
#define PROTOBUF_C__NO_DEPRECATED
#endif

#include "Behavior2.pb-c.h"
void   protocol__behavior2_req__init
                     (Protocol__Behavior2Req         *message)
{
  static const Protocol__Behavior2Req init_value = PROTOCOL__BEHAVIOR2_REQ__INIT;
  *message = init_value;
}
size_t protocol__behavior2_req__get_packed_size
                     (const Protocol__Behavior2Req *message)
{
  assert(message->base.descriptor == &protocol__behavior2_req__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t protocol__behavior2_req__pack
                     (const Protocol__Behavior2Req *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &protocol__behavior2_req__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t protocol__behavior2_req__pack_to_buffer
                     (const Protocol__Behavior2Req *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &protocol__behavior2_req__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
Protocol__Behavior2Req *
       protocol__behavior2_req__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (Protocol__Behavior2Req *)
     protobuf_c_message_unpack (&protocol__behavior2_req__descriptor,
                                allocator, len, data);
}
void   protocol__behavior2_req__free_unpacked
                     (Protocol__Behavior2Req *message,
                      ProtobufCAllocator *allocator)
{
  if(!message)
    return;
  assert(message->base.descriptor == &protocol__behavior2_req__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
void   protocol__behavior2__init
                     (Protocol__Behavior2         *message)
{
  static const Protocol__Behavior2 init_value = PROTOCOL__BEHAVIOR2__INIT;
  *message = init_value;
}
size_t protocol__behavior2__get_packed_size
                     (const Protocol__Behavior2 *message)
{
  assert(message->base.descriptor == &protocol__behavior2__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t protocol__behavior2__pack
                     (const Protocol__Behavior2 *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &protocol__behavior2__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t protocol__behavior2__pack_to_buffer
                     (const Protocol__Behavior2 *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &protocol__behavior2__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
Protocol__Behavior2 *
       protocol__behavior2__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (Protocol__Behavior2 *)
     protobuf_c_message_unpack (&protocol__behavior2__descriptor,
                                allocator, len, data);
}
void   protocol__behavior2__free_unpacked
                     (Protocol__Behavior2 *message,
                      ProtobufCAllocator *allocator)
{
  if(!message)
    return;
  assert(message->base.descriptor == &protocol__behavior2__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
static const ProtobufCFieldDescriptor protocol__behavior2_req__field_descriptors[2] =
{
  {
    "Iden",
    1,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Protocol__Behavior2Req, iden),
    &protocol__identity_msg__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "BehaviorInfo",
    2,
    PROTOBUF_C_LABEL_REPEATED,
    PROTOBUF_C_TYPE_MESSAGE,
    offsetof(Protocol__Behavior2Req, n_behaviorinfo),
    offsetof(Protocol__Behavior2Req, behaviorinfo),
    &protocol__behavior2__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
static const unsigned protocol__behavior2_req__field_indices_by_name[] = {
  1,   /* field[1] = BehaviorInfo */
  0,   /* field[0] = Iden */
};
static const ProtobufCIntRange protocol__behavior2_req__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 2 }
};
const ProtobufCMessageDescriptor protocol__behavior2_req__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "protocol.Behavior2Req",
  "Behavior2Req",
  "Protocol__Behavior2Req",
  "protocol",
  sizeof(Protocol__Behavior2Req),
  2,
  protocol__behavior2_req__field_descriptors,
  protocol__behavior2_req__field_indices_by_name,
  1,  protocol__behavior2_req__number_ranges,
  (ProtobufCMessageInit) protocol__behavior2_req__init,
  NULL,NULL,NULL    /* reserved[123] */
};
static const uint32_t protocol__behavior2__timestamp__default_value = 0u;
static const int32_t protocol__behavior2__odbax__default_value = 0;
static const int32_t protocol__behavior2__odbay__default_value = 0;
static const int32_t protocol__behavior2__odbaz__default_value = 0;
static const int32_t protocol__behavior2__meandl_x__default_value = 0;
static const int32_t protocol__behavior2__meandl_y__default_value = 0;
static const int32_t protocol__behavior2__meandl_z__default_value = 0;
static const int32_t protocol__behavior2__odba__default_value = 0;
static const ProtobufCFieldDescriptor protocol__behavior2__field_descriptors[8] =
{
  {
    "Timestamp",
    1,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_UINT32,
    offsetof(Protocol__Behavior2, has_timestamp),
    offsetof(Protocol__Behavior2, timestamp),
    NULL,
    &protocol__behavior2__timestamp__default_value,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "ODBAX",
    2,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_INT32,
    offsetof(Protocol__Behavior2, has_odbax),
    offsetof(Protocol__Behavior2, odbax),
    NULL,
    &protocol__behavior2__odbax__default_value,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "ODBAY",
    3,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_INT32,
    offsetof(Protocol__Behavior2, has_odbay),
    offsetof(Protocol__Behavior2, odbay),
    NULL,
    &protocol__behavior2__odbay__default_value,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "ODBAZ",
    4,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_INT32,
    offsetof(Protocol__Behavior2, has_odbaz),
    offsetof(Protocol__Behavior2, odbaz),
    NULL,
    &protocol__behavior2__odbaz__default_value,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "MeandlX",
    5,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_INT32,
    offsetof(Protocol__Behavior2, has_meandlx),
    offsetof(Protocol__Behavior2, meandlx),
    NULL,
    &protocol__behavior2__meandl_x__default_value,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "MeandlY",
    6,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_INT32,
    offsetof(Protocol__Behavior2, has_meandly),
    offsetof(Protocol__Behavior2, meandly),
    NULL,
    &protocol__behavior2__meandl_y__default_value,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "MeandlZ",
    7,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_INT32,
    offsetof(Protocol__Behavior2, has_meandlz),
    offsetof(Protocol__Behavior2, meandlz),
    NULL,
    &protocol__behavior2__meandl_z__default_value,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "ODBA",
    8,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_INT32,
    offsetof(Protocol__Behavior2, has_odba),
    offsetof(Protocol__Behavior2, odba),
    NULL,
    &protocol__behavior2__odba__default_value,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
static const unsigned protocol__behavior2__field_indices_by_name[] = {
  4,   /* field[4] = MeandlX */
  5,   /* field[5] = MeandlY */
  6,   /* field[6] = MeandlZ */
  7,   /* field[7] = ODBA */
  1,   /* field[1] = ODBAX */
  2,   /* field[2] = ODBAY */
  3,   /* field[3] = ODBAZ */
  0,   /* field[0] = Timestamp */
};
static const ProtobufCIntRange protocol__behavior2__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 8 }
};
const ProtobufCMessageDescriptor protocol__behavior2__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "protocol.Behavior2",
  "Behavior2",
  "Protocol__Behavior2",
  "protocol",
  sizeof(Protocol__Behavior2),
  8,
  protocol__behavior2__field_descriptors,
  protocol__behavior2__field_indices_by_name,
  1,  protocol__behavior2__number_ranges,
  (ProtobufCMessageInit) protocol__behavior2__init,
  NULL,NULL,NULL    /* reserved[123] */
};
