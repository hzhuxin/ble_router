# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: Debug.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import IdentityMsg_pb2 as IdentityMsg__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='Debug.proto',
  package='protocol',
  syntax='proto2',
  serialized_pb=_b('\n\x0b\x44\x65\x62ug.proto\x12\x08protocol\x1a\x11IdentityMsg.proto\"@\n\x08\x44\x65\x62ugReq\x12#\n\x04Iden\x18\x01 \x02(\x0b\x32\x15.protocol.IdentityMsg\x12\x0f\n\x07message\x18\x02 \x01(\t')
  ,
  dependencies=[IdentityMsg__pb2.DESCRIPTOR,])




_DEBUGREQ = _descriptor.Descriptor(
  name='DebugReq',
  full_name='protocol.DebugReq',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='Iden', full_name='protocol.DebugReq.Iden', index=0,
      number=1, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='message', full_name='protocol.DebugReq.message', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=44,
  serialized_end=108,
)

_DEBUGREQ.fields_by_name['Iden'].message_type = IdentityMsg__pb2._IDENTITYMSG
DESCRIPTOR.message_types_by_name['DebugReq'] = _DEBUGREQ
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

DebugReq = _reflection.GeneratedProtocolMessageType('DebugReq', (_message.Message,), dict(
  DESCRIPTOR = _DEBUGREQ,
  __module__ = 'Debug_pb2'
  # @@protoc_insertion_point(class_scope:protocol.DebugReq)
  ))
_sym_db.RegisterMessage(DebugReq)


# @@protoc_insertion_point(module_scope)