/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.6-dev at Wed Oct 24 15:48:43 2018. */

#ifndef PB_IDENTITYMSG_PB_H_INCLUDED
#define PB_IDENTITYMSG_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct {
    pb_callback_t UUID;
    uint32_t Token;
/* @@protoc_insertion_point(struct:proto_router_identity_msg_t) */
} proto_router_identity_msg_t;

/* Default values for struct fields */

/* Initializer values for message structs */
#define PROTO_ROUTER_IDENTITY_MSG_INIT_DEFAULT   {{{NULL}, NULL}, 0}
#define PROTO_ROUTER_IDENTITY_MSG_INIT_ZERO      {{{NULL}, NULL}, 0}

/* Field tags (for use in manual encoding/decoding) */
#define PROTO_ROUTER_IDENTITY_MSG_UUID_TAG       1
#define PROTO_ROUTER_IDENTITY_MSG_TOKEN_TAG      2

/* Struct field encoding specification for nanopb */
extern const pb_field_t proto_router_identity_msg_fields[3];

/* Maximum encoded size of messages (where known) */
/* PROTO_ROUTER_IDENTITY_MSG_SIZE depends on runtime parameters */

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define IDENTITYMSG_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
