/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.6-dev at Wed Oct 24 15:48:43 2018. */

#include "router_execsta.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t proto_router_exec_sta_rsp_fields[3] = {
    PB_FIELD(  1, MESSAGE , REQUIRED, STATIC  , FIRST, proto_router_exec_sta_rsp_t, Iden, Iden, &proto_router_identity_msg_fields),
    PB_FIELD(  2, UENUM   , REQUIRED, STATIC  , OTHER, proto_router_exec_sta_rsp_t, State, Iden, 0),
    PB_LAST_FIELD
};


/* Check that field information fits in pb_field_t */
#if !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_32BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in 8 or 16 bit
 * field descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(proto_router_exec_sta_rsp_t, Iden) < 65536), YOU_MUST_DEFINE_PB_FIELD_32BIT_FOR_MESSAGES_proto_router_exec_sta_rsp)
#endif

#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_16BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in the default
 * 8 bit descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(proto_router_exec_sta_rsp_t, Iden) < 256), YOU_MUST_DEFINE_PB_FIELD_16BIT_FOR_MESSAGES_proto_router_exec_sta_rsp)
#endif


/* @@protoc_insertion_point(eof) */
