
#ifndef _PB_CALLBACK_H
#define _PB_CALLBACK_H

#include "pb_common.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "pb.h"

bool encode_repeated_var_string(pb_ostream_t *stream, const pb_field_t *fields,  void * const *str);
bool encode_repeated_var_int(pb_ostream_t *stream, const pb_field_t *fields,  void * const *str);
bool decode_repeated_var_string(pb_istream_t *stream, const pb_field_t *field, void **str);

#endif //_PB_CALLBACK_H
