/*!
 *    @file  util.c
 *   @brief  The generic utils
 *
 *  @author  Dale.J (dj), Dale.J@zoho.com
 *
 *  @internal
 *       Created:  01/29/2018
 *      Revision:  none
 *  Organization:  Druid Tech
 *     Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */

#ifndef __UTIL_H__
#define __UTIL_H__

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/* Typedefs ------------------------------------------------------------------*/


/* Global variables ----------------------------------------------------------*/

/* Defines -------------------------------------------------------------------*/

/* Global functions ----------------------------------------------------------*/
/**
 * @brief Check for timeout
 * @return true if timeout
 */
bool util_timeout(uint32_t start, uint32_t now, uint32_t timeout) {
  return (now >= start ? now - start > timeout : 0xffffffff - start + now > timeout);
}

/**
 * @brief Run the check sum calculation over the buffer of bytes.
 *
 * @param data    Bytes buffer on which to run the check sum.
 * @param oset    Bytes offset in the buffer.
 * @param len     Bytes count in the buffer.
 *
 * @return  check sum over the given buffer.
 */
uint8_t util_checksum(uint8_t *data, uint32_t oset, uint32_t len) {
  int sum = 0;

  for(int i = 0; i < len; i++) {
    sum += data[i + oset];
  }

  return sum & 0xff;
}

/**
 * @brief Run the crc16 calculation over the buffer of bytes.
 *
 * @param crc    The init crc value.
 * @param msg    Bytes buffer on which to run crc16.
 * @param oset   Bytes offset in the buffer.
 * @param len    Bytes count in the buffer.
 *
 * @return  Bytes count in the buffer.
 */
uint16_t util_crc16(uint16_t crc, uint8_t *msg, uint32_t oset, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        crc  = (uint8_t)(crc >> 8) | (crc << 8);
        crc ^= msg[oset + i];
        crc ^= (uint8_t)(crc & 0xFF) >> 4;
        crc ^= (crc << 8) << 4;
        crc ^= ((crc & 0xFF) << 4) << 1;
    }
    return crc;
}

#endif // #ifndef __UTIL_H__

