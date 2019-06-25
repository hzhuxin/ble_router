/*!
 *    @file  util.h
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
/**
 * @brief Calculate offset
 * @return None
 */
#define util_offset(type, member) ((uint32_t)(&((type*)0)->member))


/**
 * @brief Calculate absolut difference
 *
 * @param a
 * @param b
 *
 * @return the absolute difference
 */
#define util_abs_diff(a, b)          ((a) > (b) ? (a) - (b) : (b) - (a))

/* Global functions ----------------------------------------------------------*/
/**
 * @brief Check for timeout
 * @return true if timeout
 */
bool util_timeout(uint32_t start, uint32_t now, uint32_t timeout);

/**
 * @brief Run the checksum calculation over the buffer of bytes.
 *
 * @param data    Bytes buffer on which to run checksum.
 * @param oset    Bytes offset in the buffer.
 * @param len     Bytes count in the buffer.
 *
 * @return  checksum over the given buffer.
 */
uint8_t util_checksum(uint8_t *data, uint32_t oset, uint32_t len);

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
uint16_t util_crc16(uint16_t crc, uint8_t *msg, uint32_t oset, uint32_t len);

#endif // #ifndef __UTIL_H__

