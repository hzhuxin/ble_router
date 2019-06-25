/*!
 *    @file  isl29035.h
 *   @brief  Ambient light sensor based on ISL290xx
 *
 *  @author  Dale.J (dj), Dale.J@zoho.com
 *
 *  @internal
 *       Created:  04/ 6/2016
 *      Revision:  none
 *  Organization:  Druid Tech
 *     Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */

#ifndef __ISL29035_H__
#define __ISL29035_H__
/* Includes ------------------------------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
// Register definition
#define ALS_REG_CMDI    0x00
#define ALS_REG_CMDII   0x01
#define ALS_REG_DATAL   0x02
#define ALS_REG_DATAH   0x03
#define ALS_REG_INTLL   0x04
#define ALS_REG_INTLH   0x05
#define ALS_REG_INTHL   0x06
#define ALS_REG_INTHH   0x07
#define ALS_REG_TEST    0x08
#define ALS_REG_ID      0x0F
// CMDI Register fields
#define CMDI_NOP        (0x00 << 5)
#define CMDI_ALS_ONCE   (0x01 << 5)
#define CMDI_IR_ONCE    (0x02 << 5)
#define CMDI_ALS_CONTI  (0x05 << 5)
#define CMDI_IR_CONTI   (0x06 << 5)
// CMDII Register fields
#define CMDII_RNG_1K    (0x00)
#define CMDII_RNG_4K    (0x01)
#define CMDII_RNG_16K   (0x02)
#define CMDII_RNG_64K   (0x03)
#define CMDII_RES_16BIT (0x00 << 2)
#define CMDII_RES_12BIT (0x01 << 2)
#define CMDII_RES_8BIT  (0x02 << 2)
#define CMDII_RES_4BIT  (0x03 << 2)
// ID Register fields
#define ALS_ID_MASK     (0xA8)
// Sample time
#define SAMPLE_TIME_4BIT        (1)
#define SAMPLE_TIME_8BIT        (2)
#define SAMPLE_TIME_12BIT       (8)
#define SAMPLE_TIME_16BIT       (120)
// Sample to lux output
#define SAMPLE_TO_LUX(range, bits, sample) (((uint32_t)(sample) * (range)) >> (bits))

/* Function prototypes -------------------------------------------------------*/

#endif // #ifndef __ALS_H__

