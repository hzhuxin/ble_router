/*!
 *    @file  KX022.h
 *   @brief  Accelerometer Sensor based on KX022
 *
 *  @author  Dale.J (dj), Dale.J@zoho.com
 *
 *  @internal
 *       Created:  04/ 8/2016
 *      Revision:  none
 *  Organization:  Druid Tech
 *     Copyright:  Copyright (c) 2016, Dale.J
 *
 *  This source code is released for free distribution under the terms of the
 *  GNU General Public License as published by the Free Software Foundation.
 */

#ifndef __KX022_REG_H__
#define __KX022_REG_H__

/* Accelerometer Outputs */
#define KX022_XHPL           0x00
#define KX022_XHPH           0x01
#define KX022_YHPL           0x02
#define KX022_YHPH           0x03
#define KX022_ZHPL           0x04
#define KX022_ZHPH           0x05
#define KX022_XOUTL          0x06
#define KX022_XOUTH          0x07
#define KX022_YOUTL          0x08
#define KX022_YOUTH          0x09
#define KX022_ZOUTL          0x0A
#define KX022_ZOUTH          0x0B

/* ID */
#define KX022_COTR           0x0C
#define KX022_WHO_AM_I       0x0F

/* Functions */
#define KX022_TSCP           0x10
#define KX022_TSPP           0x11
#define KX022_INS1           0x12
#define KX022_INS2           0x13
#define KX022_INS3           0x14
#define KX022_STATUS         0x15
#define KX022_INT_REL        0x17
#define KX022_CNTL1          0x18
#define KX022_CNTL2          0x19
#define KX022_CNTL3          0x1A
#define KX022_ODCNTL         0x1B
#define KX022_INC1           0x1C
#define KX022_INC2           0x1D
#define KX022_INC3           0x1E
#define KX022_INC4           0x1F
#define KX022_INC5           0x20
#define KX022_INC6           0x21
#define KX022_TILT_TIMER     0x22
#define KX022_WUFC           0x23  // Wake up counter
#define KX022_TDTRC          0x24
#define KX022_TDTC           0x25
#define KX022_TTH            0x26
#define KX022_TTL            0x27
#define KX022_FTD            0x28
#define KX022_STD            0x29
#define KX022_TLT            0x2A
#define KX022_TWS            0x2B
#define KX022_ATH            0x30  // Wake up threshold
#define KX022_TILT_ANGLE_LL  0x32
#define KX022_TILT_ANGLE_HL  0x33
#define KX022_HYST_SET       0x34
#define KX022_LP_CNTL        0x35

/* FIFO */
#define KX022_BUF_CNTL1      0x3A  // Water mark threshold
#define KX022_BUF_CNTL2      0x3B
#define KX022_BUF_STATUS1    0x3C  // Valid data bytes in the buffer
#define KX022_BUF_STATUS2    0x3D
#define KX022_BUF_CLEAR      0x3E
#define KX022_BUF_READ       0x3F
#define KX022_SELF_TEST      0x60

/* Read */
#define KX022_READ           0x80
#define KX022_WRITE          0x00

/* Bit values in CNTL1 */
#define KX022_POWER_ON      0x80
#define KX022_STANDBY       0x00
#define KX022_RES_HIHG      0x40
#define KX022_RANGE_2G      0x00
#define KX022_RANGE_4G      0x08
#define KX022_RANGE_8G      0x10
#define KX022_TAPDET_EN     0x04
#define KX022_WAKEUP_EN     0x02
#define KX022_TILTDET_EN    0x01

/* Bit values in CNTL2 */
#define KX022_SOFT_RESET    0x80
#define KX022_COMMAND_TEST  0x40

/* Bit values in CNTL3 */
#define KX022_TILT_ODR_1P5  0x00
#define KX022_TILT_ODR_6P2  0x40
#define KX022_TILT_ODR_12   0x80
#define KX022_TILT_ODR_50   0xC0

#define KX022_TAP_ODR_50    0x00
#define KX022_TAP_ODR_100   0x08
#define KX022_TAP_ODR_200   0x10
#define KX022_TAP_ODR_400   0x18
#define KX022_TAP_ODR_12    0x20
#define KX022_TAP_ODR_25    0x28
#define KX022_TAP_ODR_800   0x30
#define KX022_TAP_ODR_1600  0x38

#define KX022_WUF_ODR_0P7   0x00
#define KX022_WUF_ODR_1P5   0x01
#define KX022_WUF_ODR_3P1   0x02
#define KX022_WUF_ODR_6P2   0x03
#define KX022_WUF_ODR_12    0x04
#define KX022_WUF_ODR_25    0x05
#define KX022_WUF_ODR_50    0x06
#define KX022_WUF_ODR_100   0x07

/* Bit values in ODCNTL */
#define KX022_BYPASS_FILTER 0x80
#define KX022_LP_FILTER_M9  0x00
#define KX022_LP_FILTER_M2  0x40

#define KX022_ODR_12        0x00
#define KX022_ODR_25        0x01
#define KX022_ODR_50        0x02
#define KX022_ODR_100       0x03
#define KX022_ODR_200       0x04
#define KX022_ODR_400       0x05
#define KX022_ODR_800       0x06
#define KX022_ODR_1600      0x07
#define KX022_ODR_0P7       0x08
#define KX022_ODR_1P5       0x09
#define KX022_ODR_3P1       0x0A
#define KX022_ODR_6P2       0x0B

/* Bit values in INC1 & INT5 */
#define KX022_INT_EN        0x20
#define KX022_INT_ACTV_HIGH 0x10
#define KX022_INT_ACTV_LOW  0x00
#define KX022_INT_LATCH_OFF 0x08
#define KX022_INT_LATCH_ON  0x00

/* Bit values in INC4 & INT6 */
#define KX022_INT_OVERFLOW  0x40
#define KX022_INT_WATERMARK 0x20
#define KX022_INT_DATAREADY 0x10
#define KX022_INT_TAPDET    0x04
#define KX022_INT_WAKEUP    0x02
#define KX022_INT_TILTPOS   0x01

/* Bit values in LP_CNTL */
#define KX022_OVER_SAMP_1   0x00
#define KX022_OVER_SAMP_2   0x10
#define KX022_OVER_SAMP_4   0x20
#define KX022_OVER_SAMP_8   0x30
#define KX022_OVER_SAMP_16  0x40
#define KX022_OVER_SAMP_32  0x50
#define KX022_OVER_SAMP_64  0x60
#define KX022_OVER_SAMP_128 0x70

/* Bit values in BUF_CNTL2 */
#define KX022_BUF_EN        0x80
#define KX022_BUF_16BITS    0x40
#define KX022_BUF_FULL_IEN  0x20

#define KX022_BUF_MODE_FIFO     0x00
#define KX022_BUF_MODE_STREAM   0x01
#define KX022_BUF_MODE_TRIGGER  0x02
#define KX022_BUF_MODE_FILO     0x03

/* Bit values in SELF_TEST */
#define KX022_SELF_TEST_KEY 0xCA


/* Wartermark */
#define ACC_FIFO_WATERMARK      39

#endif // #ifndef __KX022_REG_H__

