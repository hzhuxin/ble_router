/*
 * @Author: jiaqi.chen 
 * @Date: 2018-04-11 17:33:33 
 * @Last Modified by: jiaqi.chen
 * @Last Modified time: 2018-04-11 17:36:06
 */
#ifndef __HEX_STR__H__
#define __HEX_STR__H__
#include "stdlib.h"
#include "stdint.h"

#define byte uint8_t

void StrToHex(byte *pbDest, char *pszSrc, int nLen);
void HexToStr(char *pszDest, byte *pbSrc, int nLen);
void user_str_to_hex_symbol_end(byte *pbDest, char *pszSrc, char symbol)  ;
#endif

