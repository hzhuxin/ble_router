/* 
// C prototype : void StrToHex(byte *pbDest, char *pszSrc, int nLen) 
// parameter(s): [OUT] pbDest - 输出缓冲区 
//  [IN] pszSrc - 字符串 
//  [IN] nLen - 16进制数的字节数(字符串的长度/2) 
// return value: 
// remarks : 将字符串转化为16进制数 
*/  
#include "HexStr.h"
#include "stdint.h"
#include <ctype.h>
//#include "debug.h"
void user_str_to_hex_symbol_end(byte *pbDest, char *pszSrc, char symbol)  
{  
    char h1, h2;  
    byte s1, s2; 
    int32_t i = 0;
    while(*pszSrc != symbol) 
    {  
        h1 = pszSrc[2 * i];  
        h2 = pszSrc[2 * i + 1];  
  
        s1 = toupper(h1) - 0x30;  
        if (s1 > 9)  
            s1 -= 7;  
  
        s2 = toupper(h2) - 0x30;  
        if (s2 > 9)  
            s2 -= 7;  
  
        pbDest[i] = s1 * 16 + s2; 
        i++;
    }  
}
void StrToHex(byte *pbDest, char *pszSrc, int nLen)  
{  
    char h1, h2;  
    byte s1, s2;  
    for (int i = 0; i < nLen; i++)  
    {  
        h1 = pszSrc[2 * i];  
        h2 = pszSrc[2 * i + 1];  
  
        s1 = toupper(h1) - 0x30;  
        if (s1 > 9)  
            s1 -= 7;  
  
        s2 = toupper(h2) - 0x30;  
        if (s2 > 9)  
            s2 -= 7;  
  
        pbDest[i] = s1 * 16 + s2;  
    }  
}  
  
/* 
// C prototype : void HexToStr(char *pszDest, byte *pbSrc, int nLen) 
// parameter(s): [OUT] pszDest - 存放目标字符串 
//  [IN] pbSrc - 输入16进制数的起始地址 
//  [IN] nLen - 16进制数的字节数 
// return value: 
// remarks : 将16进制数转化为字符串 
*/  
void HexToStr(char *pszDest, byte *pbSrc, int nLen)  
{  
    char    ddl, ddh;  
    for (int i = 0; i < nLen; i++)  
    {  
        
        ddh = 48 + pbSrc[i] / 16;  
        ddl = 48 + pbSrc[i] % 16;  
        if (ddh > 57) ddh = ddh + 7;  
        if (ddl > 57) ddl = ddl + 7;  
        pszDest[i * 2] = ddh;  
        pszDest[i * 2 + 1] = ddl;  
        //DBG_I("%c%c",ddh,ddl);
    }  
    pszDest[nLen * 2] = '\0';  
}  


