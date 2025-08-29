/*
 * Project : KIAPI-Platooning
 * Author: WooChang Seo (wcseo@kiapi.or.kr) 
 * Date: 2025-08-29
 */ 


#ifndef _PLATOONING_UTILS_H_
#define _PLATOONING_UTILS_H_
  
#include <stdint.h>
#include <stddef.h>

uint32_t CLI_UTIL_GetCrc32(const uint8_t* pBuf, size_t unSize);
   
#endif