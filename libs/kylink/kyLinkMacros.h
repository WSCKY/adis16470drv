/*
 * kyLinkMacros.h
 *
 *  Created on: Mar 13, 2019
 *      Author: kychu
 */

#ifndef MIDDLEWARE_KYLINK_KYLINKMACROS_H_
#define MIDDLEWARE_KYLINK_KYLINKMACROS_H_

#include <stdint.h>

/* __packed keyword used to decrease the data type alignment to 1-byte */
#if defined (__CC_ARM)         /* ARM Compiler */
  #define __PACK_BEGIN  __packed
  #define __PACK_END
#elif defined (__ICCARM__)     /* IAR Compiler */
  #define __PACK_BEGIN  __packed
  #define __PACK_END
#elif defined   ( __GNUC__ )   /* GNU Compiler */
  #define __PACK_BEGIN
  #define __PACK_END    __attribute__ ((__packed__))
#elif defined   (__TASKING__)  /* TASKING Compiler */
  #define __PACK_BEGIN  __unaligned
  #define __PACK_END
#endif /* __CC_ARM */

/* exact-width signed integer types */
#ifndef int8_t
typedef   signed          char int8_t;
#endif
#ifndef int16_t
typedef   signed short     int int16_t;
#endif
#ifndef _INT32_T_DECLARED
typedef   signed           int int32_t;
#define _INT32_T_DECLARED
#endif

/* exact-width unsigned integer types */
#ifndef uint8_t
typedef unsigned          char uint8_t;
#endif
#ifndef uint16_t
typedef unsigned short     int uint16_t;
#endif
#ifndef _UINT32_T_DECLARED
typedef unsigned           int uint32_t;
#define _UINT32_T_DECLARED
#endif

#define kyNULL                                   (0UL)

#endif /* MIDDLEWARE_KYLINK_KYLINKMACROS_H_ */

/******************** kyChu<kyChu@qq.com> **** END OF FILE ********************/
