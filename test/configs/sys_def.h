#ifndef __SYS_DEF_H
#define __SYS_DEF_H

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

typedef enum 
{
  status_ok      = 0x00U,
  status_error   = 0x01U,
  status_busy    = 0x02U,
  status_timeout = 0x03U,
  status_nomem   = 0x04U
} status_t;

typedef enum
{
  false = 0U,
  true = !false
} bool_t;

#endif /* __SYS_DEF_H */
