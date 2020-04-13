/**
 * @file    kyLink.h
 * @author  kyChu
 * @date    2017/08/20
 * @update  2020/01/03
 * @version V1.8.0
 * @brief   header file for kyLink.c
 */

/* Define to prevent recursive inclusion */
#ifndef __KYLINK_H
#define __KYLINK_H

/*
 * Includes
 */
#include "kyLinkTypes.h"
#include "LinkVersion.h"

/*
 * Export function prototypes
 */
status_t kylink_init(KYLINK_CORE_HANDLE *pHandle, kyLinkConfig_t *pConfig);
status_t kylink_send(KYLINK_CORE_HANDLE *pHandle, void *msg, uint8_t msgid, uint16_t len);
void kylink_decode(KYLINK_CORE_HANDLE *pHandle, uint8_t data);

#endif /* __KYLINK_H */

/******************** kyChu<kyChu@qq.com> **** END OF FILE ********************/
