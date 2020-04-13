/*
 * kysocket.h
 *
 *  Created on: Mar 25, 2020
 *      Author: kychu
 */

#ifndef APPS_SOCKET_H_
#define APPS_SOCKET_H_

#include <stdio.h>
#include <stdint.h>

int sck_connect(const char *ip_addr);
int sck_send_package(void *msg, uint8_t msgid, uint16_t len);

#endif /* APPS_SOCKET_H_ */
