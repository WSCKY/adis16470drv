/*
 * kysocket.c
 *
 *  Created on: Mar 25, 2020
 *      Author: kychu
 */

#include "kysocket.h"
#include "kyLink.h"

#include <sys/types.h>
#include <arpa/inet.h>
#include <sys/socket.h>

static int sockfd;
static struct sockaddr_in addr = {};

static KYLINK_CORE_HANDLE kylink_sck;

static status_t sck_send(uint8_t *buff, uint32_t len);

int sck_connect(const char *ip_addr)
{
  kyLinkConfig_t kylink_cfg;
  sockfd = socket(AF_INET,SOCK_DGRAM,0);
  if (0 > sockfd) {
    perror("socket");
    return -1;
  }

  addr.sin_family = AF_INET;
  addr.sin_port = htons(6000);
  addr.sin_addr.s_addr = inet_addr(ip_addr);
//  socklen_t addr_len = sizeof(struct sockaddr_in);

  kylink_cfg.cache_size = 0;
  kylink_cfg.callback = 0;
  kylink_cfg.decoder_cache = 0;
  kylink_cfg.txfunc = sck_send;
  kylink_init(&kylink_sck, &kylink_cfg);
  return 0;
}

int sck_send_package(void *msg, uint8_t msgid, uint16_t len)
{
  if(kylink_send(&kylink_sck, msg, msgid, len) == status_ok) return 0;
  return -1;
}

static status_t sck_send(uint8_t *buff, uint32_t len)
{
  if(sendto(sockfd, buff, len,0,(struct sockaddr *)&addr, sizeof(addr)) < 0) return status_error;
  return status_ok;
}
