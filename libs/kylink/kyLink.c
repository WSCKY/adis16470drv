/**
 * @file    kyLink.c
 * @author  kyChu
 * @date    2017/08/20
 * @update  2020/01/03
 * @version V1.8.0
 * @brief   kylink protocol in ANSI C
 */
#include "kyLink.h"

/* private variables */
/* crc16 table. */
static const uint32_t crcTab16[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

/* function prototypes */
static void kylink_block_init(kyLinkBlockDef *pBlock);
static uint16_t kylink_checksum(kyLinkBlockDef *pBlock);
static uint16_t compute_crc16(uint8_t *ptr, uint32_t len, uint16_t crc);

status_t kylink_init(KYLINK_CORE_HANDLE *pHandle, kyLinkConfig_t *pConfig)
{
  kylink_block_init(&(pHandle->block));
  pHandle->txfunc = kyNULL;
  pHandle->decoder.rx_counter = 0;
  pHandle->decoder.decode_callback = kyNULL;
  pHandle->decoder._decode_state = DECODE_STATE_UNSYNCED;

  if(pConfig->txfunc != kyNULL) {
    pHandle->txfunc = pConfig->txfunc;
  }

  if(pConfig->callback != kyNULL) {
    pHandle->decoder.decode_callback = pConfig->callback;
  }

  if(pConfig->cache_size != 0 && pConfig->decoder_cache != kyNULL) {
    pHandle->decoder._rx_packet.buffer = pConfig->decoder_cache;
    pHandle->decoder.cache_size = pConfig->cache_size;
  }
  return status_ok;
}

#pragma GCC push_options
#pragma GCC optimize ("O0")
status_t kylink_send(KYLINK_CORE_HANDLE *pHandle, void *msg, uint8_t msgid, uint16_t len)
{
  status_t ret = status_error;
  if(msg == kyNULL || len == 0 || pHandle->txfunc == kyNULL) return ret;

  pHandle->block.msg_id = msgid;
  pHandle->block.buffer = (uint8_t *)msg;
  pHandle->block.length = len;
  /* compute crc16 */
  pHandle->block.crc16 = 0;
  pHandle->block.crc16 = compute_crc16(&(pHandle->block.dev_id), 4, pHandle->block.crc16);
  pHandle->block.crc16 = compute_crc16(pHandle->block.buffer, pHandle->block.length, pHandle->block.crc16);
  ret = pHandle->txfunc((uint8_t *)&(pHandle->block.stx1), 6);
  if(ret == status_ok)
    ret = pHandle->txfunc(pHandle->block.buffer, pHandle->block.length);
  if(ret == status_ok)
    ret = pHandle->txfunc((uint8_t *)&(pHandle->block.crc16), 2);

  return ret;
}
#pragma GCC pop_options

/*
  decode process.
*/
void kylink_decode(KYLINK_CORE_HANDLE *pHandle, uint8_t data)
{
  switch(pHandle->decoder._decode_state) {
	case DECODE_STATE_UNSYNCED:
	  if(data == kySTX1) {
        pHandle->decoder._rx_packet.stx1 = data;
        pHandle->decoder._decode_state = DECODE_STATE_GOT_STX1;
	  }
	break;
	case DECODE_STATE_GOT_STX1:
	  if(data == kySTX2) {
        pHandle->decoder._rx_packet.stx2 = data;
        pHandle->decoder._decode_state = DECODE_STATE_GOT_STX2;
	  } else {
        pHandle->decoder._decode_state = DECODE_STATE_UNSYNCED;
	  }
	break;
	case DECODE_STATE_GOT_STX2:
      pHandle->decoder._rx_packet.dev_id = data;
      pHandle->decoder._decode_state = DECODE_STATE_GOT_DEVID;
	break;
	case DECODE_STATE_GOT_DEVID:
      pHandle->decoder._rx_packet.msg_id = data;
      pHandle->decoder._decode_state = DECODE_STATE_GOT_MSGID;
	break;
	case DECODE_STATE_GOT_MSGID:
      pHandle->decoder._rx_packet.length = data;
      pHandle->decoder._decode_state = DECODE_STATE_GOT_LEN_L;
	break;
	case DECODE_STATE_GOT_LEN_L:
      pHandle->decoder._rx_packet.length = ((uint16_t)data << 8) | pHandle->decoder._rx_packet.length;
	  if(pHandle->decoder._rx_packet.length <= pHandle->decoder.cache_size) {
        pHandle->decoder.rx_counter = 0;
		pHandle->decoder._decode_state = DECODE_STATE_GOT_LEN_H;
	  } else {
        pHandle->decoder._decode_state = DECODE_STATE_UNSYNCED;
	  }
	break;
	case DECODE_STATE_GOT_LEN_H:
      pHandle->decoder._rx_packet.buffer[pHandle->decoder.rx_counter ++] = data;
	  if(pHandle->decoder.rx_counter == pHandle->decoder._rx_packet.length)
        pHandle->decoder._decode_state = DECODE_STATE_GOT_DATA;
	break;
	case DECODE_STATE_GOT_DATA:
      pHandle->decoder._rx_packet.crc16 = data;
      pHandle->decoder._decode_state = DECODE_STATE_GOT_CRC_L;
	break;
	case DECODE_STATE_GOT_CRC_L:
      pHandle->decoder._rx_packet.crc16 = (data << 8) | pHandle->decoder._rx_packet.crc16; /* got the crc16. */
      if(kylink_checksum(&(pHandle->decoder._rx_packet)) == pHandle->decoder._rx_packet.crc16) {
        if(pHandle->decoder.decode_callback != kyNULL) {
          pHandle->decoder.decode_callback(&pHandle->decoder._rx_packet);
        } else {}
	  } else {}
      pHandle->decoder._decode_state = DECODE_STATE_UNSYNCED;
	break;
	default:
      pHandle->decoder._decode_state = DECODE_STATE_UNSYNCED;
	break;
  }
}

/*
 * Initialize kyLink data structure.
 */
static void kylink_block_init(kyLinkBlockDef *pBlock)
{
  pBlock->stx1 = kySTX1;
  pBlock->stx2 = kySTX2;
  pBlock->dev_id = HARD_DEV_ID;
  pBlock->length = 0;
  pBlock->crc16 = 0;
}

/*
 * checksum for kyLink data structure.
 */
static uint16_t kylink_checksum(kyLinkBlockDef *pBlock)
{
  uint16_t crc = 0;
  crc = compute_crc16(&(pBlock->dev_id), 4, crc);
  crc = compute_crc16(pBlock->buffer, pBlock->length, crc);
  return crc;
}

/*
 * compute crc16 form a buffer.
 */
static uint16_t compute_crc16(uint8_t *ptr, uint32_t len, uint16_t crc)
{
  uint8_t H8;
  while(len -- != 0) {
    H8 = (unsigned short)crc >> 8;
    crc <<= 8;
    crc ^= crcTab16[H8 ^ *ptr];
    ptr ++;
  }
  return(crc);
}

/******************** kyChu<kyChu@qq.com> **** END OF FILE ********************/
