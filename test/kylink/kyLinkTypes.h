#ifndef __KYLINKTYPES_H
#define __KYLINKTYPES_H

#include "kyLinkMacros.h"
/* user configure file */
#include "LinkIf_conf.h"

/*
 * Default Definitions
 */
#ifndef HARD_DEV_ID
  #define HARD_DEV_ID                            (0)
#endif /* HARD_DEV_ID */

/* Frame Header */
#define kySTX1                                   (0x55)
#define kySTX2                                   (0xAA)

/* Frame Structure definitions */
__PACK_BEGIN typedef enum {
  DECODE_STATE_UNSYNCED  = 0,
  DECODE_STATE_GOT_STX1  = 1,
  DECODE_STATE_GOT_STX2  = 2,
  DECODE_STATE_GOT_DEVID = 3,
  DECODE_STATE_GOT_MSGID = 4,
  DECODE_STATE_GOT_LEN_L = 5,
  DECODE_STATE_GOT_LEN_H = 6,
  DECODE_STATE_GOT_DATA  = 7,
  DECODE_STATE_GOT_CRC_L = 8,
} __PACK_END DECODE_STATE;

__PACK_BEGIN typedef enum {
  KYLINK_MSG_HEARTBEAT = 0x01,
  KYLINK_MSG_VERN_INFO = 0x02,
  KYLINK_MSG_ID_USER   = 0x03,
} __PACK_END PACKET_TYPE;

__PACK_BEGIN typedef struct {
  uint8_t stx1;
  uint8_t stx2;
  uint8_t dev_id;
  uint8_t msg_id;
  uint16_t length;
  uint8_t *buffer;
  uint16_t crc16;
} __PACK_END kyLinkBlockDef;

typedef void (*kyLinkDecodeCallback)(kyLinkBlockDef *);
typedef status_t (*kyLinkPortTxBytesFunc)(uint8_t *, uint32_t);

typedef struct {
  /* transmit configuration */
  kyLinkPortTxBytesFunc txfunc;
  /* decoder configuration */
  uint8_t *decoder_cache;
  uint16_t cache_size;
  kyLinkDecodeCallback callback;
} kyLinkConfig_t;

__PACK_BEGIN typedef struct {
  /* decoder required. */
  uint16_t cache_size;
  uint32_t rx_counter;
  DECODE_STATE _decode_state;
  kyLinkBlockDef _rx_packet;
  kyLinkDecodeCallback decode_callback;
} __PACK_END kyLinkDecoderDef;

__PACK_BEGIN typedef struct {
  kyLinkBlockDef block;
  kyLinkDecoderDef decoder;
  kyLinkPortTxBytesFunc txfunc;
} __PACK_END KYLINK_CORE_HANDLE;

#endif /* __KYLINKTYPES_H */

/******************** kyChu<kyChu@qq.com> **** END OF FILE ********************/
