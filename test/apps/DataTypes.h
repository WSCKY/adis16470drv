/*
 * DataTypes.h
 *
 *  Created on: Feb 26, 2019
 *      Author: kychu
 */

#ifndef APPS_DATATYPES_H_
#define APPS_DATATYPES_H_

#include "ComTypes.h"

#define FILE_DATA_CACHE                        (80) /* 16 * n */
#define MAIN_DATA_CACHE                        (FILE_DATA_CACHE + 5)

__PACK_BEGIN typedef enum {
	FW_TYPE_NONE = 0,
	FW_TYPE_DRV_APP = 1,
	FW_TYPE_IMU_APP = 2,
	FW_TYPE_NAV_APP = 3,
	FW_TYPE_USC_APP = 4,
} __PACK_END FW_TYPE;

__PACK_BEGIN typedef enum {
	ENC_TYPE_PLAIN = 0,
	ENC_TYPE_AES_ECB = 1,
} __PACK_END FW_ENC_TYPE;

__PACK_BEGIN typedef enum {
	IsReady = 0,
	InErasing = 1,
	Upgrading = 2,
	Refused = 3,
	JumpFailed = 4,
} __PACK_END DevpgradeState;

__PACK_BEGIN typedef struct {
	FW_TYPE FW_Type;
	FW_ENC_TYPE Enc_Type;
	uint32_t PacketNum;
	uint32_t FileSize;
	uint16_t FW_Version;
	uint32_t FileCRC;
} __PACK_END FWInfoDef;

__PACK_BEGIN typedef struct {
	uint32_t PacketID;
	uint8_t PacketLen;
	uint8_t TextData[FILE_DATA_CACHE];
} __PACK_END UpgradeDataDef;

__PACK_BEGIN typedef struct {
	DevpgradeState Dev_State;
	uint8_t reserve[5];
} __PACK_END DevResponseDef;

#endif /* APPS_DATATYPES_H_ */
