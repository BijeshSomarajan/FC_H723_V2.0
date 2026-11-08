#ifndef SRC_FC_MANAGERS_CONFIG_CONFIGHELPERCOMMON_H_
#define SRC_FC_MANAGERS_CONFIG_CONFIGHELPERCOMMON_H_

#include <sys/_stdint.h>
#include "../../calibration/Calibration.h"


#define CONFIG_SOURCE_DESKTOP_APP 0
#define CONFIG_SOURCE_MOBILE_APP 1

#define  CONFIG_DATA_MAX_LENGTH CALIB_PROP_MAX_CONFIGURABLE_LENGTH
//Available commands
#define CMD_RC_DATA 0xEC
#define CMD_ACK_RC_DATA 0xED
#define CMD_CALIBRATE_RC  0xEE
#define CMD_ACK_CALIBRATE_RC  0xEF
#define CMD_CALIBRATE_IMU_MAG  0xF1
#define CMD_ACK_CALIBRATE_IMU_MAG 0xF2
#define CMD_CALIBRATE_IMU_OFFSET  0xF5
#define CMD_ACK_CALIBRATE_IMU_OFFSET 0xF6
#define CMD_READ_CONFIG 0xF7
#define CMD_ACK_READ_CONFIG 0xF8
#define CMD_SAVE_CONFIG 0xF9
#define CMD_ACK_SAVE_CONFIG 0xFA
#define CMD_START_FC_DATA 0xFB
#define CMD_ACK_START_FC_DATA 0xFC
#define CMD_STOP_FC_DATA 0xFD
#define CMD_ACK_STOP_FC_DATA 0xFE
#define CMD_FC_DATA 0xFF
//Available sub commands
#define SUB_CMD_AG_BIAS    1
#define SUB_CMD_MAG_BIAS    2
#define SUB_CMD_RC_BIAS    3
#define SUB_CMD_ATT_MASTER_PID    4
#define SUB_CMD_ATT_RATE_PID 5
#define SUB_CMD_ALT_MASTER_PID 6
#define SUB_CMD_ALT_RATE_PID 7
#define SUB_CMD_POS_MASTER_PID 8
#define SUB_CMD_POS_RATE_PID 9
#define SUB_CMD_STICK_GAIN   10
#define SUB_CMD_DEVICE   11
#define SUB_CMD_MISC   12
#define SUB_CMD_EST_PROCESS   13
#define SUB_CMD_EST_MEASUREMENT   14

typedef struct _ConfigDataPacket ConfigDataPacket;
struct _ConfigDataPacket {
	int32_t cmd;
	int32_t subCmd;
	int32_t length;
	int32_t data[CONFIG_DATA_MAX_LENGTH];
	uint8_t source;
};

void addConfigDataPacket(ConfigDataPacket dataPacket);
uint8_t hasConfigDataPacket();
ConfigDataPacket getConfigDataPacket();

#endif /* SRC_FC_MANAGERS_CONFIG_CONFIGHELPERCOMMON_H_ */
