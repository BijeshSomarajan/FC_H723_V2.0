
#ifndef SRC_FC_MANAGERS_CONFIG_CONFIGHELPER_H_
#define SRC_FC_MANAGERS_CONFIG_CONFIGHELPER_H_

#include <stdint.h>
#include "../../calibration/Calibration.h"

#define  CONFIG_DATA_MAX_LENGTH CALIB_PROP_MAX_CONFIGURABLE_LENGTH

//Available commands
#define CMD_CALIBRATE_RC  0xEE
#define CMD_ACK_CALIBRATE_RC  0xEF
#define CMD_CALIBRATE_IMU_TEMP_DATA  0xF0
#define CMD_CALIBRATE_IMU_MAG  0xF1
#define CMD_ACK_CALIBRATE_IMU_MAG 0xF2
#define CMD_CALIBRATE_IMU_TEMP  0xF3
#define CMD_ACK_CALIBRATE_IMU_TEMP 0xF4
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

typedef struct _ConfigDataPacket ConfigDataPacket;
struct _ConfigDataPacket {
	int32_t cmd;
	int32_t length;
	int32_t data[CONFIG_DATA_MAX_LENGTH];
};

uint8_t initConfigHelper(void);
void readConfigSensors(void);
uint8_t hasConfigDataPacket(void);
ConfigDataPacket getConfigDataPacket();
void sendConfigDataPacket(ConfigDataPacket dataPacket);
void sendConfigData(int32_t *data, int32_t length, int32_t cmd);

#endif
