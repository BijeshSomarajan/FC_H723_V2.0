#include "ConfigManager.h"
#include "ConfigHelper.h"
#include "../../sensors/attitude/AttitudeSensor.h"
#include "../../sensors/rc/RCSensor.h"

#include "../../calibration/Calibration.h"
#include "../../logger/Logger.h"
#include "../../status/FCStatus.h"

uint8_t manageConfigurationSave(void);
uint8_t hasNewConfiguration(void);
void manageConfigDataPacket(void);

int32_t FC_CONFIG_DATA_BUFFER[CONFIG_DATA_MAX_LENGTH];

uint8_t initConfigManager() {
	uint8_t status = 1;
	if (status) {
		logString("[Config Manager] : FC Protocol -> Initialized\n");
		status = initCalibration();
		if (status) {
			logString("[Config Manager] : Calibration -> Initialized\n");
			//Initialize with default calibration
			setDefaultCalibration();
			//Checking calibration
			if (!isCalibrated()) {
				logString("[Config Manager] : Calibration -> Was not Initialized\n");
				//Save default calibration
				status = saveCalibration();
				if (status) {
					logString("[Config Manager] : Calibration -> Default , Success\n");
				} else {
					logString("[Config Manager] : Calibration -> Default , Failed!\n");
				}

			} else {
				//Load the persisted calibrations
				loadCalibration();
				logString("[Config Manager] : Calibration -> Was Initialized , Loaded\n");
			}
			status = initConfigHelper();
			if (status) {
				logString("[Config Manager] : Helper -> Init , Success\n");
			} else {
				logString("[Config Manager] : Helper -> Init , Failed\n");
			}

		} else {
			logString("[Config Manager] : Calibration -> Initialization Failed\n");
		}
	} else {
		logString("[Config Manager] : FC Protocol -> Initialization Failed\n");
	}
	return status;
}

void doConfigManagement() {
	if (hasNewConfiguration()) {
		manageConfigDataPacket();
	}
}

/**
 * Manages configuration save
 */
uint8_t manageConfigurationSave() {
	return saveCalibration();
}

/**
 * Checks if there is a new configuration
 */
uint8_t hasNewConfiguration() {
	return hasConfigDataPacket();
}

/**
 *Digests the available data packets
 */
void manageConfigDataPacket() {
	ConfigDataPacket dataPacket = getConfigDataPacket();
	fcStatusData.isDebugEnabled = 0;
	fcStatusData.isConfigMode = 1;
	if (dataPacket.cmd == CMD_START_FC_DATA) {
		sendConfigData(FC_CONFIG_DATA_BUFFER, 0, CMD_ACK_START_FC_DATA);
		fcStatusData.isDebugEnabled = 1;
	} else if (dataPacket.cmd == CMD_STOP_FC_DATA) {
		sendConfigData(FC_CONFIG_DATA_BUFFER, 0, CMD_ACK_STOP_FC_DATA);
	} else if (dataPacket.cmd == CMD_READ_CONFIG) {
		sendConfigData(getCalibrationData(), CALIB_PROP_MAX_CONFIGURABLE_LENGTH, CMD_ACK_READ_CONFIG);
	} else if (dataPacket.cmd == CMD_SAVE_CONFIG) {
		for (uint32_t indx1 = 0; indx1 < CALIB_PROP_MAX_CONFIGURABLE_LENGTH; indx1++) {
			setCalibrationValue(indx1, dataPacket.data[indx1]);
		}
		if (saveCalibration()) {
			sendConfigData(getCalibrationData(), CALIB_PROP_MAX_CONFIGURABLE_LENGTH, CMD_ACK_SAVE_CONFIG);
		}
	} else if (dataPacket.cmd == CMD_CALIBRATE_IMU_OFFSET) {
		calculateAccAndGyroBias();
		sendConfigData(FC_CONFIG_DATA_BUFFER, 0, CMD_ACK_CALIBRATE_IMU_OFFSET);
	} else if (dataPacket.cmd == CMD_CALIBRATE_IMU_TEMP) {
		calculateAccAndGyroTempCoeff();
		sendConfigData(FC_CONFIG_DATA_BUFFER, 0, CMD_ACK_CALIBRATE_IMU_TEMP);
	} else if (dataPacket.cmd == CMD_CALIBRATE_IMU_MAG) {
		calculateMagBias();
		sendConfigData(FC_CONFIG_DATA_BUFFER, 0, CMD_ACK_CALIBRATE_IMU_MAG);
	} else if (dataPacket.cmd == CMD_CALIBRATE_RC) {
		calibrateRCSensor();
		sendConfigData(FC_CONFIG_DATA_BUFFER, 0, CMD_ACK_CALIBRATE_RC);

	}
	fcStatusData.isConfigMode = 0;
}

