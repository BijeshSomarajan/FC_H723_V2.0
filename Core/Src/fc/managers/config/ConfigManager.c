#include "ConfigManager.h"

#include <stdio.h>
#include <sys/_stdint.h>

#include "../../calibration/Calibration.h"
#include "../../logger/Logger.h"
#include "../../sensors/attitude/AttitudeSensor.h"
#include "../../sensors/rc/RCSensor.h"
#include "../../status/FCStatus.h"
#include "../rc/RCManager.h"
#include "ConfigHelperCommon.h"
#include "ConfigHelperDesktopApp.h"
#include "ConfigHelperMobileApp.h"

uint8_t manageConfigurationSave(void);
uint8_t hasNewConfiguration(void);
void manageConfigDataPacket(void);
uint8_t configExcecPhaseLocked = 0;
uint32_t configExecCounter = 0;

uint32_t CONFIG_EXEC_COUNTER_WAIT_LOOPS = 1;
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
					return 0;
				}
			} else {
				//Load the persisted calibrations
				loadCalibration();
				logString("[Config Manager] : Calibration -> Was Initialized , Loaded\n");
				char printBuf[256];
				fcStatusData.batteryNomVolt = get1KXScaledCalibrationValue(CALIB_PROP_BAT_NOM_VOLT_ADDR);
				fcStatusData.batteryType = (uint8_t) getCalibrationValue(CALIB_PROP_BAT_TYPE_ADDR);

				fcStatusData.modelVersion = (uint8_t) getCalibrationValue(CALIB_PROP_MODEL_VERSION_ADDR);
				sprintf(printBuf, "[Config Manager] : Mdl=%d, Bat Volt=%.2f, Bat Type=%d\n", fcStatusData.modelVersion, fcStatusData.batteryNomVolt, fcStatusData.batteryType);
				logString(printBuf);
			}
			status = initConfigHelper();
			if (status) {
				logString("[Config Manager] : Helper -> Init , Success\n");
			} else {
				logString("[Config Manager] : Helper -> Init , Failed\n");
				return 0;
			}
			status = initConfigHelperMobileApp();
			if (status) {
				logString("[Config Manager] : Mobile Helper -> Init , Success\n");
			} else {
				logString("[Config Manager] : Mobile Helper -> Init , Failed\n");
				return 0;
			}
		} else {
			logString("[Config Manager] : Calibration -> Initialization Failed\n");
			return 0;
		}
	} else {
		logString("[Config Manager] : FC Protocol -> Initialization Failed\n");
		return 0;
	}
	return status;
}

void doConfigManagement() {
	if (configExcecPhaseLocked) {
		configExecCounter++;
		if (configExecCounter >= CONFIG_EXEC_COUNTER_WAIT_LOOPS) {
			manageConfigDataPacket();
			configExcecPhaseLocked = 0;
			configExecCounter = 0;
		}
	} else if (hasNewConfiguration()) {
		configExcecPhaseLocked = 1;
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
	uint8_t wasDebugEnabled = fcStatusData.isDebugEnabled;
	fcStatusData.isDebugEnabled = 0;
	if (dataPacket.cmd == CMD_START_FC_DATA) {
		sendConfigDataToDektopApp(getCalibrationData(), 0, CMD_ACK_START_FC_DATA, dataPacket.subCmd);
		fcStatusData.isDebugEnabled = 1;
	} else if (dataPacket.cmd == CMD_STOP_FC_DATA) {
		fcStatusData.isDebugEnabled = 0;
		sendConfigDataToDektopApp(getCalibrationData(), 0, CMD_ACK_STOP_FC_DATA, dataPacket.subCmd);
	} else if (dataPacket.cmd == CMD_RC_DATA) {
		setRCData(dataPacket.data, dataPacket.length);
		if (wasDebugEnabled) {
			fcStatusData.isDebugEnabled = 1;
		}
		fcStatusData.isConfigMode = 1;
	} else {
		fcStatusData.isConfigMode = 1;
		fcStatusData.canArm = 0;
		fcStatusData.canStart = 0;
		fcStatusData.canFly = 0;
		fcStatusData.canStabilize = 0;
		if (dataPacket.cmd == CMD_READ_CONFIG) {
			if (dataPacket.source == CONFIG_SOURCE_MOBILE_APP) {
				sendConfigDataToMobileApp(getCalibrationData(), CALIB_PROP_MAX_CONFIGURABLE_LENGTH, CMD_ACK_READ_CONFIG, dataPacket.subCmd);
			} else {
				sendConfigDataToDektopApp(getCalibrationData(), CALIB_PROP_MAX_CONFIGURABLE_LENGTH, CMD_ACK_READ_CONFIG, dataPacket.subCmd);
			}
		} else if (dataPacket.cmd == CMD_SAVE_CONFIG) {
			if (dataPacket.source == CONFIG_SOURCE_MOBILE_APP) {
				mergeConfigDataFromMobileApp(getCalibrationData(), &dataPacket);
			} else {
				mergeConfigDataFromDesktopApp(getCalibrationData(), &dataPacket);
			}
			if (saveCalibration()) {
				if (dataPacket.source == CONFIG_SOURCE_MOBILE_APP) {
					sendConfigDataToMobileApp(getCalibrationData(), CALIB_PROP_MAX_CONFIGURABLE_LENGTH, CMD_ACK_SAVE_CONFIG, dataPacket.subCmd);
				} else {
					sendConfigDataToDektopApp(getCalibrationData(), CALIB_PROP_MAX_CONFIGURABLE_LENGTH, CMD_ACK_SAVE_CONFIG, dataPacket.subCmd);
				}
			}
		} else if (dataPacket.cmd == CMD_CALIBRATE_IMU_OFFSET) {
			calculateAccAndGyroBias();
			if (dataPacket.source == CONFIG_SOURCE_MOBILE_APP) {
				sendConfigDataToMobileApp(getCalibrationData(), 0, CMD_ACK_CALIBRATE_IMU_OFFSET, dataPacket.subCmd);
			} else {
				sendConfigDataToDektopApp(getCalibrationData(), 0, CMD_ACK_CALIBRATE_IMU_OFFSET, dataPacket.subCmd);
			}
		} else if (dataPacket.cmd == CMD_CALIBRATE_IMU_MAG) {
			calculateMagBias();
			if (dataPacket.source == CONFIG_SOURCE_MOBILE_APP) {
				sendConfigDataToMobileApp(getCalibrationData(), 0, CMD_ACK_CALIBRATE_IMU_MAG, dataPacket.subCmd);
			} else {
				sendConfigDataToDektopApp(getCalibrationData(), 0, CMD_ACK_CALIBRATE_IMU_MAG, dataPacket.subCmd);
			}
		} else if (dataPacket.cmd == CMD_CALIBRATE_RC) {
			calibrateRCSensor();
			if (dataPacket.source == CONFIG_SOURCE_MOBILE_APP) {
				sendConfigDataToMobileApp(getCalibrationData(), 0, CMD_ACK_CALIBRATE_RC, dataPacket.subCmd);
			} else {
				sendConfigDataToDektopApp(getCalibrationData(), 0, CMD_ACK_CALIBRATE_RC, dataPacket.subCmd);
			}
		}
		//fcStatusData.isConfigMode = 0; //Need a power cycle for safety reasons.
	}

}

