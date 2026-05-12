#include <sys/_stdint.h>
#include "OSDManager.h"
#include "../../FCConfig.h"
#include "../../imu/IMU.h"
#include "../../logger/Logger.h"
#include "../../status/FCStatus.h"
#include "../../timers/DelayTimer.h"
#include "../../timers/Scheduler.h"
#include "../config/ConfigHelper.h"

#include "../../sensors/altitude/AltitudeSensor.h"
#include "../../sensors/attitude/AttitudeSensor.h"
#include "../../managers/position/PositionManager.h"
#include "../../sensors/position/GNSS.h"

int32_t OSD_DATA_BUFFER[16];
uint8_t osdSequencer = 0;

void osdUpdateTask() {
	if (fcStatusData.isOSDEnabled && fcStatusData.canStart) {
		if (osdSequencer == 0) {
			updateOSDFCStatus();
			osdSequencer = 1;
		} else if (osdSequencer == 1) {
			updateOSDBodyFrameData();
			osdSequencer = 2;
		} else if (osdSequencer == 2) {
			updateOSDGlobalFrameData();
			osdSequencer = 0;
		}
	}
}

uint8_t initOSDManager() {
	delayMs(100);
	schedulerAddTask(osdUpdateTask, OSD_TASK_FREQUENCY, OSD_TASK_PRIORITY);
	fcStatusData.isOSDEnabled = 1;
	logString("[OSD Manager] >> Init >> Success\n");
	return 1;
}

void sendOSDDataPacket(ConfigDataPacket dataPacket) {
	sendConfigDataPacket(dataPacket);
}

void updateOSD(int32_t *data, uint8_t length, uint8_t cmd) {
	if (length <= CONFIG_DATA_MAX_LENGTH) {
		ConfigDataPacket dataPacket;
		dataPacket.cmd = cmd;
		dataPacket.length = length;
		for (int32_t indx = 0; indx < length; indx++) {
			dataPacket.data[indx] = data[indx];
		}
		sendOSDDataPacket(dataPacket);
	}
}

void updateOSDFCStatus() {
	uint8_t status = OSD_INPUT_FC_STATUS_POWERED;
	if (fcStatusData.hasCrashed == 1) {
		status = OSD_INPUT_FC_STATUS_CRASHED;
	} else if (fcStatusData.canStart == 1) {
		if (fcStatusData.canArm == 1) {
			status = OSD_INPUT_FC_STATUS_INIT;
		} else if (fcStatusData.isRTHModeActive == 1) {
			status = OSD_INPUT_FC_STATUS_RTH;
		} else if (fcStatusData.isPositionHoldModeActive == 1) {
			status = OSD_INPUT_FC_STATUS_POS_HOLD;
		} else {
			status = OSD_INPUT_FC_STATUS_LOITER;
		}
	}
	OSD_DATA_BUFFER[OSD_INPUT_FC_STATUS_INDEX] = status;
	OSD_DATA_BUFFER[OSD_INPUT_GPS_STATUS_INDEX] = gnssData.fixStatus;
	updateOSD(OSD_DATA_BUFFER, 2, OSD_COMMAND_FC_STATUS);
}

void updateOSDBodyFrameData() {
	OSD_DATA_BUFFER[OSD_INPUT_SEALEVEL_ALT_INDEX] = sensorAltitudeData.altitudeSLFiltered;
	OSD_DATA_BUFFER[OSD_INPUT_TERRAIN_ALT_INDEX] = positionCordinateData.zPosition;

	OSD_DATA_BUFFER[OSD_INPUT_HEADING_INDEX] = sensorAttitudeData.heading * 10;
	OSD_DATA_BUFFER[OSD_INPUT_PITCH_INDEX] = sensorAttitudeData.pitch * 10;
	OSD_DATA_BUFFER[OSD_INPUT_ROLL_INDEX] = sensorAttitudeData.roll * 10;

	updateOSD(OSD_DATA_BUFFER, 5, OSD_COMMAND_BODY_FRAME_DATA);
}

void updateOSDGlobalFrameData() {
	OSD_DATA_BUFFER[OSD_INPUT_LAT_INDEX] = positionCordinateData.xPosition  * 10;
	OSD_DATA_BUFFER[OSD_INPUT_LON_INDEX] = positionCordinateData.yPosition  * 10;
	updateOSD(OSD_DATA_BUFFER, 2, OSD_COMMAND_GLOBAL_FRAME_DATA);
}

