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

void osdUpdateTask() {
	if (fcStatusData.isOSDEnabled && !fcStatusData.isConfigMode && !fcStatusData.isDebugEnabled) {

		OSD_DATA_BUFFER[0] = fcStatusData.canStart | fcStatusData.canStabilize << 8 | fcStatusData.canFly << 16 | fcStatusData.hasCrashed << 24;
		OSD_DATA_BUFFER[1] = fcStatusData.isTxOn   | fcStatusData.isNavigationDataReliable << 8 | fcStatusData.isNavigationModeActive << 16 | fcStatusData.isNavigationRTHModeActive << 24;
		OSD_DATA_BUFFER[2] = fcStatusData.throttleControlPercent * 100;

		OSD_DATA_BUFFER[3] = sensorAttitudeData.pitch   * 10;
		OSD_DATA_BUFFER[4] = sensorAttitudeData.roll    * 10;
		OSD_DATA_BUFFER[5] = sensorAttitudeData.heading * 10;

		OSD_DATA_BUFFER[6] = positionCordinateData.xPosition * 100;
		OSD_DATA_BUFFER[7] = positionCordinateData.xVelocity * 100;

		OSD_DATA_BUFFER[8] = positionCordinateData.yPosition * 100;
		OSD_DATA_BUFFER[9] = positionCordinateData.yVelocity * 100;

		OSD_DATA_BUFFER[10] = positionCordinateData.zPosition;
		OSD_DATA_BUFFER[11] = positionCordinateData.zVelocity;

		OSD_DATA_BUFFER[12] = gnssData.satCount | gnssData.fixType << 8;
		OSD_DATA_BUFFER[13] = gnssData.latitude * 1000000;
		OSD_DATA_BUFFER[14] = gnssData.longitude * 1000000;

		OSD_DATA_BUFFER[15] = fcStatusData.batteryVolt + 11.1;

		sendConfigData(OSD_DATA_BUFFER, 16, CMD_FC_DATA);
	}
}

uint8_t initOSDManager() {
	delayMs(100);
#if OSD_ENABLED == 1
	schedulerAddTask(osdUpdateTask, OSD_TASK_FREQUENCY, OSD_TASK_PRIORITY);
	fcStatusData.isOSDEnabled = 1;
	logString("[OSD Manager] >> Init >> Success\n");
#else
	logString("[OSD Manager] >> Init >> Skipped\n");
#endif
	return 1;
}

