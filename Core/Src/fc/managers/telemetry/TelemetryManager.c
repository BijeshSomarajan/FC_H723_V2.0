#include "TelemetryManager.h"

#include <sys/_stdint.h>
#include "../../FCConfig.h"
#include "../../imu/IMU.h"
#include "../../logger/Logger.h"
#include "../../status/FCStatus.h"
#include "../../timers/DelayTimer.h"
#include "../../timers/Scheduler.h"
#include "../config/ConfigHelper.h"
#include "../../sensors/rc/RCTelemetry.h"
#include "../../sensors/altitude/AltitudeSensor.h"
#include "../../sensors/attitude/AttitudeSensor.h"
#include "../../managers/position/PositionManager.h" // Cleaned up duplicate include
#include "../../sensors/position/GNSS.h"
#include "../../sensors/battery/BatterySensor.h"

char FC_STATUS_BUF[8];
TelemetryStep currentTelemetryStep = TELEMETRY_STEP_ALTITUDE;

void sendFCStatus() {
	// Off, Start, Stab, Fly
	if (fcStatusData.hasCrashed) {
		FC_STATUS_BUF[0] = 'C';
	} else if (fcStatusData.canFly) {
		FC_STATUS_BUF[0] = 'F';
	} else if (fcStatusData.canStabilize) {
		FC_STATUS_BUF[0] = 'S';
	} else if (!fcStatusData.canStart) {
		FC_STATUS_BUF[0] = 'O';
	} else if (fcStatusData.canStart) {
		FC_STATUS_BUF[0] = 'I';
	}
	// Loiter, Pos Hold, RTH
	FC_STATUS_BUF[1] = '-';
	if (fcStatusData.isNavModeActive) {
		FC_STATUS_BUF[2] = 'N'; //Nav Mode
	} else if (fcStatusData.isNavRTHModeActive) {
		FC_STATUS_BUF[2] = 'R';
	} else {
		FC_STATUS_BUF[2] = 'S'; // Stab Mode
	}
	// Terrain/Baro
	FC_STATUS_BUF[3] = '-';
	if (fcStatusData.isTerrainAltModeActive) {
		FC_STATUS_BUF[4] = 'T';
	} else {
		FC_STATUS_BUF[4] = 'B';
	}
	// Landing/Flying
	FC_STATUS_BUF[5] = '-';
	if (fcStatusData.isLandingModeActive) {
		FC_STATUS_BUF[6] = 'L';
	} else {
		FC_STATUS_BUF[6] = 'F';
	}
	FC_STATUS_BUF[7] = '\0';
	sendFlightModeTelemetry(FC_STATUS_BUF, 8);
}

/**
 * @brief Main task executed by scheduler.
 * Sends exactly ONE type of telemetry per execution tick.
 */
void telemetryUpdateTask() {

	switch (currentTelemetryStep) {
	case TELEMETRY_STEP_ALTITUDE:
		sendAltitudeTelemetry(positionCordinateData.zPosition, positionCordinateData.zVelocity);
		break;

	case TELEMETRY_STEP_ATTITUDE:
		sendAttitudeTelemetry(sensorAttitudeData.pitch, sensorAttitudeData.roll, sensorAttitudeData.heading);
		break;

	case TELEMETRY_STEP_BATTERY:
		sendBatteryTelemetry(batteryData.voltage, batteryData.current, 0, 0);
		break;

	case TELEMETRY_STEP_GNSS: {
		float speed = fastSqrtf(positionCordinateData.xVelocity * positionCordinateData.xVelocity + positionCordinateData.yVelocity * positionCordinateData.yVelocity);
		sendGNSSTelemetry(gnssData.latitude, gnssData.longitude, speed, fcStatusData.headingRef, fcStatusData.altitudeRef, gnssData.satCount);
	}
		break;

	case TELEMETRY_STEP_FC_STATUS:
		sendFCStatus();
		break;

	default:
		currentTelemetryStep = TELEMETRY_STEP_ALTITUDE;
		return;
	}
	// Advance to next frame step, wrapping around smoothly
	currentTelemetryStep++;
	if (currentTelemetryStep >= TELEMETRY_STEP_COUNT) {
		currentTelemetryStep = TELEMETRY_STEP_ALTITUDE;
	}
}

uint8_t initTelemetryManager() {
	schedulerAddTask(telemetryUpdateTask, TELEMETRY_TASK_FREQUENCY, TELEMETRY_TASK_PRIORITY);
	fcStatusData.isOSDEnabled = 1;
	logString("[Telemetry Manager] >> Init >> Success\n");
	return 1;
}
