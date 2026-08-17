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
#include "../../sensors/position/GNSS.h"
#include "../../sensors/battery/BatterySensor.h"
#include "../position/helpers/PositionManagerHelper.h"
#include "../position/PositionManager.h"

/* ============================================================================
 * BRHS Telemetry Mapping
 * ============================================================================
 *
 * Standard CRSF Fields have been repurposed to maximize useful flight data
 * while remaining compatible with EdgeTX telemetry and Lua scripts.
 *
 * ----------------------- Battery Frame -----------------------
 * Voltage          -> Battery Voltage (V)
 * Current          -> Nominal Voltage (A) //Repurposed
 * Capacity         -> Consumed Capacity (mAh)  //Repurposed
 * Remaining        -> Battery Alert (%)   //Repurposed
 *
 * ------------------------- GPS Frame -------------------------
 * Latitude         -> GNSS Latitude (deg)
 * Longitude        -> GNSS Longitude (deg)
 * Ground Speed     -> Distance to Home (m) //Repurposed
 * Heading          -> GNSS Heading Reference (deg) // Repurposed
 * Altitude         -> Home / Reference Altitude (m) // Repurposed
 * Satellites[6:0]  -> Satellite Count
 * Satellites[7]    -> Navigation Reliability Flag
 *
 * ---------------------- Attitude Frame -----------------------
 * Pitch            -> Aircraft Pitch (rad)
 * Roll             -> Aircraft Roll (rad)
 * Yaw              -> Aircraft Heading (rad)
 *
 * -------------------- Barometer Frame ------------------------
 * Altitude         -> EKF Relative Altitude (m)
 * Vertical Speed   -> Home Bearing (deg)   // Repurposed
 *
 * -------------------- Flight Mode Frame ----------------------
 * Flight Mode      -> Flight Mode String
 */
char FC_STATUS_BUF[10];
TelemetryStep currentTelemetryStep = TELEMETRY_STEP_ALTITUDE;

void prepareAndSendFCStatus() {
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
	if (fcStatusData.isNavRTHModeActive) {
		if (fcStatusData.isNavMissionComplete) {
			FC_STATUS_BUF[2] = 'C';
		} else {
			FC_STATUS_BUF[2] = 'R';
		}
	} else if (fcStatusData.isNavModeActive) {
		FC_STATUS_BUF[2] = 'N'; //Nav Mode
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
	//Mission
	FC_STATUS_BUF[7] = '-';
	if (fcStatusData.isNavModeActive && fcStatusData.isNavMissionModeActive && !fcStatusData.isNavRTHModeActive) {
		if (fcStatusData.isNavMissionComplete) {
			FC_STATUS_BUF[8] = 'C';
		} else {
			FC_STATUS_BUF[8] = 'M';
		}
	} else {
		FC_STATUS_BUF[8] = 'N';
	}
	FC_STATUS_BUF[9] = '\0';
	sendFlightModeTelemetry(FC_STATUS_BUF, 10);
}

void prepareAndSendGNSSData(void) {
	uint8_t satCountAndReliable = (gnssData.satCount & 0x3F) | ((uint8_t) (fcStatusData.isNavDataReliable && fcStatusData.isPositionHomeSet) << 6);
	float distance = 0.0f;
	if (fcStatusData.isPositionHomeSet) {
		float north = positionCordinateData.xPositionRaw;
		float east = positionCordinateData.yPositionRaw;
		distance = fastSqrtf(north * north + east * east);
	}
	sendGNSSTelemetry(gnssData.latitude, gnssData.longitude, distance, fcStatusData.headingRef, fcStatusData.altitudeRef, satCountAndReliable);
}

void prepareAndSendAltitudeData() {
	float bearing = calculateBearing(positionCordinateData.xPositionRaw, positionCordinateData.yPositionRaw);
	sendAltitudeTelemetry(positionCordinateData.zPosition, bearing);
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
		sendBatteryTelemetry(batteryData.voltage, fcStatusData.batteryNomVolt, (uint32_t)fcStatusData.currentThrottle, fcStatusData.batteryAlertState);
		break;
	case TELEMETRY_STEP_GNSS:
		prepareAndSendGNSSData();
		break;
	case TELEMETRY_STEP_FC_STATUS:
		prepareAndSendFCStatus();
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
#if RC_RX_TYPE== RC_RX_TYPE_CRSF
	fcStatusData.isTelemetryEnabled = 1;
#else
	fcStatusData.isTelemetryEnabled = 0;
#endif
	logString("[Telemetry Manager] >> Init >> Success\n");
	return 1;
}
