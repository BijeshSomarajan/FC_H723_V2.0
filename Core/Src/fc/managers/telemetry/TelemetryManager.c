#include "TelemetryManager.h"

#include <sys/_stdint.h>

#include "../../FCConfig.h"
#include "../../logger/Logger.h"
#include "../../sensors/attitude/AttitudeSensor.h"
#include "../../sensors/battery/BatterySensor.h"
#include "../../sensors/position/GNSS.h"
#include "../../sensors/rc/RCTelemetry.h"
#include "../../status/FCStatus.h"
#include "../../timers/Scheduler.h"
#include "../../util/MathUtil.h"
#include "../position/common/PositionCommon.h"
#include "../position/estimator/PositionEstimatorHelper.h"
#include "../../io/uart/UART.h"

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
 * Ground Speed     -> Ground Speed (m)
 * Heading          -> GNSS Heading Reference (deg) // Repurposed
 * Altitude         -> Distance // Repurposed
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
 * Vertical Speed   -> Vertical Speed
 *
 * -------------------- Flight Mode Frame ----------------------
 * Flight Mode      -> Flight Mode String
 */

char fcStatusBuf[10];
char osdBuf[100];
uint8_t satCountAndReliability = 0;
float homeDistance = 0;
float groundSpeed = 0;

TelemetryStep currentTelemetryStep = TELEMETRY_STEP_ALTITUDE;

void prepareFCStatus() {
	// Off, Start, Stab, Fly
	if (fcStatusData.hasCrashed) {
		fcStatusBuf[0] = 'C';
	} else if (fcStatusData.canFly) {
		fcStatusBuf[0] = 'F';
	} else if (fcStatusData.canStabilize) {
		fcStatusBuf[0] = 'S';
	} else if (!fcStatusData.canStart) {
		fcStatusBuf[0] = 'O';
	} else if (fcStatusData.canStart) {
		fcStatusBuf[0] = 'I';
	}
	// Loiter, Pos Hold, RTH
	fcStatusBuf[1] = '-';
	if (fcStatusData.isFailSafeModeActive) {
		fcStatusBuf[2] = 'F';
		if (fcStatusData.isNavMissionComplete) {
			fcStatusBuf[2] = 'C';
		}
	} else if (fcStatusData.isNavRTHModeActive) {
		if (fcStatusData.isNavMissionComplete) {
			fcStatusBuf[2] = 'C';
		} else {
			fcStatusBuf[2] = 'R';
		}
	} else if (fcStatusData.isNavModeActive) {
		fcStatusBuf[2] = 'N'; //Nav Mode
	} else {
		fcStatusBuf[2] = 'S'; // Stab Mode
	}
	// Terrain/Baro
	fcStatusBuf[3] = '-';
	if (fcStatusData.isTerrainAltModeActive && fcStatusData.isTerrainSensorExist) {
		fcStatusBuf[4] = 'T';
	} else {
		fcStatusBuf[4] = 'B';
	}
	// Landing/Flying
	fcStatusBuf[5] = '-';
	if (fcStatusData.isLandingModeActive) {
		fcStatusBuf[6] = 'L';
	} else {
		fcStatusBuf[6] = 'F';
	}
	//Mission
	fcStatusBuf[7] = '-';
	if (fcStatusData.isNavModeActive && fcStatusData.isNavMissionModeActive && !fcStatusData.isNavRTHModeActive) {
		if (fcStatusData.isNavMissionComplete) {
			fcStatusBuf[8] = 'C';
		} else {
			fcStatusBuf[8] = 'M';
		}
	} else {
		fcStatusBuf[8] = 'N';
	}
	fcStatusBuf[9] = '\0';
}

void prepareGNSSData(void) {
	satCountAndReliability = (gnssData.satCount & 0x3F) | ((uint8_t) (fcStatusData.isNavDataReliable && fcStatusData.isPositionHomeSet) << 6);
	if (fcStatusData.isPositionHomeSet) {
		float north = positionCordinateData.xPositionRaw;
		float east = positionCordinateData.yPositionRaw;
		homeDistance = fastSqrtf(north * north + east * east);
		groundSpeed = getGroundSpeed();
	}

}

void sendOSDData() {
	//rxBat,rxBatMax,alt,homeDistance,heading,headingRef,verticalSpeed,groundSpeed,satField,fm,pitch,roll,throttle,batteryAlertState
	sprintf(osdBuf, "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%s,%.1f,%.1f,%lu,%d\n",
	        batteryData.voltage,
	        fcStatusData.batteryNomVolt,
	        positionCordinateData.zPosition,
	        homeDistance,
	        sensorAttitudeData.heading,
	        fcStatusData.headingHomeRef,
	        positionCordinateData.zVelocity,
	        groundSpeed,
	        satCountAndReliability,
	        fcStatusBuf,
	        sensorAttitudeData.pitch,
	        sensorAttitudeData.roll,
	        (uint32_t) fcStatusData.currentThrottle,
			fcStatusData.batteryAlertState

	);

	uart8WriteDMA((uint8_t *)osdBuf, strlen(osdBuf));
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
		sendBatteryTelemetry(batteryData.voltage, fcStatusData.batteryNomVolt, (uint32_t) fcStatusData.currentThrottle, fcStatusData.batteryAlertState);
		break;
	case TELEMETRY_STEP_GNSS:
		prepareGNSSData();
		sendGNSSTelemetry(gnssData.latitude, gnssData.longitude, groundSpeed, fcStatusData.headingHomeRef, homeDistance, satCountAndReliability);
		break;
	case TELEMETRY_STEP_FC_STATUS:
		prepareFCStatus();
		sendFlightModeTelemetry(fcStatusBuf, 10);
		break;
	default:
		currentTelemetryStep = TELEMETRY_STEP_ALTITUDE;
		return;
	}

	// Advance to next frame step, wrapping around smoothly
	currentTelemetryStep++;
	if (currentTelemetryStep >= TELEMETRY_STEP_COUNT) {
		currentTelemetryStep = TELEMETRY_STEP_ALTITUDE;
		#if TELEMETRY_OSD_ENABLED == 1
				sendOSDData();
		#endif
	}
}

uint8_t initTelemetryManager() {

#if TELEMETRY_OSD_ENABLED == 1
	if (uart8Init()) {
		logString("[Telemetry Manager] >> OSD UART Init >> Success\n");
	}
#endif

	schedulerAddTask(telemetryUpdateTask, TELEMETRY_TASK_FREQUENCY, TELEMETRY_TASK_PRIORITY);
#if RC_RX_TYPE== RC_RX_TYPE_CRSF
	fcStatusData.isTelemetryEnabled = 1;
#else
	fcStatusData.isTelemetryEnabled = 0;
#endif
	logString("[Telemetry Manager] >> Init >> Success\n");
	return 1;
}
