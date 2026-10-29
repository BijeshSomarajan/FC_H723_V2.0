#include "TelemetryManager.h"

#include <stdio.h>
#include <string.h>
#include <sys/_stdint.h>

#include "../../FCConfig.h"
#include "../../io/uart/UART.h"
#include "../../logger/Logger.h"
#include "../../sensors/attitude/AttitudeSensor.h"
#include "../../sensors/battery/BatterySensor.h"
#include "../../sensors/position/GNSS.h"
#include "../../sensors/rc/RCTelemetry.h"
#include "../../status/FCStatus.h"
#include "../../timers/Scheduler.h"
#include "../../util/MathUtil.h"
#include "../altitude/helpers/AltitudeCommandHelper.h"
#include "../position/common/PositionCommon.h"
#include "../position/estimator/PositionEstimatorHelper.h"
#include "../position/helpers/PositionManagerHelper.h"
#include "../mission/MissionManager.h"

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

char fcStatusBuf[16];
char osdBuf[100];
uint8_t satCountAndReliability = 0;
float homeDistance = 0;
float groundSpeed = 0;
float expectedGroundSpeed = 0;

TelemetryStep currentTelemetryStep = TELEMETRY_STEP_ALTITUDE;

// Mission progress, derived here from state MissionManager already exposes
uint8_t tmWpDone = 0;            // waypoints reached since START
uint8_t tmOpSeq = 0;             // 0-9, steps when an announceable op finishes
char tmOpChar = '0';          // T/L/A/W: last finished op
uint8_t tmWpTotal = 0;
uint8_t tmLastWpIdx = 0;
uint8_t tmLastOpsDone = 0;
uint8_t tmMissionWasActive = 0;
uint8_t tmRthContext = 0;
uint8_t tmCompleteKind = 0;      // 0 none, 1 mission complete, 2 RTH complete

char tmOpTypeChar(uint8_t op) {
	char opChar = 'U';
	switch (op) {
	case MISSION_OPERATION_TAKEOFF:
		opChar = 'T';
		break;
	case MISSION_OPERATION_LAND:
		opChar = 'L';
		break;
	case MISSION_OPERATION_SET_ALT:
		opChar = 'A';
		break;
	case MISSION_OPERATION_WAIT:
		opChar = 'W';
		break;
	case MISSION_OPERATION_SET_HEADING:
		opChar = 'H';
		break;
	case MISSION_OPERATION_CUSTOM:
		opChar = 'C';
		break;
	default:
		opChar = 'U';
		break;
	}
	return opChar;
}

void updateMissionProgress(void) {
	uint8_t rthContext = (isRTHModeActive() || isFailSafeModeActive()) ? 1 : 0;
	uint8_t missionActive = fcStatusData.isMissionModeActive ? 1 : 0;
// New mission: count from zero
	if (missionActive && !tmMissionWasActive) {
		tmWpDone = 0;
		tmOpSeq = 0;
		tmOpChar = '0';
		tmLastWpIdx = 0;
		tmLastOpsDone = 0;
		tmCompleteKind = 0;
	}

// Completion event: MissionManager sets the latch, it is consumed here
	if (fcStatusData.isMissionCompleteLatch) {
		fcStatusData.isMissionCompleteLatch = 0;
		if (rthContext || tmRthContext) {
			tmCompleteKind = 2;
		} else if (tmMissionWasActive) {
			tmCompleteKind = 1;
			tmWpDone = tmWpTotal;         // the final waypoint is never seen advancing
		}
	}

// Progress while a mission is running
	if (missionActive) {
		MissionData *m = currentMissionData;      // read once, then bounds-check
		MissionWPData *wp = currentMissionWPData;
		if (m != NULL && wp != NULL && m->waypointCount <= NAV_MAX_WAYPOINTS) {
			uint8_t wpIdx = m->currentWaypointIndex;
			uint8_t opCount = wp->operationCount;
			if (opCount > NAV_MAX_OPERATIONS) {
				opCount = NAV_MAX_OPERATIONS;
			}
			tmWpTotal = m->waypointCount;
			if (wpIdx > tmLastWpIdx) {
				tmWpDone = wpIdx;
				tmLastWpIdx = wpIdx;
				tmLastOpsDone = 0;
			} else if (wpIdx < tmLastWpIdx) {      // reset underneath us: resync
				tmLastWpIdx = wpIdx;
				tmLastOpsDone = 0;
			}
			uint8_t opsDone = wp->currentOperationIndex;
			if (wp->status == MISSION_STATUS_WIP_CONCLUDING || wp->status == MISSION_STATUS_COMPLETE) {
				opsDone = opCount;                 // navigating to the waypoint: all ops finished
			}
			if (opsDone > opCount) {
				opsDone = opCount;
			}
			if (opsDone > tmLastOpsDone) {
				char c = 0;
				for (uint8_t i = tmLastOpsDone; i < opsDone; i++) {
					char t = tmOpTypeChar(wp->operations[i].operation);
					if (t) {
						c = t;
					}
				}
				if (c) {
					tmOpChar = c;
					tmOpSeq = (tmOpSeq + 1) % 10;
				}
			}
			tmLastOpsDone = opsDone;
		}
	}
// How long the "complete" display lives
	if (tmCompleteKind == 1 && ((!isNavModeActive() && !rthContext) || (rthContext && !tmRthContext))) {
		tmCompleteKind = 0;
	}
	if (tmCompleteKind == 2 && !rthContext && !tmRthContext) {
		tmCompleteKind = 0;
	}
	tmRthContext = rthContext;
	tmMissionWasActive = missionActive;
}

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

// Stab, Nav,Cruise , RTH
	fcStatusBuf[1] = '-';
	if (isFailSafeModeActive()) {
		fcStatusBuf[2] = (tmCompleteKind == 2) ? 'C' : 'F';
	} else if (isRTHModeActive()) {
		fcStatusBuf[2] = (tmCompleteKind == 2) ? 'C' : 'R';
	} else if (isCruiseModeActive()) {
		fcStatusBuf[2] = 'V'; //Cruise Mode
	} else if (isNavModeActive()) {
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
	if (isAltCommandComplete()) {
		fcStatusBuf[6] = 'C';
	} else if (fcStatusData.isLandingModeActive && isAltCommandMode()) {
		fcStatusBuf[6] = 'L';
	} else if (fcStatusData.isTakeOffModeActive && isAltCommandMode()) {
		fcStatusBuf[6] = 'T';
	} else {
		fcStatusBuf[6] = 'F';
	}

//Mission
	fcStatusBuf[7] = '-';
	if (isNavModeActive() && !isRTHModeActive()) {
		if (tmCompleteKind == 1) {
			fcStatusBuf[8] = 'C';
		} else if (isMissionModeActive()) {
			fcStatusBuf[8] = 'M';
		} else {
			fcStatusBuf[8] = 'N';
		}
	}

	fcStatusBuf[9] = '-';
	uint8_t wp = (tmWpDone > 99) ? 99 : tmWpDone;
	fcStatusBuf[10] = '0' + (wp / 10);
	fcStatusBuf[11] = '0' + (wp % 10);
	fcStatusBuf[12] = '-';
	fcStatusBuf[13] = '0' + (tmOpSeq % 10);
	fcStatusBuf[14] = tmOpChar;       // always '0' or T/L/A/W

	fcStatusBuf[15] = '\0';

}

void prepareGNSSData(void) {
	satCountAndReliability = (gnssData.satCount & 0x3F) | ((uint8_t) (fcStatusData.isNavDataReliable && fcStatusData.isPositionHomeSet) << 6);
	if (isPositionHomeSet()) {
		float north = positionCordinateData.xPositionRaw;
		float east = positionCordinateData.yPositionRaw;
		homeDistance = fastSqrtf(north * north + east * east);
		groundSpeed = getGroundSpeed();
	} else {
		homeDistance = 0.0f;
		groundSpeed = 0.0f;
	}

}

void sendOSDData() {
//rxBat,rxBatMax,alt,homeDistance,heading,headingRef,verticalSpeed,groundSpeed,satField,fm,pitch,roll,throttle,batteryAlertState
	sprintf(osdBuf, "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%s,%.1f,%.1f,%lu,%d\n", batteryData.voltage, fcStatusData.batteryNomVolt, positionCordinateData.zPosition, homeDistance, sensorAttitudeData.heading, fcStatusData.headingHomeRef, expectedGroundSpeed, groundSpeed, satCountAndReliability,
			fcStatusBuf, sensorAttitudeData.pitch, sensorAttitudeData.roll, (uint32_t) fcStatusData.currentThrottle, fcStatusData.batteryAlertState);

	uart8WriteDMA((uint8_t*) osdBuf, strlen(osdBuf));
}

/**
 * @brief Main task executed by scheduler.
 * Sends exactly ONE type of telemetry per execution tick.
 */
void telemetryUpdateTask() {
	updateMissionProgress();

	switch (currentTelemetryStep) {
	case TELEMETRY_STEP_ALTITUDE:
		if (isCruiseModeActive()) {
			expectedGroundSpeed = fastSqrtf(positionCommandData.targetXVel * positionCommandData.targetXVel + positionCommandData.targetYVel * positionCommandData.targetYVel);
		} else {
			expectedGroundSpeed = 0.0f;
		}
		sendAltitudeTelemetry(positionCordinateData.zPosition, expectedGroundSpeed);
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
		sendFlightModeTelemetry(fcStatusBuf, 16);
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
