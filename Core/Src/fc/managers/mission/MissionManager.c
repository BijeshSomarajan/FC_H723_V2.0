#include "MissionManager.h"

#include <math.h>
#include <stddef.h>
#include <sys/_stdint.h>

#include "../../calibration/Calibration.h"
#include "../../control/position/PositionControl.h"
#include "../../logger/Logger.h"
#include "../../memory/Memory.h"
#include "../../sensors/groundStation/GroundStation.h"
#include "../../status/FCStatus.h"
#include "../../util/MathUtil.h"
#include "../position/common/PositionCommon.h"
#include "../position/estimator/PositionEstimatorHelper.h"
#include "../position/helpers/PositionManagerHelper.h"
#include "../altitude/helpers/AltitudeCommandHelper.h"

void _missionCallBack(uint8_t action);
void resetWPStates();
uint8_t loadCurrentMissionWPData();
uint8_t loadNextMissionWPData();
uint8_t loadCurrentMissionOperationData();
uint8_t loadNextMissionOperationData();

MissionData *currentMissionData = NULL;
MissionWPData *currentMissionWPData = NULL;
MissionOperationData *currentMissionOperationData = NULL;

float missionRTHSpeed = MISSION_MGR_SPEED_MIN;
float missionCruiseSpeed = MISSION_MGR_SPEED_MIN;
float missionWPCompleteDt = 0;
float missionWPDistanceFiltered = 0;
float missionVxCommand, missionVyCommand;
uint8_t missionWasRTHModeActive = 0;
uint8_t missionWasMissionModeActive = 0;
float missionWaitCounter = 0;

void startOperation(float dt) {
	if (currentMissionOperationData == NULL) {
		return;
	}
	switch (currentMissionOperationData->operation) {
	case MISSION_OPERATION_TAKEOFF:
		fcStatusData.isLandingModeActive = 0;
		fcStatusData.isAltChangeModeActive = 0;
		fcStatusData.isTakeOffModeActive = 1;
		currentMissionOperationData->status = MISSION_STATUS_WIP;
		break;
	case MISSION_OPERATION_LAND:
		fcStatusData.isTakeOffModeActive = 0;
		fcStatusData.isAltChangeModeActive = 0;
		fcStatusData.isLandingModeActive = 1;
		currentMissionOperationData->status = MISSION_STATUS_WIP;
		break;
	case MISSION_OPERATION_SET_ALT:
		fcStatusData.isTakeOffModeActive = 0;
		fcStatusData.isLandingModeActive = 0;
		setTargetAltForAltChangeCommand(currentMissionOperationData->value);
		fcStatusData.isAltChangeModeActive = 1;
		currentMissionOperationData->status = MISSION_STATUS_WIP;
		break;
	case MISSION_OPERATION_WAIT:
		missionWaitCounter = 0;
		currentMissionOperationData->status = MISSION_STATUS_WIP;
		break;
	case MISSION_OPERATION_SET_HEADING:
		currentMissionOperationData->status = MISSION_STATUS_COMPLETE; //Not supported yet
		break;
	}
}

uint8_t checkOperation(float dt) {
	uint8_t complete = 0;
	if (currentMissionOperationData == NULL) {
		return 1;
	}
	switch (currentMissionOperationData->operation) {
	case MISSION_OPERATION_TAKEOFF:
		if (isAltCommandComplete()) {
			complete = 1;
		}
		break;
	case MISSION_OPERATION_LAND:
		if (isAltCommandComplete()) {
			complete = 1;
		}
		break;
	case MISSION_OPERATION_SET_ALT:
		if (isAltCommandComplete()) {
			complete = 1;
		}
		break;
	case MISSION_OPERATION_WAIT:
		missionWaitCounter += dt;
		if (missionWaitCounter > currentMissionOperationData->value) {
			complete = 1;
		}
		break;
	case MISSION_OPERATION_SET_HEADING:
		complete = 1;
		break;
	}
	return complete;
}

void handleOperation(float dt) {
	if (currentMissionOperationData == NULL) {
		return;
	}
	if (currentMissionOperationData->status == MISSION_STATUS_OPEN) {
		currentMissionOperationData->status = MISSION_STATUS_WIP;
		startOperation(dt);
	} else if (currentMissionOperationData->status == MISSION_STATUS_WIP) {
		if (checkOperation(dt)) {
			currentMissionOperationData->status = MISSION_STATUS_COMPLETE;
		}
	}
}

__ATTR_ITCM_TEXT
void updateNavVelocityCommand(float dt) {
	// ---------------------------------------------------------------
	// Position error to waypoint
	// ---------------------------------------------------------------
	float dx = fcStatusData.positionXRefMission - positionCordinateData.xPosition;
	float dy = fcStatusData.positionYRefMission - positionCordinateData.yPosition;
	// ---------------------------------------------------------------
	// Distance to waypoint
	// ---------------------------------------------------------------
	float distance = fastSqrtf(dx * dx + dy * dy);
	// ---------------------------------------------------------------
	// Desired velocity
	// ---------------------------------------------------------------
	float desiredVx = 0.0f;
	float desiredVy = 0.0f;

	// ---------------------------------------------------------------
	// Navigation phase
	//
	// Navigation resumes automatically whenever the aircraft is
	// outside the stop radius.
	// ---------------------------------------------------------------
	if (distance > POSITION_MISSION_WP_STOP_RADIUS) {
		float invDist = 1.0f / distance;
		float dirX = dx * invDist;
		float dirY = dy * invDist;
		float remainingDistance = distance - POSITION_MISSION_WP_STOP_RADIUS;
		float brakingSpeed = fastSqrtf(2.0f * POSITION_MISSION_BRAKE_DECEL * remainingDistance);
		float targetSpeed = fminf(fcStatusData.positionVelMission, brakingSpeed);
		desiredVx = dirX * targetSpeed;
		desiredVy = dirY * targetSpeed;
	}

	// ---------------------------------------------------------------
	// Acceleration limiting
	// ---------------------------------------------------------------
	float dvx = desiredVx - missionVxCommand;
	float dvy = desiredVy - missionVyCommand;
	float deltaMag = fastSqrtf(dvx * dvx + dvy * dvy);
	float maxDelta = POSITION_MISSION_MAX_ACCEL * dt;
	if (deltaMag > maxDelta && deltaMag > 0.0001f) {
		float scale = maxDelta / deltaMag;
		dvx *= scale;
		dvy *= scale;
	}
	// ---------------------------------------------------------------
	// Smoothed mission velocity command
	// ---------------------------------------------------------------
	missionVxCommand += dvx;
	missionVyCommand += dvy;
	// ---------------------------------------------------------------
	// Feed into position / velocity controller
	// ---------------------------------------------------------------
	setExpectedPositionVelocity(dt, missionVxCommand, missionVyCommand);
}

__ATTR_ITCM_TEXT
uint8_t updateNavCompletionStatus(float dt) {
	float dx = fcStatusData.positionXRefMission - positionCordinateData.xPosition;
	float dy = fcStatusData.positionYRefMission - positionCordinateData.yPosition;
	float distSq = dx * dx + dy * dy;
	missionWPDistanceFiltered += POSITION_MISSION_WP_DISTANCE_LPF_ALPHA * (distSq - missionWPDistanceFiltered);

	// Squared compare: no sqrt needed
	uint8_t insideRadius = (missionWPDistanceFiltered <= POSITION_MISSION_WP_COMPLETE_RADIUS_SQ);
	uint8_t lowGroundSpeed = (getGroundSpeed() <= POSITION_MISSION_WP_COMPLETE_MAX_GROUND_SPEED);

	// Dwell timer: counts continuous time inside the capture radius
	if (insideRadius) {
		if (missionWPCompleteDt < POSITION_MISSION_WP_DWELL_TIMEOUT) {
			missionWPCompleteDt += dt;
		}
	} else {
		// Decay rather than hard-reset: a brief single-cycle excursion
		// (noise) shouldn't erase several seconds of accumulated dwell.
		missionWPCompleteDt = fmaxf(missionWPCompleteDt - dt, 0.0f);
	}
	// Normal path: inside radius and settled
	// Worst case:  inside radius for DWELL_TIMEOUT regardless of speed
	uint8_t dwellTimeout = (missionWPCompleteDt >= POSITION_MISSION_WP_DWELL_TIMEOUT);
	return (insideRadius && (lowGroundSpeed || dwellTimeout)) ? 1 : 0;
}

__ATTR_ITCM_TEXT
void loadWPNavCollaterals() {
	resetWPStates();
	float posX, posY;
	convertGNSSToXYCordinates(currentMissionWPData->latitude, currentMissionWPData->longitude, fcStatusData.positionLatHome, fcStatusData.positionLongHome, &posX, &posY);
	fcStatusData.positionXRefMission = posX;
	fcStatusData.positionYRefMission = posY;
	fcStatusData.positionVelMission = constrainToRangeF(fabsf(currentMissionWPData->velocity), MISSION_MGR_SPEED_MIN, missionCruiseSpeed);
	float dx = posX - positionCordinateData.xPosition;
	float dy = posY - positionCordinateData.yPosition;
	missionWPDistanceFiltered = dx * dx + dy * dy;   // seed with the real distance
	missionWPCompleteDt = 0;
}

__ATTR_ITCM_TEXT
void doWPNavigation(float dt) {
	if (currentMissionWPData->status != MISSION_STATUS_WIP_CONCLUDING) {
		loadWPNavCollaterals();
		currentMissionWPData->status = MISSION_STATUS_WIP_CONCLUDING;
	}
	updateNavVelocityCommand(dt);
	uint8_t navComplete = updateNavCompletionStatus(dt);
	if (navComplete) {
		currentMissionWPData->status = MISSION_STATUS_COMPLETE;
	}
}

/*----------------------- Debug ----------------------------*/

static const char* missionStatusName(uint8_t s) {
	switch (s) {
	case MISSION_STATUS_INVALID:
		return "INVALID";
	case MISSION_STATUS_OPEN:
		return "OPEN";
	case MISSION_STATUS_WIP:
		return "WIP";
	case MISSION_STATUS_WIP_CONCLUDING:
		return "CONCLUDING";
	case MISSION_STATUS_COMPLETE:
		return "COMPLETE";
	default:
		return "?";
	}
}

static const char* missionOpName(uint8_t op) {
	switch (op) {
	case MISSION_OPERATION_TAKEOFF:
		return "TAKEOFF";
	case MISSION_OPERATION_LAND:
		return "LAND";
	case MISSION_OPERATION_SET_ALT:
		return "SET_ALT";
	case MISSION_OPERATION_SET_HEADING:
		return "SET_HDG";
	case MISSION_OPERATION_WAIT:
		return "WAIT";
	case MISSION_OPERATION_CUSTOM:
		return "CUSTOM";
	default:
		return "?";
	}
}

void dumpMissionData(void) {
	char buf[112];
	MissionData *m = getMissionData();
	if (m == NULL) {
		logString("[Mission Dump] NULL\n");
		return;
	}

	snprintf(buf, sizeof(buf), "[Mission Dump] status=%s wpCount=%u curWP=%u\n", missionStatusName(m->status), (unsigned) m->waypointCount, (unsigned) m->currentWaypointIndex);
	logString(buf);

	// Clamp so corrupted counts can't walk past the arrays
	uint8_t wpCount = (m->waypointCount > NAV_MAX_WAYPOINTS) ? NAV_MAX_WAYPOINTS : m->waypointCount;
	if (wpCount != m->waypointCount) {
		logString("[Mission Dump] WARNING: waypointCount out of range\n");
	}

	for (uint8_t i = 0; i < wpCount; i++) {
		MissionWPData *wp = &m->waypoints[i];
		char wpMark = (wp == currentMissionWPData) ? '*' : ' ';

		// Fixed-point prints avoid needing float printf support in the libc build:
		// lat/lon in 1e-7 deg, velocity in 0.1 m/s
		long lat = (long) (wp->latitude * 1e7);
		long lon = (long) (wp->longitude * 1e7);
		long vel = (long) (wp->velocity * 10.0f);

		snprintf(buf, sizeof(buf), "%c WP[%u] idx=%u st=%s ops=%u curOp=%u lat=%ld lon=%ld vel=%ld\n", wpMark, (unsigned) i, (unsigned) wp->waypointIndex, missionStatusName(wp->status), (unsigned) wp->operationCount, (unsigned) wp->currentOperationIndex, lat, lon, vel);
		logString(buf);

		uint8_t opCount = (wp->operationCount > NAV_MAX_OPERATIONS) ? NAV_MAX_OPERATIONS : wp->operationCount;
		if (opCount != wp->operationCount) {
			logString("    WARNING: operationCount out of range\n");
		}

		for (uint8_t j = 0; j < opCount; j++) {
			MissionOperationData *op = &wp->operations[j];
			char opMark = (op == currentMissionOperationData) ? '*' : ' ';
			long val = (long) (op->value * 10.0f);

			snprintf(buf, sizeof(buf), "  %c OP[%u] %s(%u) st=%s val=%ld\n", opMark, (unsigned) j, missionOpName(op->operation), (unsigned) op->operation, missionStatusName(op->status), val);
			logString(buf);
		}
	}
}
/*---------------------- Debug End -------------------------*/

// Internal start: resets states and loads the first waypoint / operation
// of whatever is currently in the mission buffer. Not blocked by RTH mode,
// so startRTHMission() can use it.
static void startLoadedMission(void) {
	resetMissionStates(0);
	currentMissionData = getMissionData();
	if (currentMissionData == NULL) {
		return;
	}
	if (!loadCurrentMissionWPData()) {
		currentMissionData->status = MISSION_STATUS_INVALID;   // nothing to fly
		return;
	}
	loadCurrentMissionOperationData();
	fcStatusData.isMissionComplete = 0;
	currentMissionData->status = MISSION_STATUS_WIP;           // last, once everything is valid
}

// Stops a running mission directly (not through _missionCallBack, which
// ignores commands while RTH is active).
static void stopMissionIfRunning(void) {
	if (currentMissionData != NULL && currentMissionData->status == MISSION_STATUS_WIP) {
		resetMissionStates(1);
		fcStatusData.isMissionComplete = 0;
	}
}

void startRTHMission() {
	resetMissionStates(1);
	currentMissionData = getMissionData();
	if (currentMissionData == NULL) {
		return;
	}
	currentMissionData->waypointCount = 1;
	MissionWPData *missionWPData = &currentMissionData->waypoints[0];
	missionWPData->latitude = fcStatusData.positionLatHome;
	missionWPData->longitude = fcStatusData.positionLongHome;
	missionWPData->velocity = missionRTHSpeed;
	missionWPData->status = MISSION_STATUS_OPEN;
	missionWPData->operationCount = 0;
	missionWPData->currentOperationIndex = 0;
	startLoadedMission();
}

__ATTR_ITCM_TEXT
void updatePositionReferenceToWPRef() {
	fcStatusData.positionXRef = fcStatusData.positionXRefMission;
	fcStatusData.positionYRef = fcStatusData.positionYRefMission;
}

__ATTR_ITCM_TEXT
void handleMission(float dt) {
	if (isRTHModeActive()) {
		missionWasMissionModeActive = 0;
		fcStatusData.isMissionModeActive = 0;
		if (!missionWasRTHModeActive) {
			startRTHMission();
			missionWasRTHModeActive = 1;
		}
	} else {
		if (missionWasRTHModeActive) {
			stopMissionIfRunning();             // RTH switched off
		}
		missionWasRTHModeActive = 0;
		if (isMissionModeActive()) {
			missionWasMissionModeActive = 1;
		}
	}

	if (currentMissionData == NULL || currentMissionWPData == NULL) {
		fcStatusData.isMissionComplete = 1;
		return;
	}

	if (currentMissionData->status != MISSION_STATUS_WIP) {
		return;                                 // idle, aborted or complete: run nothing
	}

	if (currentMissionWPData->status == MISSION_STATUS_COMPLETE) {
		if (!loadNextMissionWPData()) {
			currentMissionData->status = MISSION_STATUS_COMPLETE;
			fcStatusData.isMissionComplete = 1;
			fcStatusData.isMissionModeActive = 0;
			updatePositionReferenceToWPRef(); // The last mission WP reference is taken as the new anchor
			if (fcStatusData.isFailSafeModeActive) {
				fcStatusData.isLandingModeActive = 1;
				fcStatusData.isFailSafeModeActive = 0;
			}
			return;
		}
		loadCurrentMissionOperationData();
	}

	if (currentMissionWPData->operationCount == 0) {
		doWPNavigation(dt);
	} else if (currentMissionOperationData != NULL) {
		if (currentMissionOperationData->status == MISSION_STATUS_COMPLETE) {
			if (!loadNextMissionOperationData()) {
				doWPNavigation(dt);
			}
		} else {
			handleOperation(dt);
		}
	}

}

uint8_t loadCurrentMissionWPData() {
	if (currentMissionData != NULL) {
		if (currentMissionData->currentWaypointIndex < currentMissionData->waypointCount) {
			currentMissionWPData = &currentMissionData->waypoints[currentMissionData->currentWaypointIndex];
			currentMissionWPData->status = MISSION_STATUS_WIP;
			return 1;
		}
	}
	return 0;
}

uint8_t loadNextMissionWPData() {
	if (currentMissionData != NULL) {
		uint8_t nextWPIndx = currentMissionData->currentWaypointIndex + 1;
		if (nextWPIndx < currentMissionData->waypointCount) {
			currentMissionData->currentWaypointIndex++;
			currentMissionWPData = &currentMissionData->waypoints[currentMissionData->currentWaypointIndex];
			currentMissionWPData->status = MISSION_STATUS_WIP;
			return 1;
		}
	}
	return 0;
}

uint8_t loadCurrentMissionOperationData() {
	if (currentMissionWPData != NULL) {
		if (currentMissionWPData->currentOperationIndex < currentMissionWPData->operationCount) {
			currentMissionOperationData = &currentMissionWPData->operations[currentMissionWPData->currentOperationIndex];
			currentMissionOperationData->status = MISSION_STATUS_WIP;
			return 1;
		}
	}
	return 0;
}

uint8_t loadNextMissionOperationData() {
	if (currentMissionWPData != NULL) {
		uint8_t nextOperationIndx = currentMissionWPData->currentOperationIndex + 1;
		if (nextOperationIndx < currentMissionWPData->operationCount) {
			currentMissionWPData->currentOperationIndex++;
			currentMissionOperationData = &currentMissionWPData->operations[currentMissionWPData->currentOperationIndex];
			currentMissionOperationData->status = MISSION_STATUS_WIP;
			return 1;
		}
	}
	return 0;
}

void _missionCallBack(uint8_t action) {
	if (isRTHModeActive()) {
		return;                                 // Failsafe or RTH cannot be cancelled.
	}
	if (action == MISSION_ACTION_START) {
		dumpMissionData();
		//startLoadedMission();
	} else if (action == MISSION_ACTION_ABORT) {
		resetMissionStates(1);
		fcStatusData.isMissionComplete = 0;
		fcStatusData.isMissionModeActive = 0;
	}
}

uint8_t initMissionManager(void) {
	uint8_t status = initGroundStation(_missionCallBack);
	if (status) {
		logString("[Mission Manager] Init > Success\n");
		missionRTHSpeed = fabsf(get1KXScaledCalibrationValue(CALIB_PROP_POS_HOLD_RTH_SPEED_ADDR));
		missionRTHSpeed = constrainToRangeF(missionRTHSpeed, MISSION_MGR_SPEED_MIN, MISSION_MGR_SPEED_MAX);

		missionCruiseSpeed = fabs(get1KXScaledCalibrationValue(CALIB_PROP_POS_HOLD_CRUISE_SPEED_ADDR));
		if (missionCruiseSpeed > MISSION_MGR_SPEED_MAX) {
			missionCruiseSpeed = MISSION_MGR_SPEED_MIN;
		}

		missionRTHSpeed = fabs(get1KXScaledCalibrationValue(CALIB_PROP_POS_HOLD_RTH_SPEED_ADDR));
		if (missionRTHSpeed > missionCruiseSpeed) {
			missionRTHSpeed = missionCruiseSpeed * 0.8f;
		}

		char buf[64];
		sprintf(buf, "[PositionMissionHelper] RS:%.3f,CS:%.3f\n", missionRTHSpeed, missionCruiseSpeed);
		logString(buf);

	} else {
		logString("[Mission Manager] Init > Failed!\n");
		return 0;
	}
	return 1;
}

void resetRTHStates() {
	missionWasRTHModeActive = 0;
}

void resetWPStates() {
	missionWPCompleteDt = 0;
	missionWPDistanceFiltered = 0;
}

void resetMissionModeStates() {
	missionWasMissionModeActive = 0;
}

void resetMissionStates(uint8_t abort) {
	MissionData *missionData = getMissionData();
	if (missionData != NULL) {
		missionData->currentWaypointIndex = 0;
		if (abort) {
			missionData->status = MISSION_STATUS_INVALID;
		} else {
			missionData->status = MISSION_STATUS_OPEN;
		}
		for (uint8_t i = 0; i < missionData->waypointCount; i++) {
			MissionWPData *wp = &missionData->waypoints[i];
			wp->status = MISSION_STATUS_OPEN;
			wp->currentOperationIndex = 0;
			for (uint8_t j = 0; j < wp->operationCount; j++) {
				wp->operations[j].status = MISSION_STATUS_OPEN;
			}
		}
	}
	missionVxCommand = 0;
	missionVyCommand = 0;
	resetRTHStates();
	resetWPStates();
	fcStatusData.positionXRefMission = positionCordinateData.xPosition;
	fcStatusData.positionYRefMission = positionCordinateData.yPosition;
	fcStatusData.positionVelMission = MISSION_MGR_SPEED_MIN;
}
