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

void startOperation() {
	if (currentMissionOperationData == NULL) {
		return;
	}
	switch (currentMissionOperationData->operation) {
	case MISSION_OPERATION_TAKEOFF:
		//To initiate Take off
		break;
	case MISSION_OPERATION_LAND:
		//To initiate Landing
		break;
	case MISSION_OPERATION_SET_ALT:
		//To initiate alt setting
		break;
	case MISSION_OPERATION_SET_HEADING:
		//To initiate heading set
		break;
	case MISSION_OPERATION_WAIT:
		//To initiate wait
		break;
	}
}

uint8_t checkOperation() {
	uint8_t complete = 1;
	if (currentMissionOperationData == NULL) {
		return complete;
	}
	switch (currentMissionOperationData->operation) {
	case MISSION_OPERATION_TAKEOFF:
		//To check Take off completion
		break;
	case MISSION_OPERATION_LAND:
		//To check Landing completion
		break;
	case MISSION_OPERATION_SET_ALT:
		//To check Alt completion
		break;
	case MISSION_OPERATION_SET_HEADING:
		//To check set heading completion
		break;
	case MISSION_OPERATION_WAIT:
		//To check wait completion
		break;
	}
	return complete;
}

void handleOperation() {
	if (currentMissionOperationData == NULL) {
		return;
	}
	if (currentMissionOperationData->status == MISSION_STATUS_OPEN) {
		currentMissionOperationData->status = MISSION_STATUS_WIP;
		startOperation();
	} else if (currentMissionOperationData->status == MISSION_STATUS_WIP) {
		if (checkOperation()) {
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
			handleOperation();
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
		startLoadedMission();
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
