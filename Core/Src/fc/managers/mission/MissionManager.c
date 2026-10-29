#include "MissionManager.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
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

// ---------------------------------------------------------------------------
// Flight trace. Logged on transitions only (start / next step / complete),
// never from per-cycle paths. Integers only (no %f, so it prints on newlib-nano):
//   *100 values are float * 100 truncated to a long, e.g. 3.0 -> 300
// Set MISSION_MGR_TRACE_ENABLED to 0 (header or build flag) to compile it out.
// ---------------------------------------------------------------------------
#ifndef MISSION_MGR_TRACE_ENABLED
#define MISSION_MGR_TRACE_ENABLED 1
#endif

#if MISSION_MGR_TRACE_ENABLED == 1
#define MISSION_TRACE(...) do { char _mmBuf[96]; snprintf(_mmBuf, sizeof(_mmBuf), __VA_ARGS__); logString(_mmBuf); } while (0)
#else
#define MISSION_TRACE(...) ((void)0)
#endif

#define MM_X100(v) ((long)((v) * 100.0f))

void _missionCallBack(uint8_t action);
void resetWPStates();
uint8_t loadCurrentMissionWPData();
uint8_t loadNextMissionWPData();
uint8_t loadCurrentMissionOperationData();
uint8_t loadNextMissionOperationData();

MissionData *currentMissionData = NULL;

MissionWPData *currentMissionWPData = NULL;
MissionOperationData *currentMissionOperationData = NULL;

MissionData rthMissionData;

float missionRTHSpeed = MISSION_MGR_SPEED_MIN;
float missionCruiseSpeed = MISSION_MGR_SPEED_MIN;
float missionWPCompleteDt = 0;
float missionWPDistanceFiltered = 0;
float missionVxCommand, missionVyCommand;
uint8_t missionWasRTHModeActive = 0;
uint8_t missionWasMissionModeActive = 0;
float missionWaitCounter = 0;

float missionPosHoldX = 0, missionPosHoldY = 0;
uint8_t missionPosHoldActive = 0;
uint8_t missionPosSettled = 0;
float missionPosSettleDt = 0;

__ATTR_ITCM_TEXT
void startOperation(float dt) {
	if (currentMissionOperationData == NULL) {
		return;
	}
	// altC / altS = alt-command complete / skipped flags at the moment the op starts.
	// If either is 1 here, they are stale and the op will finish instantly.
	MISSION_TRACE("[MM] OP start wp=%u op=%u code=%u val*100=%ld altC=%u altS=%u\n", (unsigned ) currentMissionData->currentWaypointIndex, (unsigned ) currentMissionWPData->currentOperationIndex, (unsigned ) currentMissionOperationData->operation, MM_X100(currentMissionOperationData->value),
			(unsigned ) isAltCommandComplete(), (unsigned ) isAltCommandSkipped());
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
		setTargetAltForAltChangeCommand(constrainToRangeF(currentMissionOperationData->value, MISSION_MGR_ALT_COMMAND_MIN, MISSION_MGR_ALT_COMMAND_MAX));
		fcStatusData.isAltChangeModeActive = 1;
		currentMissionOperationData->status = MISSION_STATUS_WIP;
		break;
	case MISSION_OPERATION_WAIT:
		missionWaitCounter = 0;
		currentMissionOperationData->status = MISSION_STATUS_WIP;
		break;
	case MISSION_OPERATION_SET_HEADING:
		currentMissionOperationData->status = MISSION_STATUS_COMPLETE; //Not supported yet
		MISSION_TRACE("[MM] OP skipped (heading not supported)\n");
		break;
	default:
		currentMissionOperationData->status = MISSION_STATUS_COMPLETE;  // unknown: skip
		MISSION_TRACE("[MM] OP skipped (unknown code=%u)\n", (unsigned ) currentMissionOperationData->operation);
		break;
	}
}

__ATTR_ITCM_TEXT
uint8_t checkOperation(float dt) {
	uint8_t complete = 0;
	if (currentMissionOperationData == NULL) {
		return 1;
	}
	switch (currentMissionOperationData->operation) {
	case MISSION_OPERATION_TAKEOFF:
		if (isAltCommandComplete() || isAltCommandSkipped()) {
			complete = 1;
		}
		break;
	case MISSION_OPERATION_LAND:
		if (isAltCommandComplete() || isAltCommandSkipped()) {
			complete = 1;
		}
		break;
	case MISSION_OPERATION_SET_ALT:
		if (isAltCommandComplete() || isAltCommandSkipped()) {
			complete = 1;
		}
		break;
	case MISSION_OPERATION_WAIT:
		missionWaitCounter += dt;
		if (missionWaitCounter >= currentMissionOperationData->value) {
			complete = 1;
		}
		break;
	case MISSION_OPERATION_SET_HEADING:
		complete = 1;
		break;
	default:
		complete = 1;
		break;
	}
	return complete;
}

__ATTR_ITCM_TEXT
void handleOperation(float dt) {
	if (currentMissionOperationData == NULL) {
		return;
	}
	if (currentMissionOperationData->status == MISSION_STATUS_OPEN) {
		startOperation(dt);
	} else if (currentMissionOperationData->status == MISSION_STATUS_WIP) {
		if (checkOperation(dt)) {
			currentMissionOperationData->status = MISSION_STATUS_COMPLETE;
			MISSION_TRACE("[MM] OP done wp=%u op=%u code=%u altC=%u altS=%u wait*100=%ld\n", (unsigned ) currentMissionData->currentWaypointIndex, (unsigned ) currentMissionWPData->currentOperationIndex, (unsigned ) currentMissionOperationData->operation, (unsigned ) isAltCommandComplete(),
					(unsigned ) isAltCommandSkipped(), MM_X100(missionWaitCounter));
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
	MISSION_TRACE("[MM] WP nav start wp=%u tgtX*100=%ld tgtY*100=%ld dist*100=%ld vel*100=%ld (wpVel*100=%ld cruise*100=%ld)\n", (unsigned ) currentMissionData->currentWaypointIndex, MM_X100(posX), MM_X100(posY), MM_X100(fastSqrtf(missionWPDistanceFiltered)),
			MM_X100(fcStatusData.positionVelMission), MM_X100(currentMissionWPData->velocity), MM_X100(missionCruiseSpeed));
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
		MISSION_TRACE("[MM] WP reached wp=%u gs*100=%ld dwell*100=%ld\n", (unsigned ) currentMissionData->currentWaypointIndex, MM_X100(getGroundSpeed()), MM_X100(missionWPCompleteDt));
	}
}

// Internal start: resets states and loads the first waypoint / operation
// of whatever is currently in the mission buffer. Not blocked by RTH mode,
// so startRTHMission() can use it.
uint8_t startLoadedMission(void) {
	resetMissionStates(0);
	if (currentMissionData == NULL) {
		currentMissionWPData = NULL;
		currentMissionOperationData = NULL;
		MISSION_TRACE("[MM] LOAD failed: no mission data\n");
		return 0;
	}
	if (!loadCurrentMissionWPData()) {
		currentMissionData->status = MISSION_STATUS_INVALID;
		currentMissionWPData = NULL;
		currentMissionOperationData = NULL;
		MISSION_TRACE("[MM] LOAD failed: no waypoints (count=%u)\n", (unsigned ) currentMissionData->waypointCount);
		return 0;
	}
	loadCurrentMissionOperationData();
	fcStatusData.isMissionComplete = 0;
	currentMissionData->status = MISSION_STATUS_WIP;
	MISSION_TRACE("[MM] LOAD ok rth=%u wps=%u wp0ops=%u\n", (unsigned ) (currentMissionData == &rthMissionData), (unsigned ) currentMissionData->waypointCount, (unsigned ) currentMissionWPData->operationCount);
	return 1;
}

// Stops a running mission directly (not through _missionCallBack, which
// ignores commands while RTH is active).
void stopMissionIfRunning(void) {
	if (currentMissionData != NULL && currentMissionData->status == MISSION_STATUS_WIP) {
		MISSION_TRACE("[MM] STOP running mission (wp=%u)\n", (unsigned ) currentMissionData->currentWaypointIndex);
		resetMissionStates(1);
		fcStatusData.isMissionComplete = 0;
	}
}

void startRTHMission() {
	resetMissionStates(1);
	rthMissionData.waypointCount = 1;
	rthMissionData.currentWaypointIndex = 0;
	MissionWPData *missionWPData = &rthMissionData.waypoints[0];
	missionWPData->latitude = fcStatusData.positionLatHome;
	missionWPData->longitude = fcStatusData.positionLongHome;
	missionWPData->velocity = missionRTHSpeed;
	missionWPData->status = MISSION_STATUS_OPEN;
	missionWPData->operationCount = 0;
	missionWPData->currentOperationIndex = 0;
	currentMissionData = &rthMissionData;
	MISSION_TRACE("[MM] RTH start spd*100=%ld failsafe=%u\n", MM_X100(missionRTHSpeed), (unsigned ) fcStatusData.isFailSafeModeActive);
	startLoadedMission();
}

__ATTR_ITCM_TEXT
void updatePositionReferenceToWPRef() {
	fcStatusData.positionXRef = fcStatusData.positionXRefMission;
	fcStatusData.positionYRef = fcStatusData.positionYRefMission;
}

__ATTR_ITCM_TEXT
void updatePositionReferenceToPresent() {
	fcStatusData.positionXRef = positionCordinateData.xPosition;
	fcStatusData.positionYRef = positionCordinateData.yPosition;
}

// Anchor at wherever the aircraft is right now
__ATTR_ITCM_TEXT
void beginMissionPositionHold(void) {
	missionPosHoldX = positionCordinateData.xPosition;
	missionPosHoldY = positionCordinateData.yPosition;
	missionPosSettleDt = 0;
	missionPosSettled = 0;
	missionPosHoldActive = 1;
	MISSION_TRACE("[MM] HOLD begin wp=%u x*100=%ld y*100=%ld gs*100=%ld\n", (unsigned ) currentMissionData->currentWaypointIndex, MM_X100(missionPosHoldX), MM_X100(missionPosHoldY), MM_X100(getGroundSpeed()));
}

// Leaving the op phase: seed the nav velocity command with the real
// velocity so doWPNavigation() doesn't start from a stale value
__ATTR_ITCM_TEXT
void endMissionPositionHold(void) {
	missionVxCommand = positionCordinateData.xVelocity;
	missionVyCommand = positionCordinateData.yVelocity;
	missionPosHoldActive = 0;
	MISSION_TRACE("[MM] HOLD end, all ops done, navigating wp=%u seed vx*100=%ld vy*100=%ld\n", (unsigned ) currentMissionData->currentWaypointIndex, MM_X100(missionVxCommand), MM_X100(missionVyCommand));
}

__ATTR_ITCM_TEXT
void missionPositionRelock(void) {
	missionPosHoldActive = 0;
	missionVxCommand = positionCordinateData.xVelocity;
	missionVyCommand = positionCordinateData.yVelocity;
	MISSION_TRACE("[MM] RELOCK after stick override seed vx*100=%ld vy*100=%ld\n", MM_X100(missionVxCommand), MM_X100(missionVyCommand));
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
			MISSION_TRACE("[MM] RTH off\n");
			stopMissionIfRunning();             // RTH switched off
		}
		missionWasRTHModeActive = 0;
		if (isMissionModeActive()) {
			missionWasMissionModeActive = 1;
		}
	}
	if (currentMissionData == NULL || currentMissionWPData == NULL) {
		if (!fcStatusData.isMissionComplete) {
			MISSION_TRACE("[MM] no mission data, marking complete\n");
		}
		fcStatusData.isMissionComplete = 1;
		fcStatusData.isMissionCompleteLatch = 1;
		return;
	}
	if (currentMissionData->status != MISSION_STATUS_WIP) {
		return;                                 // idle, aborted or complete: run nothing
	}

	if (currentMissionWPData->status == MISSION_STATUS_COMPLETE) {
		if (!loadNextMissionWPData()) {
			currentMissionData->status = MISSION_STATUS_COMPLETE;
			fcStatusData.isMissionComplete = 1;
			fcStatusData.isMissionCompleteLatch = 1;
			fcStatusData.isMissionModeActive = 0;
			updatePositionReferenceToWPRef();
			MISSION_TRACE("[MM] MISSION complete rth=%u failsafe=%u\n", (unsigned ) (currentMissionData == &rthMissionData), (unsigned ) fcStatusData.isFailSafeModeActive);
			if (fcStatusData.isFailSafeModeActive) {
				fcStatusData.isLandingModeActive = 1;
				fcStatusData.isFailSafeModeActive = 0;
				MISSION_TRACE("[MM] failsafe landing started\n");
			}
			return;
		}
		loadCurrentMissionOperationData();
		MISSION_TRACE("[MM] WP advance -> wp=%u/%u ops=%u\n", (unsigned ) (currentMissionData->currentWaypointIndex + 1), (unsigned ) currentMissionData->waypointCount, (unsigned ) currentMissionWPData->operationCount);
	}

	if (currentMissionWPData->operationCount == 0) {
		doWPNavigation(dt);
	} else if (currentMissionOperationData != NULL) {
		if (currentMissionOperationData->status == MISSION_STATUS_COMPLETE && !loadNextMissionOperationData()) {
			if (missionPosHoldActive) {
				endMissionPositionHold();
			}
			doWPNavigation(dt);
		} else {
			if (!missionPosHoldActive) {
				beginMissionPositionHold();
			}
			controlPositionCordinatesWithGains(dt, missionPosHoldX, missionPosHoldY, 1.0f);
			// Settle gate: only before the first op of this phase
			if (!missionPosSettled) {
				missionPosSettleDt += dt;
				uint8_t settleBySpeed = (getGroundSpeed() <= POSITION_MISSION_WP_COMPLETE_MAX_GROUND_SPEED);
				uint8_t settleByTimeout = (missionPosSettleDt >= POSITION_MISSION_OP_SETTLE_TIMEOUT);
				if (settleBySpeed || settleByTimeout) {
					missionPosSettled = 1;
					MISSION_TRACE("[MM] SETTLE done by %s t*100=%ld gs*100=%ld\n", settleBySpeed ? "speed" : "timeout", MM_X100(missionPosSettleDt), MM_X100(getGroundSpeed()));
				}
			}
			if (missionPosSettled) {
				handleOperation(dt);
			}
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

__ATTR_ITCM_TEXT
uint8_t loadCurrentMissionOperationData() {
	currentMissionOperationData = NULL;
	if (currentMissionWPData != NULL && currentMissionWPData->currentOperationIndex < currentMissionWPData->operationCount) {
		currentMissionOperationData = &currentMissionWPData->operations[currentMissionWPData->currentOperationIndex];
		return 1;
	}
	return 0;
}

__ATTR_ITCM_TEXT
uint8_t loadNextMissionOperationData() {
	if (currentMissionWPData != NULL) {
		uint8_t nextOperationIndx = currentMissionWPData->currentOperationIndex + 1;
		if (nextOperationIndx < currentMissionWPData->operationCount) {
			currentMissionWPData->currentOperationIndex++;
			currentMissionOperationData = &currentMissionWPData->operations[currentMissionWPData->currentOperationIndex];
			MISSION_TRACE("[MM] OP next wp=%u op=%u/%u\n", (unsigned ) currentMissionData->currentWaypointIndex, (unsigned ) (currentMissionWPData->currentOperationIndex + 1), (unsigned ) currentMissionWPData->operationCount);
			return 1;
		}
	}
	return 0;
}

void _missionCallBack(uint8_t action) {
	if (action == MISSION_ACTION_START) {
		if (fcStatusData.isMissionModeActive || isRTHModeActive()) {
			MISSION_TRACE("[MM] CMD start ignored (active=%u rth=%u)\n", (unsigned ) fcStatusData.isMissionModeActive, (unsigned ) isRTHModeActive());
			return;     // already running, or RTH owns the mission
		}
		currentMissionData = getMissionData();
		if (startLoadedMission()) {
			fcStatusData.isMissionModeActive = 1;
			MISSION_TRACE("[MM] CMD start accepted\n");
		} else {
			fcStatusData.isMissionModeActive = 0;
			MISSION_TRACE("[MM] CMD start rejected\n");
		}
	} else if (action == MISSION_ACTION_ABORT) {
		if (!isRTHModeActive() && fcStatusData.isMissionModeActive) { //Block reset while in RTH mode.
			resetMissionStates(1);
			fcStatusData.isMissionComplete = 0;
			fcStatusData.isMissionCompleteLatch = 0;
			fcStatusData.isMissionModeActive = 0;
			updatePositionReferenceToPresent();
			MISSION_TRACE("[MM] CMD abort done, holding x*100=%ld y*100=%ld\n", MM_X100(positionCordinateData.xPosition), MM_X100(positionCordinateData.yPosition));
		} else {
			MISSION_TRACE("[MM] CMD abort ignored (active=%u rth=%u)\n", (unsigned ) fcStatusData.isMissionModeActive, (unsigned ) isRTHModeActive());
		}
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

__ATTR_ITCM_TEXT
void resetRTHStates() {
	missionWasRTHModeActive = 0;
}

__ATTR_ITCM_TEXT
void resetWPStates() {
	missionWPCompleteDt = 0;
	missionWPDistanceFiltered = 0;
}

__ATTR_ITCM_TEXT
void resetMissionModeStates() {
	missionWasMissionModeActive = 0;
}

// NOTE: no trace here. The position manager calls this every cycle while idle.
__ATTR_ITCM_TEXT
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
	resetRTHStates();
	resetWPStates();

	fcStatusData.positionXRefMission = positionCordinateData.xPosition;
	fcStatusData.positionYRefMission = positionCordinateData.yPosition;
	fcStatusData.positionVelMission = MISSION_MGR_SPEED_MIN;
	missionVxCommand = 0;
	missionVyCommand = 0;
	missionPosHoldActive = 0;
	fcStatusData.isMissionComplete = 0;

}
