#include "MissionManager.h"
#include "MissionNavigationHelper.h"

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
// Mission trace: short, transition-only lines (never from per-cycle paths).
// WP = waypoint index, OP = operation index within that waypoint.
// Set MISSION_MGR_TRACE_ENABLED to 0 (header or build flag) to compile it out.
// ---------------------------------------------------------------------------
#ifndef MISSION_MGR_TRACE_ENABLED
#define MISSION_MGR_TRACE_ENABLED 1
#endif

#if MISSION_MGR_TRACE_ENABLED == 1
#define MISSION_TRACE(...) do { char _mmBuf[48]; snprintf(_mmBuf, sizeof(_mmBuf), __VA_ARGS__); logString(_mmBuf); } while (0)
#else
#define MISSION_TRACE(...) ((void)0)
#endif

#define MM_WP() ((unsigned) currentMissionData->currentWaypointIndex)
#define MM_OP() ((unsigned) currentMissionWPData->currentOperationIndex)

void _missionCallBack(uint8_t action);
void resetWPStates();
uint8_t loadCurrentMissionWPData();
uint8_t loadNextMissionWPData();
uint8_t loadCurrentMissionOperationData();
uint8_t loadNextMissionOperationData();
void updatePositionReferenceToPresent();
void stopMissionIfRunning(void);
void updatePositionReferenceToWPRef();

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

__ATTR_ITCM_TEXT
void startOperation(float dt) {
	if (currentMissionOperationData == NULL) {
		return;
	}
	MISSION_TRACE("[MM] WP%u OP%u start code=%u\n", MM_WP(), MM_OP(), (unsigned ) currentMissionOperationData->operation);
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
	case MISSION_OPERATION_SET_ALT: {
		float altCmd = constrainToRangeF(currentMissionOperationData->value, MISSION_MGR_ALT_COMMAND_MIN, MISSION_MGR_ALT_COMMAND_MAX);
		if (altCmd != currentMissionOperationData->value) {
			MISSION_TRACE("[MM] WP%u OP%u alt clamped\n", MM_WP(), MM_OP());
		}
		fcStatusData.isTakeOffModeActive = 0;
		fcStatusData.isLandingModeActive = 0;
		setTargetAltForAltChangeCommand(altCmd);
		fcStatusData.isAltChangeModeActive = 1;
		currentMissionOperationData->status = MISSION_STATUS_WIP;
		break;
	}
	case MISSION_OPERATION_WAIT:
		missionWaitCounter = 0;
		currentMissionOperationData->status = MISSION_STATUS_WIP;
		break;
	case MISSION_OPERATION_SET_HEADING:
		currentMissionOperationData->status = MISSION_STATUS_COMPLETE; //Not supported yet
		MISSION_TRACE("[MM] WP%u OP%u skip (heading)\n", MM_WP(), MM_OP());
		break;
	default:
		currentMissionOperationData->status = MISSION_STATUS_COMPLETE;  // unknown: skip
		MISSION_TRACE("[MM] WP%u OP%u skip (unknown)\n", MM_WP(), MM_OP());
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
uint8_t handleWPOperation(float dt) {
	uint8_t operationComplete = 0;
	if (currentMissionOperationData == NULL) {
		operationComplete = 1;
	} else if (currentMissionOperationData->status == MISSION_STATUS_OPEN) {
		startOperation(dt);
	} else if (currentMissionOperationData->status == MISSION_STATUS_WIP) {
		operationComplete = checkOperation(dt);
	} else {
		operationComplete = 1;
	}
	return operationComplete;
}

__ATTR_ITCM_TEXT
uint8_t doWPOperations(float dt) {
	uint8_t operationsComplete = 0;
	if (currentMissionOperationData == NULL) {
		operationsComplete = 1;
	} else if (handleWPOperation(dt)) {
		currentMissionOperationData->status = MISSION_STATUS_COMPLETE;
		MISSION_TRACE("[MM] WP%u OP%u done\n", MM_WP(), MM_OP());
		if (!loadNextMissionOperationData()) {
			operationsComplete = 1;
		}
	}
	return operationsComplete;
}

__ATTR_ITCM_TEXT
void handleWPCompletion(float dt) {
	if (!loadNextMissionWPData()) {
		currentMissionData->status = MISSION_STATUS_COMPLETE;
		fcStatusData.isMissionComplete = 1;
		fcStatusData.isMissionCompleteLatch = 1;
		fcStatusData.isMissionModeActive = 0;
		updatePositionReferenceToWPRef();
		MISSION_TRACE("[MM] MISSION complete rth=%u fs=%u\n", (unsigned ) (currentMissionData == &rthMissionData), (unsigned ) fcStatusData.isFailSafeModeActive);
		if (fcStatusData.isFailSafeModeActive) {
			fcStatusData.isLandingModeActive = 1;
			fcStatusData.isFailSafeModeActive = 0;
			MISSION_TRACE("[MM] failsafe landing\n");
		}
	}
}

__ATTR_ITCM_TEXT
void handleMission(float dt) {
	//Mode selection and switch over
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
	//Validation Phase
	if (currentMissionData == NULL || currentMissionWPData == NULL) {
		if (!fcStatusData.isMissionComplete) {
			MISSION_TRACE("[MM] no mission data\n");
		}
		fcStatusData.isMissionComplete = 1;
		fcStatusData.isMissionCompleteLatch = 1;
		return;
	}
	if (currentMissionData->status != MISSION_STATUS_WIP) {
		return;                                 // idle, aborted or complete: run nothing
	}
	//Navigation and Operation execution
	if (currentMissionWPData->status == MISSION_STATUS_COMPLETE) {
		handleWPCompletion(dt);
	} else if (currentMissionWPData->status == MISSION_STATUS_WIP || currentMissionWPData->status == MISSION_STATUS_WIP_NAV) {
		if (doWPNavigation(dt)) {
			currentMissionWPData->status = MISSION_STATUS_WIP_OP;
			MISSION_TRACE("[MM] WP%u OPS begin n=%u\n", MM_WP(), (unsigned ) currentMissionWPData->operationCount);
		}
	} else if (currentMissionWPData->status == MISSION_STATUS_WIP_OP) {
		if (fcStatusData.isFlying && fcStatusData.isPositionHomeSet) {
			controlPositionCordinatesWithGains(dt, fcStatusData.positionXRefMission, fcStatusData.positionYRefMission, 1.0f);
		} else if (!fcStatusData.isFlying) {
			fcStatusData.positionXRefMission = positionCordinateData.xPosition;   // on the ground: follow, so liftoff locks here
			fcStatusData.positionYRefMission = positionCordinateData.yPosition;
		}
		if (currentMissionWPData->operationCount > 0) {
			if (doWPOperations(dt)) {
				currentMissionWPData->status = MISSION_STATUS_COMPLETE;
				MISSION_TRACE("[MM] WP%u done\n", MM_WP());
			}
		} else {
			currentMissionWPData->status = MISSION_STATUS_COMPLETE;
			MISSION_TRACE("[MM] WP%u done (no ops)\n", MM_WP());
		}
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
		MISSION_TRACE("[MM] LOAD failed: no data\n");
		return 0;
	}
	if (!loadCurrentMissionWPData()) {
		currentMissionData->status = MISSION_STATUS_INVALID;
		currentMissionWPData = NULL;
		currentMissionOperationData = NULL;
		MISSION_TRACE("[MM] LOAD failed: no waypoints\n");
		return 0;
	}

	fcStatusData.isMissionComplete = 0;
	currentMissionData->status = MISSION_STATUS_WIP;
	MISSION_TRACE("[MM] LOAD ok wps=%u wp0ops=%u rth=%u\n", (unsigned ) currentMissionData->waypointCount, (unsigned ) currentMissionWPData->operationCount, (unsigned ) (currentMissionData == &rthMissionData));
	return 1;
}

void startRTHMission() {
	if (fcStatusData.isFlying) {
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
		MISSION_TRACE("[MM] RTH start fs=%u\n", (unsigned ) fcStatusData.isFailSafeModeActive);
		startLoadedMission();
	} else {
		MISSION_TRACE("[MM] RTH skipped (not flying)\n");
		resetMissionStates(1);
		fcStatusData.isMissionComplete = 1;     // set after the reset, which clears it
	}
}

// Stops a running mission directly (not through _missionCallBack, which
// ignores commands while RTH is active).
void stopMissionIfRunning(void) {
	if (currentMissionData != NULL && currentMissionData->status == MISSION_STATUS_WIP) {
		MISSION_TRACE("[MM] STOP at WP%u\n", MM_WP());
		resetMissionStates(1);
		fcStatusData.isMissionComplete = 0;
	}
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
uint8_t loadCurrentMissionWPData() {
	if (currentMissionData != NULL) {
		if (currentMissionData->currentWaypointIndex < currentMissionData->waypointCount) {
			currentMissionWPData = &currentMissionData->waypoints[currentMissionData->currentWaypointIndex];
			currentMissionWPData->status = MISSION_STATUS_WIP;
			loadCurrentMissionOperationData();
			return 1;
		}
	}
	return 0;
}

__ATTR_ITCM_TEXT
uint8_t loadNextMissionWPData() {
	if (currentMissionData != NULL) {
		uint8_t nextWPIndx = currentMissionData->currentWaypointIndex + 1;
		if (nextWPIndx < currentMissionData->waypointCount) {
			currentMissionData->currentWaypointIndex++;
			currentMissionWPData = &currentMissionData->waypoints[currentMissionData->currentWaypointIndex];
			currentMissionWPData->status = MISSION_STATUS_WIP;
			loadCurrentMissionOperationData();
			return 1;
		}
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
			return 1;
		}
	}
	return 0;
}

void _missionCallBack(uint8_t action) {
	if (action == MISSION_ACTION_START) {
		if (!fcStatusData.isMissionModeActive && !isRTHModeActive()) {   // not already running, RTH does not own the mission
			currentMissionData = getMissionData();
			fcStatusData.isMissionModeActive = startLoadedMission() ? 1 : 0;
			MISSION_TRACE("[MM] CMD start %s\n", fcStatusData.isMissionModeActive ? "ok" : "rejected");
		} else {
			MISSION_TRACE("[MM] CMD start ignored\n");
		}
	} else if (action == MISSION_ACTION_ABORT) {
		if (!isRTHModeActive() && fcStatusData.isMissionModeActive) { //Block reset while in RTH mode.
			resetMissionStates(1);
			fcStatusData.isMissionComplete = 0;
			fcStatusData.isMissionCompleteLatch = 0;
			fcStatusData.isMissionModeActive = 0;
			updatePositionReferenceToPresent();
			MISSION_TRACE("[MM] CMD abort ok\n");
		} else {
			MISSION_TRACE("[MM] CMD abort ignored\n");
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
	rthMissionData.status = MISSION_STATUS_INVALID;
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

// Note: the position manager calls this every cycle while idle, so no trace here.
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
	fcStatusData.isMissionComplete = 0;
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

