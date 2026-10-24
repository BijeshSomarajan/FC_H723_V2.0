#include "PositionMissionHelper.h"

#include <math.h>
#include <stdio.h>
#include <sys/_stdint.h>

#include "../../../calibration/Calibration.h"
#include "../../../control/position/PositionControl.h"
#include "../../../logger/Logger.h"
#include "../../../memory/Memory.h"
#include "../../../sensors/groundStation/GroundStationSensor.h"
#include "../../../status/FCStatus.h"
#include "../../../util/MathUtil.h"
#include "../estimator/PositionEstimatorHelper.h"
#include "PositionManagerHelper.h"

float positionMissionVxCommand, positionMissionVyCommand;

float positionMissionWPCompleteDt = 0;
float positionMissionWPDistanceFiltered = 0;
uint8_t positionMissionWasRTHModeActive = 0;
uint8_t positionMissionWasMissionModeActive = 0;

float positionCruiseSpeed = POSITION_MISSION_CRUISE_SPEED_MIN;
float positionRTHSpeed = POSITION_MISSION_CRUISE_SPEED_MIN;

GroundStationSensorWPData *currentGroundStationSensorWPData;


void initPositionMissionHelper() {
	positionCruiseSpeed = fabs(get1KXScaledCalibrationValue(CALIB_PROP_POS_HOLD_CRUISE_SPEED_ADDR));
	if (positionCruiseSpeed > POSITION_MISSION_CRUISE_SPEED_MAX) {
		positionCruiseSpeed = POSITION_MISSION_CRUISE_SPEED_MIN;
	}

	positionRTHSpeed = fabs(get1KXScaledCalibrationValue(CALIB_PROP_POS_HOLD_RTH_SPEED_ADDR));
	if (positionRTHSpeed > positionCruiseSpeed) {
		positionRTHSpeed = positionCruiseSpeed * 0.8f;
	}

	char buf[64];
	sprintf(buf, "[PositionMissionHelper] RS:%.3f,CS:%.3f\n", positionRTHSpeed, positionCruiseSpeed);
	logString(buf);
}


void groundStationMissionCallBack(uint8_t action) {
	if (action == NAV_ACTION_START_MISSION) {
		resetNavMissionStates();
		fcStatusData.isNavMissionComplete = 0;
		if (!isNavRTHModeActive()) { // RTH injects a synthetic mission
			fcStatusData.isNavMissionModeActive = 1;
			positionMissionWasMissionModeActive = 0;
		} else {
			positionMissionWasRTHModeActive = 0;
		}
		positionMissionWPCount = getGroundStationSensorWPDataCount();
		loadNextNavMissionWayPoint();
	} else if (action == NAV_ACTION_ABORT_MISSION) {
		resetNavMissionStates();
		fcStatusData.isNavMissionComplete = 0;
		fcStatusData.isNavMissionModeActive = 0;
	}
}


void handleWPOperations() {

}


__ATTR_ITCM_TEXT
void handleNavMission(float dt) {
	if (isNavRTHModeActive()) {
		positionMissionWasMissionModeActive = 0;
		fcStatusData.isNavMissionModeActive = 0;
		if (!positionMissionWasRTHModeActive) {
			clearGroundStationSensorWPData();
			GroundStationSensorWPData groundStationSensorWPData;
			groundStationSensorWPData.waypointIndex = 0;
			groundStationSensorWPData.latitude = fcStatusData.positionLatHome;
			groundStationSensorWPData.longitude = fcStatusData.positionLongHome;
			groundStationSensorWPData.status = NAV_STATUS_OPEN;
			GroundStationSensorOPData setRTHVelocityOperation;
			//RTH will be at a percentage of cruise speed
			setRTHVelocityOperation.value = positionRTHSpeed;
			setRTHVelocityOperation.operation = NAV_OPERATION_SET_VEL;
			groundStationSensorWPData.operations[0] = setRTHVelocityOperation;
			groundStationSensorWPData.operations[0].status = NAV_STATUS_OPEN;
			setGroundStationSensorWPData(groundStationSensorWPData);
			groundStationMissionCallBack(NAV_ACTION_START_MISSION);
			positionMissionWasRTHModeActive = 1;
		}
	} else {
		positionMissionWasRTHModeActive = 0;
		if (isNavMissionModeActive()) {
			if (!positionMissionWasMissionModeActive) {
				positionMissionWasMissionModeActive = 1;
			}
		}
	}
	updateMissionVelocityCommand(dt);

	//This is to enable stickiness to current position if Mission is aborted.
	updatePositionReference();
	updateWPCompletionStatus(dt);

	if (isCurrentNavWPComplete()) {
		positionMissionWPIndx++;
		resetNavWPStates();
		uint8_t hasMoreWP = loadNextNavMissionWayPoint();
		if (!hasMoreWP) {
			fcStatusData.isNavMissionComplete = 1;
			fcStatusData.isNavMissionModeActive = 0;
			updatePositionReferenceToWPRef(); // The last mission WP reference is taken as the new anchor
			if (fcStatusData.isFailSafeModeActive) {
				//Trigger Landing
				fcStatusData.isLandingModeActive = 1;
				fcStatusData.isFailSafeModeActive = 0;
			}
		} else {
			//Manage WP Operations

		}
	} else {
		updateCurrentNavWPStatus(NAV_STATUS_WIP);
	}
}

