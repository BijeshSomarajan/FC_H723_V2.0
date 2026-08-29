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
uint8_t positionMissionWasRTHModeActive = 0;
uint8_t positionMissionWasNavMissionModeActive = 0;
uint8_t positionMissionWPCaptureLatched = 0;
int16_t positionMissionWPIndx = 0;
uint8_t positionMissionWPComplete = 0;
int16_t positionMissionWPCount = 0;
float positionCruiseSpeed = POSITION_MISSION_CRUISE_SPEED_DEFAULT;

uint8_t loadWayPoints(void);
void updateWPCompletionStatus(float dt);
void updateMissionVelocityCommand(float dt);

void initPositionMissionHelper() {
	positionCruiseSpeed = get1KXScaledCalibrationValue(CALIB_PROP_POS_HOLD_CRUISE_SPEED_ADDR);
	if (positionCruiseSpeed < 0 || positionCruiseSpeed > POSITION_MISSION_CRUISE_SPEED_MAX) {
		positionCruiseSpeed = POSITION_MISSION_CRUISE_SPEED_DEFAULT;
	}
}

void resetNavWPStates() {
	positionMissionWPComplete = 0;
	positionMissionWPCompleteDt = 0;
	positionMissionWPCaptureLatched = 0;
}

void resetNavRTHStates() {
	positionMissionWasRTHModeActive = 0;
}

void resetNavMissionModeStates() {
	positionMissionWasNavMissionModeActive = 0;
}

void resetNavMissionStates() {
	positionMissionVxCommand = 0;
	positionMissionVyCommand = 0;

	fcStatusData.isNavMissionComplete = 0;

	resetNavRTHStates();
	resetNavMissionModeStates();

	positionMissionWPIndx = 0;
	positionMissionWPCount = 0;

	fcStatusData.positionXRefMission = positionCordinateData.xPosition;
	fcStatusData.positionYRefMission = positionCordinateData.yPosition;
	resetNavWPStates();
}

__ATTR_ITCM_TEXT
uint8_t loadWayPoints() {
	if (positionMissionWPIndx >= 0 && positionMissionWPIndx < positionMissionWPCount) {
		GroundStationSensorWPData *groundStationSensorWPData = getGroundStationSensorWPData(positionMissionWPIndx);
		if (groundStationSensorWPData != NULL) {
			float posX, posY;
			convertGNSSToXYCordinates(groundStationSensorWPData->latitude, groundStationSensorWPData->longitude, fcStatusData.positionLatHome, fcStatusData.positionLongHome, &posX, &posY);
			fcStatusData.positionXRefMission = posX;
			fcStatusData.positionYRefMission = posY;
			return 1;
		}
	}
	return 0;
}

void groundStationMissionCallBack(uint8_t action) {
	if (action == NAV_ACTION_START_MISSION) {
		resetNavMissionStates();
		positionMissionWPCount = getGroundStationSensorWPDataCount();
		loadWayPoints();
	} else if (action == NAV_ACTION_ABORT_MISSION) {
		resetNavMissionStates();
	}
}

__ATTR_ITCM_TEXT
void handleNavMission(float dt) {
	if (fcStatusData.isNavRTHModeActive) {
		positionMissionWasNavMissionModeActive = 0;
		if (!positionMissionWasRTHModeActive) {
			clearGroundStationSensorWPData();
			GroundStationSensorWPData groundStationSensorWPData;
			groundStationSensorWPData.waypointIndex = 0;
			groundStationSensorWPData.latitude = fcStatusData.positionLatHome;
			groundStationSensorWPData.longitude = fcStatusData.positionLongHome;
			setGroundStationSensorWPData(groundStationSensorWPData);
			groundStationMissionCallBack(NAV_ACTION_START_MISSION);
			positionMissionWasRTHModeActive = 1;
		}
	} else {
		positionMissionWasRTHModeActive = 0;
		if (fcStatusData.isNavMissionModeActive) {
			if (!positionMissionWasNavMissionModeActive) {
				groundStationMissionCallBack(NAV_ACTION_START_MISSION);
				positionMissionWasNavMissionModeActive = 1;
			}
		}
	}
	updateMissionVelocityCommand(dt);
	updateWPCompletionStatus(dt);
	updatePositionReference();
	if (positionMissionWPComplete) {
		positionMissionWPIndx++;
		resetNavWPStates();
		uint8_t hasMoreWP = loadWayPoints();
		if (!hasMoreWP) {
			fcStatusData.isNavMissionComplete = 1;
		}
	}
}

__ATTR_ITCM_TEXT
void updateMissionVelocityCommand(float dt) {
	// Position error to waypoint
	float dx = fcStatusData.positionXRefMission - positionCordinateData.xPosition;
	float dy = fcStatusData.positionYRefMission - positionCordinateData.yPosition;
	// Distance to waypoint
	float distance = fastSqrtf(dx * dx + dy * dy);
	//------------------------------------------------------------------
	// Capture latch
	//------------------------------------------------------------------
	if (!positionMissionWPCaptureLatched && distance <= POSITION_MISSION_WP_CAPTURE_RADIUS) {
		positionMissionWPCaptureLatched = 1;
	}
	//------------------------------------------------------------------
	// Desired velocity
	//------------------------------------------------------------------
	float desiredVx = 0.0f;
	float desiredVy = 0.0f;
	/*
	 * Navigation phase
	 *
	 * Once capture is latched, we stay in capture mode even if
	 * wind subsequently pushes the vehicle outside the capture radius.
	 */
	if (!positionMissionWPCaptureLatched && distance > POSITION_MISSION_WP_CAPTURE_RADIUS) {
		float invDist = 1.0f / distance;
		float dirX = dx * invDist;
		float dirY = dy * invDist;
		float remainingDistance = distance - POSITION_MISSION_WP_CAPTURE_RADIUS;
		float brakingSpeed = fastSqrtf(2.0f * POSITION_MISSION_BRAKE_DECEL * remainingDistance);
		float targetSpeed = fminf(positionCruiseSpeed, brakingSpeed);
		desiredVx = dirX * targetSpeed;
		desiredVy = dirY * targetSpeed;
	}

	/*
	 * Capture phase:  desiredVx = 0 , desiredVy = 0
	 * The position controller is now responsible for holding
	 * the waypoint against wind/disturbances.
	 */
	//------------------------------------------------------------------
	// Acceleration limiting
	//------------------------------------------------------------------
	float dvx = desiredVx - positionMissionVxCommand;
	float dvy = desiredVy - positionMissionVyCommand;
	float deltaMag = fastSqrtf(dvx * dvx + dvy * dvy);
	float maxDelta = POSITION_MISSION_MAX_ACCEL * dt;
	if (deltaMag > maxDelta && deltaMag > 0.0001f) {
		float scale = maxDelta / deltaMag;
		dvx *= scale;
		dvy *= scale;
	}
	// Smoothed mission velocity command
	positionMissionVxCommand += dvx;
	positionMissionVyCommand += dvy;
	//------------------------------------------------------------------
	// Feed into position / velocity controller
	//------------------------------------------------------------------
	setExpectedPositionVelocity(dt, positionMissionVxCommand, positionMissionVyCommand);
}

__ATTR_ITCM_TEXT
void updateWPCompletionStatus(float dt) {
	float dx = fcStatusData.positionXRefMission - positionCordinateData.xPosition;
	float dy = fcStatusData.positionYRefMission - positionCordinateData.yPosition;
	uint8_t lowGroundSpeed = (getGroundSpeed() <= POSITION_MISSION_WP_COMPLETE_MAX_GROUND_SPEED);
	float distance = fastSqrtf(dx * dx + dy * dy);
	if (distance <= POSITION_MISSION_WP_COMPLETE_RADIUS) {
		if (positionMissionWPCompleteDt < POSITION_MISSION_WP_COMPLETE_PERIOD) {
			positionMissionWPCompleteDt += dt;
		}
	} else {
		positionMissionWPCompleteDt = 0.0f;
	}
	uint8_t timeoutReached = (positionMissionWPCompleteDt >= POSITION_MISSION_WP_COMPLETE_PERIOD);
	if (distance <= POSITION_MISSION_WP_COMPLETE_RADIUS && (lowGroundSpeed || timeoutReached)) {
		positionMissionWPComplete = 1;
	} else {
		positionMissionWPComplete = 0;
	}
}

