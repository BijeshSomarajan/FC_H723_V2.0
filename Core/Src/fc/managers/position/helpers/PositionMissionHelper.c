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
			logString("Set Home Position for RTH\n");
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
				logString("Starting Nav Mission\n");
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
		uint8_t hasMoreWP = loadWayPoints();
		if (hasMoreWP) {
			resetNavWPStates();
		} else {
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
	// Desired velocity
	float desiredVx = 0.0f;
	float desiredVy = 0.0f;
	/*
	 * Navigation phase
	 *
	 * Outside the capture radius, generate a velocity vector
	 * pointing toward the waypoint.
	 */
	if (distance > POSITION_MISSION_WP_CAPTURE_RADIUS) {
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
	 * Capture phase
	 *
	 * Inside the capture radius desired velocity remains zero.
	 * The position controller should then capture and hold the
	 * waypoint against disturbances such as wind.
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
void updateMissionVelocityCommandOld(float dt) {
// Position error to home
	float dx = fcStatusData.positionXRefMission - positionCordinateData.xPosition;
	float dy = fcStatusData.positionYRefMission - positionCordinateData.yPosition;
// Distance to home
	float distance = fastSqrtf(dx * dx + dy * dy);
// Normalize direction vector
	float dirX = 0.0f;
	float dirY = 0.0f;
	if (distance > 0.01f) {
		float invDist = 1.0f / distance;
		dirX = dx * invDist;
		dirY = dy * invDist;
	}
// Base cruise speed
	float targetSpeed = fminf(positionCruiseSpeed, fastSqrtf(2.0f * POSITION_MISSION_BRAKE_DECEL * distance));
// Slow down near home
	if (distance < POSITION_MISSION_WP_NEAR_RADIUS) {
		float scale = distance / POSITION_MISSION_WP_NEAR_RADIUS;
		scale = constrainToRangeF(scale, 0.0f, 1.0f);
		targetSpeed *= scale;
	}
// Desired velocity command
	float desiredVx = dirX * targetSpeed;
	float desiredVy = dirY * targetSpeed;
//------------------------------------------------------------------
// Acceleration limiting (vector magnitude based)
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
// Smoothed velocity target
	positionMissionVxCommand += dvx;
	positionMissionVyCommand += dvy;
//------------------------------------------------------------------
// Feed velocity target into velocity controller
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

