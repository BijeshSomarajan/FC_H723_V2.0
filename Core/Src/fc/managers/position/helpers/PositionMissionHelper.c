#include "PositionMissionHelper.h"

#include <math.h>
#include <stddef.h>
#include <sys/_stdint.h>

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

uint8_t loadWayPoints(void);
void updateWPCompletionStatus(float dt);
void updateMissionVelocityCommand(float dt);
char missionDebugBuf[128];

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
			sprintf(missionDebugBuf, "Nxt WP>>I:%d, Lt:%.7lf, Ln:%.7lf\n", groundStationSensorWPData->waypointIndex, groundStationSensorWPData->latitude, groundStationSensorWPData->longitude);
			logString(missionDebugBuf);
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
		sprintf(missionDebugBuf, "Mission Start: %d WPs\n", positionMissionWPCount);
		logString(missionDebugBuf);
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
		sprintf(missionDebugBuf, "WP Complete: %d\n", positionMissionWPIndx + 1);
		logString(missionDebugBuf);
		positionMissionWPIndx++;
		uint8_t hasMoreWP = loadWayPoints();
		if (hasMoreWP) {
			resetNavWPStates();
		} else {
			sprintf(missionDebugBuf, "Mission Complete: %d WPs\n", positionMissionWPIndx + 1);
			logString(missionDebugBuf);
			fcStatusData.isNavMissionComplete = 1;
		}
	}
}

__ATTR_ITCM_TEXT
void updateMissionVelocityCommand(float dt) {
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
	float targetSpeed = fminf(POSITION_MISSION_CRUISE_SPEED, fastSqrtf(2.0f * POSITION_MISSION_BRAKE_DECEL * distance));
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

