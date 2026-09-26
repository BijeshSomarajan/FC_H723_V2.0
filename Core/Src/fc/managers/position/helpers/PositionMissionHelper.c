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
int16_t positionMissionWPIndx = 0;
uint8_t positionMissionWPComplete = 0;
int16_t positionMissionWPCount = 0;

float positionCruiseSpeed = POSITION_MISSION_CRUISE_SPEED_MIN;
float positionRTHSpeed = POSITION_MISSION_CRUISE_SPEED_MIN;

uint8_t loadWayPoints(void);
void updateWPCompletionStatus(float dt);
void updateMissionVelocityCommand(float dt);

void initPositionMissionHelper() {
	positionCruiseSpeed = fabs(get1KXScaledCalibrationValue(CALIB_PROP_POS_HOLD_CRUISE_SPEED_ADDR));
	if (positionCruiseSpeed > POSITION_MISSION_CRUISE_SPEED_MAX) {
		positionCruiseSpeed = POSITION_MISSION_CRUISE_SPEED_MIN;
	}

	positionRTHSpeed = fabs(get1KXScaledCalibrationValue(CALIB_PROP_POS_HOLD_RTH_SPEED_ADDR));
	if ( positionRTHSpeed > positionCruiseSpeed) {
		positionRTHSpeed = positionCruiseSpeed * 0.8f;
	}

	char buf[64];
	sprintf(buf, "[PositionMissionHelper] RS:%.3f,CS:%.3f\n", positionRTHSpeed, positionCruiseSpeed);
	logString(buf);

}

float getMaxCruiseSpeed() {
	return positionCruiseSpeed;
}

void resetNavWPStates() {
	positionMissionWPComplete = 0;
	positionMissionWPCompleteDt = 0;
	positionMissionWPDistanceFiltered = 0;
}

void resetNavRTHStates() {
	positionMissionWasRTHModeActive = 0;
}

void resetNavMissionModeStates() {
	positionMissionWasMissionModeActive = 0;
}

void resetNavMissionStates() {
	positionMissionVxCommand = 0;
	positionMissionVyCommand = 0;

	resetNavRTHStates();
	resetNavMissionModeStates();

	positionMissionWPIndx = 0;
	positionMissionWPCount = 0;

	fcStatusData.positionXRefMission = positionCordinateData.xPosition;
	fcStatusData.positionYRefMission = positionCordinateData.yPosition;
	fcStatusData.positionVelMission = positionCruiseSpeed;

	resetNavWPStates();
}

__ATTR_ITCM_TEXT
void updatePositionReferenceToWPRef() {
	fcStatusData.positionXRef = fcStatusData.positionXRefMission;
	fcStatusData.positionYRef = fcStatusData.positionYRefMission;
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
			//Limit the velocity to cruise speed set , min is set to 0.5 M/s
			fcStatusData.positionVelMission = constrainToRangeF(fabs(groundStationSensorWPData->velocity), POSITION_MISSION_CRUISE_SPEED_MIN, positionCruiseSpeed);
			return 1;
		}
	}
	return 0;
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
		loadWayPoints();
	} else if (action == NAV_ACTION_ABORT_MISSION) {
		resetNavMissionStates();
		fcStatusData.isNavMissionComplete = 0;
		fcStatusData.isNavMissionModeActive = 0;
	}
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
			//RTH will be at a percentage of cruise speed
			groundStationSensorWPData.velocity = positionRTHSpeed;
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
	updateWPCompletionStatus(dt);
	updatePositionReference();
	if (positionMissionWPComplete) {
		positionMissionWPIndx++;
		resetNavWPStates();
		uint8_t hasMoreWP = loadWayPoints();
		if (!hasMoreWP) {
			fcStatusData.isNavMissionComplete = 1;
			fcStatusData.isNavMissionModeActive = 0;
			updatePositionReferenceToWPRef(); // The last mission WP reference is taken as the new anchor
			if (fcStatusData.isFailSafeModeActive) {
				//Trigger Landing
				fcStatusData.isLandingModeActive = 1;
				fcStatusData.isFailSafeModeActive = 0;
			}
		}
	}
}

__ATTR_ITCM_TEXT
void updateMissionVelocityCommand(float dt) {
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
	// outside the capture radius.
	// ---------------------------------------------------------------
	if (distance > POSITION_MISSION_WP_CAPTURE_RADIUS) {
		float invDist = 1.0f / distance;
		float dirX = dx * invDist;
		float dirY = dy * invDist;
		float remainingDistance = distance - POSITION_MISSION_WP_CAPTURE_RADIUS;
		float brakingSpeed = fastSqrtf(2.0f * POSITION_MISSION_BRAKE_DECEL * remainingDistance);

		//float targetSpeed = fminf(positionCruiseSpeed, brakingSpeed);
		//Each Waypoint can have its velocity
		float targetSpeed = fminf(fcStatusData.positionVelMission, brakingSpeed);

		desiredVx = dirX * targetSpeed;
		desiredVy = dirY * targetSpeed;
	}
	// ---------------------------------------------------------------
	// Acceleration limiting
	// ---------------------------------------------------------------
	float dvx = desiredVx - positionMissionVxCommand;
	float dvy = desiredVy - positionMissionVyCommand;
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
	positionMissionVxCommand += dvx;
	positionMissionVyCommand += dvy;
	// ---------------------------------------------------------------
	// Feed into position / velocity controller
	// ---------------------------------------------------------------
	setExpectedPositionVelocity(dt, positionMissionVxCommand, positionMissionVyCommand);
}

__ATTR_ITCM_TEXT
void updateWPCompletionStatus(float dt) {
	float dx = fcStatusData.positionXRefMission - positionCordinateData.xPosition;
	float dy = fcStatusData.positionYRefMission - positionCordinateData.yPosition;
	float distSq = dx * dx + dy * dy;
	positionMissionWPDistanceFiltered +=  POSITION_MISSION_WP_DISTANCE_LPF_ALPHA * (distSq - positionMissionWPDistanceFiltered);

	// Squared compare: no sqrt needed
	uint8_t insideRadius = (positionMissionWPDistanceFiltered <= POSITION_MISSION_WP_COMPLETE_RADIUS_SQ);
	uint8_t lowGroundSpeed = (getGroundSpeed() <= POSITION_MISSION_WP_COMPLETE_MAX_GROUND_SPEED);

	// Dwell timer: counts continuous time inside the capture radius
	if (insideRadius) {
		if (positionMissionWPCompleteDt < POSITION_MISSION_WP_DWELL_TIMEOUT) {
			positionMissionWPCompleteDt += dt;
		}
	} else {
		// Decay rather than hard-reset: a brief single-cycle excursion
		// (noise) shouldn't erase several seconds of accumulated dwell.
		positionMissionWPCompleteDt = fmaxf(positionMissionWPCompleteDt - dt, 0.0f);
	}
	// Normal path: inside radius and settled
	// Worst case:  inside radius for DWELL_TIMEOUT regardless of speed
	uint8_t dwellTimeout = (positionMissionWPCompleteDt >= POSITION_MISSION_WP_DWELL_TIMEOUT);
	positionMissionWPComplete = (insideRadius && (lowGroundSpeed || dwellTimeout)) ? 1 : 0;
}


