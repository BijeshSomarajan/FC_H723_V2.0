#include "MissionNavigationHelper.h"

#include <math.h>
#include <stdio.h>

#include "MissionManager.h"
#include "../../memory/Memory.h"
#include "../../logger/Logger.h"
#include "../../status/FCStatus.h"
#include "../../control/position/PositionControl.h"
#include "../../util/MathUtil.h"
#include "../position/common/PositionCommon.h"
#include "../position/estimator/PositionEstimatorHelper.h"
#include "../position/helpers/PositionManagerHelper.h"

#ifndef MISSION_MGR_TRACE_ENABLED
#define MISSION_MGR_TRACE_ENABLED 0
#endif

#if MISSION_MGR_TRACE_ENABLED == 1
#define MISSION_TRACE(...) do { char _mmBuf[48]; snprintf(_mmBuf, sizeof(_mmBuf), __VA_ARGS__); logString(_mmBuf); } while (0)
#else
#define MISSION_TRACE(...) ((void)0)
#endif

#define MM_WP() ((unsigned) currentMissionData->currentWaypointIndex)

extern MissionData *currentMissionData;
extern MissionWPData *currentMissionWPData;
extern float missionCruiseSpeed;
extern float missionWPCompleteDt;
extern float missionWPDistanceFiltered;
extern float missionVxCommand;
extern float missionVyCommand;
void resetWPStates(void);

__ATTR_ITCM_TEXT
void updateNavCommand(float dt) {
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
uint8_t checkNavCompletion(float dt) {
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
void loadNavCordinates() {
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
uint8_t doWPNavigation(float dt) {
	uint8_t navCompletionStatus = 0;
	if (fcStatusData.isFlying && fcStatusData.isPositionHomeSet) {
		if (currentMissionWPData->status == MISSION_STATUS_WIP) {
			loadNavCordinates();
			missionVxCommand = positionCordinateData.xVelocity;
			missionVyCommand = positionCordinateData.yVelocity;
			currentMissionWPData->status = MISSION_STATUS_WIP_NAV;
			MISSION_TRACE("[MM] WP%u NAV start\n", MM_WP());
		}
		updateNavCommand(dt);
		if (checkNavCompletion(dt)) {
			MISSION_TRACE("[MM] WP%u NAV reached\n", MM_WP());
			navCompletionStatus = 1;
		}
	} else {
		MISSION_TRACE("[MM] WP%u NAV skipped fly=%u home=%u\n", MM_WP(), (unsigned ) fcStatusData.isFlying, (unsigned ) fcStatusData.isPositionHomeSet);
		navCompletionStatus = 1;     // cannot navigate (not flying / no home): assert complete
	}
	return navCompletionStatus;
}

__ATTR_ITCM_TEXT
void missionPositionRelock(void) {
	missionVxCommand = positionCordinateData.xVelocity;
	missionVyCommand = positionCordinateData.yVelocity;
	MISSION_TRACE("[MM] RELOCK after stick override\n");
}
