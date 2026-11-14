#include "MissionNavigationHelper.h"

#include <math.h>
#include <sys/_stdint.h>

#include "../../control/position/PositionControl.h"
#include "../../memory/Memory.h"
#include "../../sensors/groundStation/GroundStation.h"
#include "../../status/FCStatus.h"
#include "../../util/MathUtil.h"
#include "../attitude/helpers/HeadingCommandHelper.h"
#include "../position/common/PositionCommon.h"
#include "../position/estimator/PositionEstimatorHelper.h"
#include "../position/helpers/PositionManagerHelper.h"
#include "MissionManager.h"

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
extern float missionRTHSpeed;
extern float missionWPCompleteDt;
extern float missionWPDistanceFiltered;
extern float missionVxCommand;
extern float missionVyCommand;
void resetWPStates(void);

// Where the drone holds while the nose turns at the start of a course-locked leg
static float navTurnHoldX = 0.0f;
static float navTurnHoldY = 0.0f;

/** Flies the leg toward the waypoint: velocity profile, acceleration limit, velocity command. */
__ATTR_ITCM_TEXT
static void navigateToWaypoint(float dt) {
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

/**
 * Course-lock leg: while the nose is still turning to the bearing the drone holds position
 * at the point where the turn started (no drift in wind), then the leg is flown.
 */
__ATTR_ITCM_TEXT
void updateWpNavCommand(float dt) {
	if (isHeadingCommandComplete()) {
		navigateToWaypoint(dt);
	} else {
		controlPositionCordinatesWithGains(dt, navTurnHoldX, navTurnHoldY, 1.0f);
		// Track the real velocity so the leg starts without a velocity step when the turn ends
		missionVxCommand = positionCordinateData.xVelocity;
		missionVyCommand = positionCordinateData.yVelocity;
	}
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

/**
 * Course lock: points the nose along the direction of flight (current position to the target).
 * Waypoints without the lock keep whatever heading the drone already has.
 */
__ATTR_ITCM_TEXT
static void loadMissionHeading(void) {
	cancelHeadingCommand();
	if (currentMissionWPData->courseLock != 0) {
		float dx = fcStatusData.positionXRefMission - positionCordinateData.xPosition;   // north
		float dy = fcStatusData.positionYRefMission - positionCordinateData.yPosition;   // east
		// A bearing is meaningless when the target is too close
		if ((dx * dx + dy * dy) >= MISSION_COURSE_LOCK_MIN_LEG_SQ) {
			float bearing = atan2Approx(dy, dx) * (180.0f / (float) M_PI);   // clockwise from north
			if (bearing < 0.0f) {
				bearing += 360.0f;
			}
			navTurnHoldX = positionCordinateData.xPosition;   // hold here while the nose turns
			navTurnHoldY = positionCordinateData.yPosition;
			setAbsoluteHeadingCommand(bearing);
		}
	}
}

/** Common part of every navigation target: reference, speed, distance filter seed, dwell timer. */
static void setNavTarget(float posX, float posY, float speed) {
	resetWPStates();
	fcStatusData.positionXRefMission = posX;
	fcStatusData.positionYRefMission = posY;
	fcStatusData.positionVelMission = constrainToRangeF(fabsf(speed), MISSION_MGR_SPEED_MIN, missionCruiseSpeed);
	float dx = posX - positionCordinateData.xPosition;
	float dy = posY - positionCordinateData.yPosition;
	missionWPDistanceFiltered = dx * dx + dy * dy;   // seed with the real distance
	missionWPCompleteDt = 0;
}

__ATTR_ITCM_TEXT
void loadNavCordinates() {
	float posX, posY;
	convertGNSSToXYCordinates(currentMissionWPData->latitude, currentMissionWPData->longitude, fcStatusData.positionLatHome, fcStatusData.positionLongHome, &posX, &posY);
	setNavTarget(posX, posY, currentMissionWPData->velocity);
}

/**
 * Soft RTH (mission operation): retargets the navigation to the home point.
 * The operation then flies the leg through doOpNavigation().
 */
__ATTR_ITCM_TEXT
void startRTHOpNavigation(void) {
	float posX, posY;
	convertGNSSToXYCordinates(fcStatusData.positionLatHome, fcStatusData.positionLongHome, fcStatusData.positionLatHome, fcStatusData.positionLongHome, &posX, &posY);
	setNavTarget(posX, posY, missionRTHSpeed);
	cancelHeadingCommand();   // no course lock on the way home: the nose keeps its heading
	missionVxCommand = positionCordinateData.xVelocity;
	missionVyCommand = positionCordinateData.yVelocity;
}

/**
 * Navigation owned by an operation (soft RTH): flies toward the current navigation target
 * and returns 1 once it is reached. The target is loaded by the operation's start (loadNavHome).
 */
__ATTR_ITCM_TEXT
uint8_t doRTHOpNavigation(float dt) {
	uint8_t navCompletionStatus = 0;
	if (fcStatusData.isFlying && fcStatusData.isPositionHomeSet) {
		navigateToWaypoint(dt);              // plain leg, no course-lock turn/hold
		navCompletionStatus = checkNavCompletion(dt);
	} else {
		navCompletionStatus = 1;
	}
	return navCompletionStatus;
}

__ATTR_ITCM_TEXT
uint8_t doWPNavigation(float dt) {
	uint8_t navCompletionStatus = 0;
	if (fcStatusData.isFlying && fcStatusData.isPositionHomeSet) {
		if (currentMissionWPData->status == MISSION_STATUS_WIP) {
			loadNavCordinates();
			loadMissionHeading();
			missionVxCommand = positionCordinateData.xVelocity;
			missionVyCommand = positionCordinateData.yVelocity;
			currentMissionWPData->status = MISSION_STATUS_WIP_NAV;
			MISSION_TRACE("[MM] WP%u NAV start\n", MM_WP());
		}
		updateWpNavCommand(dt);
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
