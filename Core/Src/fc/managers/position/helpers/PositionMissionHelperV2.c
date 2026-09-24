#include "PositionMissionHelper.h"

#if POSITION_MISSION_IMPL_VERSION == 2

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

#define POSITION_MISSION_FLYBY_ENABLED 1
#define POSITION_MISSION_WP_LATERAL_ACCEL_MAX (POSITION_MISSION_MAX_ACCEL * 0.7f)
#define POSITION_MISSION_WP_FLYBY_MIN_DIST (POSITION_MISSION_WP_COMPLETE_RADIUS)

// ---------------------------------------------------------------
// Arc guidance radial correction.
//
// Pure tangential guidance (direction only, no radius feedback) is
// unstable against any lag between commanded and actual velocity:
// nothing opposes outward drift, so radius error grows without
// bound. This adds a proportional radial term that pulls the
// aircraft back toward the intended circle.
//
// kR = RADIAL_GAIN * (cornerSpeed / radius)  -- natural orbit rate
// Radial correction magnitude is capped at MAX_RADIAL_FRAC of
// cornerSpeed so a large entry error cannot demand a large radial
// velocity; convergence just takes a bit longer instead.
// ---------------------------------------------------------------
#define POSITION_MISSION_WP_ARC_RADIAL_GAIN 1.0f
#define POSITION_MISSION_WP_ARC_MAX_RADIAL_FRAC 0.4f

float positionMissionVxCommand;
float positionMissionVyCommand;

float positionMissionWPCompleteDt = 0;

uint8_t positionMissionWasRTHModeActive = 0;
uint8_t positionMissionWasMissionModeActive = 0;

int16_t positionMissionWPIndx = 0;

uint8_t positionMissionWPComplete = 0;

int16_t positionMissionWPCount = 0;

float positionCruiseSpeed = POSITION_MISSION_CRUISE_SPEED_MIN;
float positionRTHSpeed = POSITION_MISSION_CRUISE_SPEED_MIN;

// ---------------------------------------------------------------
// Fly-by leg geometry
//
// The WP remains active during the complete fly-by transition.
//
// ux,uy = incoming leg direction
// vx,vy = outgoing leg direction
//
// flybyDistance = distance before/after WP used for the transition
// dMax          = maximum allowed fly-by distance
//
// During the transition:
//
//     incoming direction  --->  outgoing direction
//
// ---------------------------------------------------------------
typedef struct {
	float ux;
	float uy;

	float vx;
	float vy;

	float tanHalf;

	float dMax;

	float flybyDistance;
	float cornerSpeed;

	float radius;
	float tangentDistance;

	float centerX;
	float centerY;

	float entryX;
	float entryY;

	float exitX;
	float exitY;

	uint8_t clockwise;
	uint8_t mustStop;
} FlybyLeg;

FlybyLeg flybyLeg = { .mustStop = 1 };

uint8_t loadWayPoints(void);
void updateWPCompletionStatus(float dt);
void updateMissionVelocityCommand(float dt);

// ---------------------------------------------------------------
// Init
// ---------------------------------------------------------------
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

float getMaxCruiseSpeed() {
	return positionCruiseSpeed;
}

// ---------------------------------------------------------------
// Reset WP states
// ---------------------------------------------------------------
void resetNavWPStates() {
	positionMissionWPComplete = 0;
	positionMissionWPCompleteDt = 0;
}

// ---------------------------------------------------------------
// Reset RTH states
// ---------------------------------------------------------------
void resetNavRTHStates() {
	positionMissionWasRTHModeActive = 0;
}

// ---------------------------------------------------------------
// Reset mission mode states
// ---------------------------------------------------------------
void resetNavMissionModeStates() {
	positionMissionWasMissionModeActive = 0;
}

// ---------------------------------------------------------------
// Reset complete mission state
// ---------------------------------------------------------------
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
	fcStatusData.positionVelMission = positionCruiseSpeed;

	flybyLeg = (FlybyLeg ) { .mustStop = 1 };

	resetNavWPStates();
}

// ---------------------------------------------------------------
// Update position reference
// ---------------------------------------------------------------
__ATTR_ITCM_TEXT
void updatePositionReferenceToWPRef() {
	fcStatusData.positionXRef = fcStatusData.positionXRefMission;
	fcStatusData.positionYRef = fcStatusData.positionYRefMission;
}

static void flybyLegInit(FlybyLeg *L, float prevX, float prevY, float wpX, float wpY, uint8_t hasNext, float nextX, float nextY, float wpVel, uint8_t forceStop) {
	*L = (FlybyLeg ) { .mustStop = 1 };
	if (!hasNext || forceStop) {
		return;
	}
	// -----------------------------------------------------------
	// Incoming leg
	// -----------------------------------------------------------
	float ux = wpX - prevX;
	float uy = wpY - prevY;
	// -----------------------------------------------------------
	// Outgoing leg
	// -----------------------------------------------------------
	float vx = nextX - wpX;
	float vy = nextY - wpY;
	float len = fastSqrtf(ux * ux + uy * uy);
	float nlen = fastSqrtf(vx * vx + vy * vy);
	if (len < 0.01f || nlen < 0.01f) {
		return;
	}
	ux /= len;
	uy /= len;
	vx /= nlen;
	vy /= nlen;
	// -----------------------------------------------------------
	// Turn angle
	// -----------------------------------------------------------
	float cosT = ux * vx + uy * vy;
	cosT = fminf(fmaxf(cosT, -1.0f), 1.0f);
	// -----------------------------------------------------------
	// Near reversal
	// -----------------------------------------------------------
	if (cosT < -0.95f) {
		return;
	}
	// -----------------------------------------------------------
	// tan(turnAngle / 2)
	// -----------------------------------------------------------
	float denominator = fmaxf(1.0f + cosT, 0.05f);
	float tanHalf = fastSqrtf((1.0f - cosT) / denominator);
	if (tanHalf < 0.001f) {
		// Essentially a straight line: no fillet needed, no corner slow-down.
		float dMaxStraight = 0.5f * fminf(len, nlen);
		float flybyDist = fminf(POSITION_MISSION_WP_FLYBY_MIN_DIST, dMaxStraight);

		L->ux = ux;
		L->uy = uy;
		L->vx = ux;   // straight through: outgoing == incoming
		L->vy = uy;

		L->dMax = dMaxStraight;
		L->flybyDistance = flybyDist;
		L->tangentDistance = flybyDist;

		L->exitX = wpX + ux * flybyDist;
		L->exitY = wpY + uy * flybyDist;

		L->cornerSpeed = wpVel;
		L->mustStop = 0;
		return;
	}
	// -----------------------------------------------------------
	// Maximum tangent distance.
	//
	// Never consume more than half of either adjacent leg.
	// ----------------------------------------------------------
	float dMax = 0.5f * fminf(len, nlen);
	// -----------------------------------------------------------
	// Determine corner speed.
	//
	// For a circular path:
	//
	//     aLat = V^2 / R
	//
	// and:
	//
	//     tangentDistance = R * tan(theta / 2)
	//
	// -----------------------------------------------------------

	float cornerSpeed = wpVel;
	float maxRadius = dMax / tanHalf;
	float maxCornerSpeed = fastSqrtf(POSITION_MISSION_WP_LATERAL_ACCEL_MAX * maxRadius);
	cornerSpeed = fminf(cornerSpeed, maxCornerSpeed);
	cornerSpeed = fminf(fmaxf(cornerSpeed, POSITION_MISSION_CRUISE_SPEED_MIN), wpVel);
	// -----------------------------------------------------------
	// Circular radius
	// -----------------------------------------------------------
	float radius = (cornerSpeed * cornerSpeed) / POSITION_MISSION_WP_LATERAL_ACCEL_MAX;
	// Make absolutely sure the tangent points fit.
	float tangentDistance = radius * tanHalf;
	if (tangentDistance > dMax) {
		tangentDistance = dMax;
		radius = tangentDistance / tanHalf;
		cornerSpeed = fastSqrtf(POSITION_MISSION_WP_LATERAL_ACCEL_MAX * radius);
	}
	// -----------------------------------------------------------
	// Turn direction.
	//
	// cross > 0  = CCW
	// cross < 0  = CW
	// -----------------------------------------------------------

	float cross = ux * vy - uy * vx;
	uint8_t clockwise = (cross < 0.0f);
	// Normal pointing toward the inside of the turn.
	float normalX;
	float normalY;
	if (clockwise) {
		normalX = uy;
		normalY = -ux;
	} else {
		normalX = -uy;
		normalY = ux;
	}
	// -----------------------------------------------------------
	// Entry tangent point
	// -----------------------------------------------------------
	float entryX = wpX - ux * tangentDistance;
	float entryY = wpY - uy * tangentDistance;
	// -----------------------------------------------------------
	// Exit tangent point
	// -----------------------------------------------------------
	float exitX = wpX + vx * tangentDistance;
	float exitY = wpY + vy * tangentDistance;
	// -----------------------------------------------------------
	// Circle centre
	// -----------------------------------------------------------
	float centerX = entryX + normalX * radius;
	float centerY = entryY + normalY * radius;
	// -----------------------------------------------------------
	// Store geometry
	// -----------------------------------------------------------
	L->ux = ux;
	L->uy = uy;
	L->vx = vx;
	L->vy = vy;

	L->tanHalf = tanHalf;
	L->dMax = dMax;

	L->flybyDistance = tangentDistance;
	L->cornerSpeed = cornerSpeed;

	L->radius = radius;
	L->tangentDistance = tangentDistance;

	L->centerX = centerX;
	L->centerY = centerY;

	L->entryX = entryX;
	L->entryY = entryY;

	L->exitX = exitX;
	L->exitY = exitY;

	L->clockwise = clockwise;
	L->mustStop = 0;
}

__ATTR_ITCM_TEXT
static float getFlybyDistance(const FlybyLeg *L) {
	return L->flybyDistance;
}

// ---------------------------------------------------------------
// Closed-loop circular arc guidance.
//
// Commands tangential velocity (for progress around the fillet)
// PLUS a proportional radial term (to correct radius error). The
// radial term is what makes this stable against real velocity-
// tracking lag; a pure-tangent law has no such term and its
// radius error grows without bound under lag (see field test /
// desktop check).
// ---------------------------------------------------------------
__ATTR_ITCM_TEXT
static void getArcGuidanceVelocity(const FlybyLeg *L, float px, float py, float *outVx, float *outVy) {
	float nx = px - L->centerX;
	float ny = py - L->centerY;
	float r = fastSqrtf(nx * nx + ny * ny);

	if (r < 0.01f || L->radius < 0.01f) {
		// Degenerate: no well-defined radial direction, fall back to incoming leg direction
		*outVx = L->ux * L->cornerSpeed;
		*outVy = L->uy * L->cornerSpeed;
		return;
	}

	nx /= r;
	ny /= r;

	float tanX, tanY;
	if (L->clockwise) {
		tanX = ny;
		tanY = -nx;
	} else {
		tanX = -ny;
		tanY = nx;
	}

	float e = r - L->radius;   // + = outside intended circle
	float kR = POSITION_MISSION_WP_ARC_RADIAL_GAIN * (L->cornerSpeed / L->radius);
	float corrMag = fminf(fabsf(kR * e), POSITION_MISSION_WP_ARC_MAX_RADIAL_FRAC * L->cornerSpeed);
	float corrSign = (e > 0.0f) ? -1.0f : 1.0f;   // pull inward if outside, outward if inside

	float vx = tanX * L->cornerSpeed + corrSign * corrMag * nx;
	float vy = tanY * L->cornerSpeed + corrSign * corrMag * ny;

	// Bound the combined magnitude; direction is unaffected by uniform scaling.
	float mag = fastSqrtf(vx * vx + vy * vy);
	float capSpeed = fminf(fcStatusData.positionVelMission, L->cornerSpeed * 1.3f);
	if (mag > capSpeed && mag > 0.0001f) {
		float s = capSpeed / mag;
		vx *= s;
		vy *= s;
	}

	*outVx = vx;
	*outVy = vy;
}

__ATTR_ITCM_TEXT
static uint8_t flybyReached(const FlybyLeg *L, float px, float py) {
	float dx = px - L->exitX;
	float dy = py - L->exitY;
	float passedExit = (dx * L->vx + dy * L->vy) >= 0.0f;
	return passedExit;
}

// ---------------------------------------------------------------
// Load waypoint
// ---------------------------------------------------------------
__ATTR_ITCM_TEXT
uint8_t loadWayPoints() {
	if (positionMissionWPIndx >= 0 && positionMissionWPIndx < positionMissionWPCount) {
		GroundStationSensorWPData *groundStationSensorWPData = getGroundStationSensorWPData(positionMissionWPIndx);
		if (groundStationSensorWPData != NULL) {
			// ---------------------------------------------------
			// Previous leg anchor
			//
			// For WP0 this is the position captured when the
			// mission was started.
			//
			// Afterwards this is the previous WP reference.
			// ---------------------------------------------------
			float prevX = fcStatusData.positionXRefMission;
			float prevY = fcStatusData.positionYRefMission;

			float posX;
			float posY;
			convertGNSSToXYCordinates(groundStationSensorWPData->latitude, groundStationSensorWPData->longitude, fcStatusData.positionLatHome, fcStatusData.positionLongHome, &posX, &posY);
			// ---------------------------------------------------
			// Per-WP velocity
			// ---------------------------------------------------
			float wpVel = constrainToRangeF(fabs(groundStationSensorWPData->velocity), POSITION_MISSION_CRUISE_SPEED_MIN, positionCruiseSpeed);
			// ---------------------------------------------------
			// Fly-by force-stop
			// ---------------------------------------------------
			uint8_t forceStop = (POSITION_MISSION_FLYBY_ENABLED == 0);
			// ---------------------------------------------------
			// Next WP
			// ---------------------------------------------------
			uint8_t hasNext = 0;
			float nextX = 0.0f;
			float nextY = 0.0f;
			if (positionMissionWPIndx + 1 < positionMissionWPCount) {
				GroundStationSensorWPData *nextWPData = getGroundStationSensorWPData(positionMissionWPIndx + 1);
				if (nextWPData != NULL) {
					convertGNSSToXYCordinates(nextWPData->latitude, nextWPData->longitude, fcStatusData.positionLatHome, fcStatusData.positionLongHome, &nextX, &nextY);
					hasNext = 1;
				}
			}
			// ---------------------------------------------------
			// Build fly-by geometry
			// ---------------------------------------------------
			flybyLegInit(&flybyLeg, prevX, prevY, posX, posY, hasNext, nextX, nextY, wpVel, forceStop);
			// ---------------------------------------------------
			// Set current WP
			// ---------------------------------------------------
			fcStatusData.positionXRefMission = posX;
			fcStatusData.positionYRefMission = posY;
			fcStatusData.positionVelMission = wpVel;
			return 1;
		}
	}
	// No WP = stop behavior
	flybyLeg = (FlybyLeg ) { .mustStop = 1 };
	return 0;
}

// ---------------------------------------------------------------
// Mission callback
// ---------------------------------------------------------------
void groundStationMissionCallBack(uint8_t action) {
	if (action == NAV_ACTION_START_MISSION) {
		resetNavMissionStates();
		if (!isNavRTHModeActive()) {
			fcStatusData.isNavMissionModeActive = 1;
			positionMissionWasMissionModeActive = 0;
		} else {
			positionMissionWasRTHModeActive = 0;
		}
		positionMissionWPCount = getGroundStationSensorWPDataCount();
		loadWayPoints();
	} else if (action == NAV_ACTION_ABORT_MISSION) {
		resetNavMissionStates();
		fcStatusData.isNavMissionModeActive = 0;
	}
}

// ---------------------------------------------------------------
// Mission handler
// ---------------------------------------------------------------
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
	// -----------------------------------------------------------
	// First calculate velocity command
	// -----------------------------------------------------------
	updateMissionVelocityCommand(dt);
	// -----------------------------------------------------------
	// Then determine WP completion
	// -----------------------------------------------------------
	updateWPCompletionStatus(dt);
	updatePositionReference();
	// -----------------------------------------------------------
	// Advance WP only after the fly-by transition has finished
	// -----------------------------------------------------------
	if (positionMissionWPComplete) {
		positionMissionWPIndx++;
		resetNavWPStates();
		uint8_t hasMoreWP = loadWayPoints();
		if (!hasMoreWP) {
			fcStatusData.isNavMissionComplete = 1;
			// Last mission WP becomes the new anchor
			updatePositionReferenceToWPRef();
			if (fcStatusData.isFailSafeModeActive) {
				fcStatusData.isLandingModeActive = 1;
				fcStatusData.isFailSafeModeActive = 0;
			}
		}
	}
}

// ---------------------------------------------------------------
// Mission velocity command
// ---------------------------------------------------------------
__ATTR_ITCM_TEXT
void updateMissionVelocityCommand(float dt) {
	// -----------------------------------------------------------
	// Position error to current WP
	// -----------------------------------------------------------
	float dx = fcStatusData.positionXRefMission - positionCordinateData.xPosition;
	float dy = fcStatusData.positionYRefMission - positionCordinateData.yPosition;
	float distance = fastSqrtf(dx * dx + dy * dy);
	float desiredVx = 0.0f;
	float desiredVy = 0.0f;
	// -----------------------------------------------------------
	// Stop WP
	// -----------------------------------------------------------
	if (flybyLeg.mustStop) {
		float navGate = POSITION_MISSION_WP_CAPTURE_RADIUS;
		if (distance > navGate) {
			float invDist = 1.0f / distance;
			float dirX = dx * invDist;
			float dirY = dy * invDist;
			float remainingDistance = distance - POSITION_MISSION_WP_CAPTURE_RADIUS;
			float brakingSpeed = fastSqrtf(2.0f * POSITION_MISSION_BRAKE_DECEL * fmaxf(remainingDistance, 0.0f));
			float targetSpeed = fminf(fcStatusData.positionVelMission, brakingSpeed);
			desiredVx = dirX * targetSpeed;
			desiredVy = dirY * targetSpeed;
		}

	} else {
		// -------------------------------------------------------
		// Fly-by WP
		// -------------------------------------------------------
		float remaining = dx * flybyLeg.ux + dy * flybyLeg.uy;
		float d = getFlybyDistance(&flybyLeg);
		// -------------------------------------------------------
		// Before circular arc
		// -------------------------------------------------------
		if (remaining > d) {
			if (distance > 0.05f) {
				float invDist = 1.0f / distance;
				float dirX = dx * invDist;
				float dirY = dy * invDist;
				// Brake toward corner speed.
				float brakeDistance = fmaxf(distance - d, 0.0f);
				float brakingSpeed = fastSqrtf(flybyLeg.cornerSpeed * flybyLeg.cornerSpeed + 2.0f * POSITION_MISSION_BRAKE_DECEL * brakeDistance);
				float targetSpeed = fminf(fcStatusData.positionVelMission, brakingSpeed);
				desiredVx = dirX * targetSpeed;
				desiredVy = dirY * targetSpeed;
			}

		} else if (flybyLeg.tanHalf < 0.001f) {
			// Straight pass-through: negligible turn angle, no arc needed
			desiredVx = flybyLeg.vx * flybyLeg.cornerSpeed;
			desiredVy = flybyLeg.vy * flybyLeg.cornerSpeed;

		} else {
			// ---------------------------------------------------
			// Closed-loop circular arc guidance
			// ---------------------------------------------------
			getArcGuidanceVelocity(&flybyLeg, positionCordinateData.xPosition, positionCordinateData.yPosition, &desiredVx, &desiredVy);
		}
	}
	// -----------------------------------------------------------
	// Acceleration limiting
	// -----------------------------------------------------------
	float dvx = desiredVx - positionMissionVxCommand;
	float dvy = desiredVy - positionMissionVyCommand;
	float deltaMag = fastSqrtf(dvx * dvx + dvy * dvy);
	float maxDelta = POSITION_MISSION_MAX_ACCEL * dt;
	if (deltaMag > maxDelta && deltaMag > 0.0001f) {
		float scale = maxDelta / deltaMag;
		dvx *= scale;
		dvy *= scale;
	}
	// -----------------------------------------------------------
	// Smoothed mission velocity command
	// -----------------------------------------------------------
	positionMissionVxCommand += dvx;
	positionMissionVyCommand += dvy;
	// -----------------------------------------------------------
	// Feed into position / velocity controller
	// -----------------------------------------------------------
	setExpectedPositionVelocity(dt, positionMissionVxCommand, positionMissionVyCommand);
}

// ---------------------------------------------------------------
// WP completion
// ---------------------------------------------------------------
__ATTR_ITCM_TEXT
void updateWPCompletionStatus(float dt) {
	float dx = fcStatusData.positionXRefMission - positionCordinateData.xPosition;
	float dy = fcStatusData.positionYRefMission - positionCordinateData.yPosition;
	float distSq = dx * dx + dy * dy;
	uint8_t insideRadius = (distSq <= POSITION_MISSION_WP_COMPLETE_RADIUS_SQ);
	// -----------------------------------------------------------
	// Fly-by WP
	//
	// IMPORTANT:
	//
	// Do NOT complete simply because the drone enters the WP
	// radius. The WP remains active while the velocity direction
	// transitions through the corner.
	// -----------------------------------------------------------
	if (!flybyLeg.mustStop) {
		// Worst case: bounded time inside the fly-by zone, regardless of
		// whether along-track progress is being made.
		uint8_t withinFlybyZone = (distSq <= (flybyLeg.dMax * flybyLeg.dMax));
		if (withinFlybyZone) {
			if (positionMissionWPCompleteDt < POSITION_MISSION_WP_DWELL_TIMEOUT) {
				positionMissionWPCompleteDt += dt;
			}
		} else {
			positionMissionWPCompleteDt = 0.0f;
		}
		uint8_t dwellTimeout = (positionMissionWPCompleteDt >= POSITION_MISSION_WP_DWELL_TIMEOUT);
		positionMissionWPComplete = (flybyReached(&flybyLeg, positionCordinateData.xPosition, positionCordinateData.yPosition) || dwellTimeout) ? 1 : 0;

		return;
	}

	// -----------------------------------------------------------
	// Stop WP
	// -----------------------------------------------------------
	uint8_t lowGroundSpeed = (getGroundSpeed() <= POSITION_MISSION_WP_COMPLETE_MAX_GROUND_SPEED);
	// -----------------------------------------------------------
	// Dwell timer
	//
	// Counts continuous time inside capture radius.
	// -----------------------------------------------------------
	if (insideRadius) {
		if (positionMissionWPCompleteDt < POSITION_MISSION_WP_DWELL_TIMEOUT) {
			positionMissionWPCompleteDt += dt;
		}
	} else {
		positionMissionWPCompleteDt = 0.0f;
	}
	// -----------------------------------------------------------
	// Completion
	// -----------------------------------------------------------
	uint8_t dwellTimeout = (positionMissionWPCompleteDt >= POSITION_MISSION_WP_DWELL_TIMEOUT);
	positionMissionWPComplete = (insideRadius && (lowGroundSpeed || dwellTimeout)) ? 1 : 0;
}

#endif
