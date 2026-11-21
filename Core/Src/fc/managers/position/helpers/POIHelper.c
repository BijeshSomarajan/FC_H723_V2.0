#include "POIHelper.h"

#include <math.h>
#include <sys/_stdint.h>

#include "../../../control/position/PositionControl.h"
#include "../../../logger/Logger.h"
#include "../../../sensors/attitude/AttitudeSensor.h"
#include "../../../sensors/rc/RCSensor.h"
#include "../../../status/FCStatus.h"
#include "../../../util/MathUtil.h"
#include "../../attitude/helpers/HeadingCommandHelper.h"
#include "../common/PositionCommon.h"
#include "../estimator/PositionEstimatorHelper.h"
#include "../PositionManager.h"
#include "PositionManagerHelper.h"

/* ADAPT: true while the pilot is moving the yaw stick (a yawCentered flag is assumed) */
#define POI_PILOT_YAW_ACTIVE()   (!rcData.yawCentered)

typedef enum {
	POI_PHASE_ARMED = 0,   // radio: switch raised, waiting to start
	POI_PHASE_ENTRY, POI_PHASE_ALIGN, POI_PHASE_ORBIT, POI_PHASE_SETTLE, POI_PHASE_RETURN,
	POI_PHASE_DONE         // finished, rejected or cancelled
} POI_PHASE;

static POI_PHASE poiPhase = POI_PHASE_ARMED;
static uint8_t poiMissionOwned = 0;      // 1 while a mission operation owns the sequence
static float poiCfgRadius = POSITION_POI_DEFAULT_RADIUS;
static uint8_t poiCfgCcw = POSITION_POI_DEFAULT_CCW;
static uint8_t poiCfgIterations = POSITION_POI_DEFAULT_ITERATIONS;

static float poiCenterX = 0.0f, poiCenterY = 0.0f;
static float poiEntryX = 0.0f, poiEntryY = 0.0f, poiEntryHeading = 0.0f;
static float poiReturnX = 0.0f, poiReturnY = 0.0f;
static uint8_t poiReturnToAnchor = 0;
static float poiTargetX = 0.0f, poiTargetY = 0.0f;       // current leg target
static float poiHoldX = 0.0f, poiHoldY = 0.0f;           // held during ALIGN and SETTLE
static float poiYawHoldX = 0.0f, poiYawHoldY = 0.0f;
static float poiRadius = 0.0f, poiOrbitSpeed = 0.0f;
static float poiDirection = 1.0f;                        // +1 clockwise, -1 counter-clockwise (seen from above)
static float poiTargetAngle = 0.0f;                      // rad, iterations * 2*pi
static float poiPrevRx = 0.0f, poiPrevRy = 0.0f, poiSwept = 0.0f;
static float poiPhaseTime = 0.0f, poiStableTime = 0.0f, poiDwell = 0.0f;
static float poiVxCmd = 0.0f, poiVyCmd = 0.0f;           // own velocity command, independent of the mission
static uint8_t poiPrevValid = 0, poiResumePending = 0, poiResumeOrbit = 0, poiYawHolding = 0;

/* ---------------------------------------------------------------- helpers */

static uint8_t poiIsRunning(void) {
	return (poiPhase >= POI_PHASE_ENTRY && poiPhase <= POI_PHASE_RETURN) ? 1 : 0;
}

/** The state when nothing is running: re-armed if the radio switch is low, latched off while it is still high. */
static POI_PHASE poiRestingPhase(void) {
	return fcStatusData.isPoIModeActive ? POI_PHASE_DONE : POI_PHASE_ARMED;
}

static uint8_t poiReject(uint8_t code) {
	logPrintf("[POI] rejected:%u\n", (unsigned) code);
	return 0;
}

/** Stops a running sequence with a controlled stop: the manager brakes, settles and re-locks. */
static void poiCancel(void) {
	if (poiIsRunning()) {
		cancelHeadingCommand();
		fcStatusData.postionHoldState = POS_HOLD_STATE_IDLE;
	}
	poiPhase = POI_PHASE_DONE;
}

static void poiReleaseMission(void) {
	poiMissionOwned = 0;
	poiResumePending = 0;
	poiResumeOrbit = 0;
	poiYawHolding = 0;
	poiPhase = poiRestingPhase();
}

static float poiBearingToCenter(void) {
	float px = positionCordinateData.xPosition - poiCenterX;
	float py = positionCordinateData.yPosition - poiCenterY;
	if ((px * px + py * py) < (POSITION_POI_MIN_DIST * POSITION_POI_MIN_DIST)) {
		return poiEntryHeading;
	}
	return calculateBearing(px, py);
}

static void poiAimNose(void) {
	setAbsoluteHeadingCommand(poiBearingToCenter());   // every cycle on purpose: completion is judged numerically
}

static void poiTrackVelocity(void) {
	poiVxCmd = positionCordinateData.xVelocity;
	poiVyCmd = positionCordinateData.yVelocity;
}

static void poiHoldReference(float dt) {
	controlPositionCordinatesWithGains(dt, fcStatusData.positionXRef, fcStatusData.positionYRef, 1.0f);
}

static void poiSetLegTarget(float x, float y) {
	poiTargetX = x;
	poiTargetY = y;
	poiDwell = 0.0f;
	poiTrackVelocity();
}

/** Acceleration limit, feedforward and velocity setpoint. decelOnlyFF: 1 = feedforward only while slowing (legs), 0 = always (orbit). */
__ATTR_ITCM_TEXT
static void poiCommandVelocity(float desiredVx, float desiredVy, float dt, uint8_t decelOnlyFF) {
	float dvx = desiredVx - poiVxCmd;
	float dvy = desiredVy - poiVyCmd;
	float deltaMag = fastSqrtf(dvx * dvx + dvy * dvy);
	float maxDelta = POSITION_POI_MAX_ACCEL * dt;
	if (deltaMag > maxDelta && deltaMag > 0.0001f) {
		float scale = maxDelta / deltaMag;
		dvx *= scale;
		dvy *= scale;
	}
	uint8_t decelerating = ((dvx * poiVxCmd + dvy * poiVyCmd) < 0.0f);   // before the command is updated
	poiVxCmd += dvx;
	poiVyCmd += dvy;

	uint8_t useFF = decelOnlyFF ? decelerating : 1;
	float invDt = (dt > 1e-4f) ? (1.0f / dt) : 0.0f;
	float ax = useFF ? (dvx * invDt) : 0.0f;
	float ay = useFF ? (dvy * invDt) : 0.0f;
	float accMagSq = ax * ax + ay * ay;
	if (accMagSq > (POSITION_POI_FF_ACC_MAX * POSITION_POI_FF_ACC_MAX)) {
		float s = POSITION_POI_FF_ACC_MAX / fastSqrtf(accMagSq);
		ax *= s;
		ay *= s;
	}
	setPositionAccelFeedforward(ax, ay);
	setExpectedPositionVelocity(dt, poiVxCmd, poiVyCmd);
}

__ATTR_ITCM_TEXT
static void poiFlyToTarget(float dt) {
	float dx = poiTargetX - positionCordinateData.xPosition;
	float dy = poiTargetY - positionCordinateData.yPosition;
	float distance = fastSqrtf(dx * dx + dy * dy);
	float desiredVx = 0.0f, desiredVy = 0.0f;
	if (distance > POSITION_POI_STOP_RADIUS) {
		float braking = fastSqrtf(2.0f * POSITION_POI_BRAKE_DECEL * (distance - POSITION_POI_STOP_RADIUS));
		float speed = fminf(POSITION_POI_SPEED, braking);
		float inv = 1.0f / distance;
		desiredVx = dx * inv * speed;
		desiredVy = dy * inv * speed;
	}
	poiCommandVelocity(desiredVx, desiredVy, dt, 1);
}

__ATTR_ITCM_TEXT
static uint8_t poiLegReached(float dt) {
	float dx = poiTargetX - positionCordinateData.xPosition;
	float dy = poiTargetY - positionCordinateData.yPosition;
	uint8_t inside = ((dx * dx + dy * dy) <= (POSITION_POI_REACH_RADIUS * POSITION_POI_REACH_RADIUS));
	float speed = getGroundSpeed();
	if (inside) {
		poiDwell = fminf(poiDwell + dt, POSITION_POI_DWELL_TIME);
	} else {
		poiDwell = fmaxf(poiDwell - dt, 0.0f);
	}
	return (inside && (speed <= POSITION_POI_REACH_SPEED || (poiDwell >= POSITION_POI_DWELL_TIME && speed <= POSITION_POI_DWELL_SPEED))) ? 1 : 0;
}

/* ---------------------------------------------------------------- phase transitions */

static void poiFinish(float holdX, float holdY) {
	cancelHeadingCommand();
	if (!poiMissionOwned) {                   // the radio path holds its end point through the manager's reference
		fcStatusData.positionXRef = holdX;
		fcStatusData.positionYRef = holdY;
	}
	poiPhase = POI_PHASE_DONE;
}

static void beginPOIEntry(void) {
	poiSetLegTarget(poiEntryX, poiEntryY);
	poiPhase = POI_PHASE_ENTRY;
	poiPhaseTime = 0.0f;
}

static void beginPOIAlign(float holdX, float holdY, uint8_t resumeOrbit) {
	poiHoldX = holdX;
	poiHoldY = holdY;
	poiResumeOrbit = resumeOrbit;
	poiPhase = POI_PHASE_ALIGN;
	poiPhaseTime = 0.0f;
	poiStableTime = 0.0f;
	poiTrackVelocity();
}

static void beginPOIOrbit(uint8_t resume) {
	if (!resume) {
		poiSwept = 0.0f;
	}
	poiPrevValid = 0;                         // seeded on the first orbit cycle
	poiPhase = POI_PHASE_ORBIT;
	poiPhaseTime = 0.0f;
	poiTrackVelocity();
}

static void beginPOISettle(void) {
	poiHoldX = positionCordinateData.xPosition;   // hold where the orbit ended
	poiHoldY = positionCordinateData.yPosition;
	poiPhase = POI_PHASE_SETTLE;
	poiPhaseTime = 0.0f;
	poiStableTime = 0.0f;
}

static void beginPOIReturn(void) {
	if (poiReturnToAnchor) {
		poiSetLegTarget(poiReturnX, poiReturnY);
	} else {
		poiSetLegTarget(positionCordinateData.xPosition, positionCordinateData.yPosition);
	}
	poiPhase = POI_PHASE_RETURN;
	poiPhaseTime = 0.0f;
}

/** After a pilot interruption (mission only): re-establish the current phase from where the drone now is. */
static void poiApplyResume(void) {
	poiResumePending = 0;
	switch (poiPhase) {
	case POI_PHASE_ENTRY:
		beginPOIEntry();
		poiAimNose();                         // the pilot may have turned the nose: face the centre again
		break;
	case POI_PHASE_ALIGN:
		if (poiResumeOrbit) {
			beginPOIAlign(positionCordinateData.xPosition, positionCordinateData.yPosition, 1);
		} else {
			beginPOIEntry();                  // back to the entry point, aligning again from there
			poiAimNose();
		}
		break;
	case POI_PHASE_ORBIT:
		beginPOIAlign(positionCordinateData.xPosition, positionCordinateData.yPosition, 1);   // realign, then continue the orbit
		break;
	case POI_PHASE_SETTLE:
		poiHoldX = positionCordinateData.xPosition;
		poiHoldY = positionCordinateData.yPosition;
		poiStableTime = 0.0f;
		break;
	case POI_PHASE_RETURN:
		beginPOIReturn();
		break;
	default:
		break;
	}
}

/* ---------------------------------------------------------------- orbit */

/** Returns 1 when the last iteration is finished and the drone is nearly at rest. */
__ATTR_ITCM_TEXT
static uint8_t runPOIOrbit(float dt) {
	float px = positionCordinateData.xPosition - poiCenterX;
	float py = positionCordinateData.yPosition - poiCenterY;
	float d = fastSqrtf(px * px + py * py);
	if (d < POSITION_POI_MIN_DIST) {
		return 1;                             // at the centre: the orbit is meaningless
	}
	float rx = px / d;                        // unit vector, centre -> drone (north, east)
	float ry = py / d;

	if (poiPrevValid) {                       // swept angle, positive in the direction of travel
		poiSwept += poiDirection * (poiPrevRx * ry - poiPrevRy * rx);
	}
	poiPrevRx = rx;
	poiPrevRy = ry;
	poiPrevValid = 1;

	float arcRemaining = (poiTargetAngle - poiSwept) * poiRadius;
	float arcDone = fmaxf(poiSwept, 0.0f) * poiRadius;
	if (arcRemaining <= POSITION_POI_DONE_ARC && getGroundSpeed() <= POSITION_POI_DONE_SPEED) {
		return 1;
	}

	float bearing = calculateBearing(px, py);
	setAbsoluteHeadingCommand(bearing);
	float hdgErr = fabsf(wrap180(bearing - sensorAttitudeData.heading));
	float hdgScale = constrainToRangeF((POSITION_POI_HDG_ZERO_DEG - hdgErr) / (POSITION_POI_HDG_ZERO_DEG - POSITION_POI_HDG_FULL_DEG), 0.0f, 1.0f);

	float vr = constrainToRangeF(-POSITION_POI_RADIAL_KP * (d - poiRadius), -POSITION_POI_RADIAL_VMAX, POSITION_POI_RADIAL_VMAX);
	float vt = fastSqrtf(fmaxf(poiOrbitSpeed * poiOrbitSpeed - vr * vr, 0.0f));
	float endSpeed = fastSqrtf(2.0f * POSITION_POI_BRAKE_DECEL * fmaxf(arcRemaining - POSITION_POI_ARC_STOP, 0.0f));
	float startSpeed = fastSqrtf(2.0f * POSITION_POI_BRAKE_DECEL * (arcDone + POSITION_POI_ARC_STOP));
	vt = fminf(vt, fminf(endSpeed, startSpeed)) * hdgScale;

	// Tangent for increasing angle (clockwise seen from above) is (-ry, rx)
	float desiredVx = poiDirection * (-ry) * vt + rx * vr;
	float desiredVy = poiDirection * (rx) * vt + ry * vr;
	poiCommandVelocity(desiredVx, desiredVy, dt, 0);   // full feedforward: carries the centripetal acceleration
	return 0;
}

/* ---------------------------------------------------------------- start */

/** Validates and starts the sequence. anchored = 1: the drone starts at the centre (a mission waypoint), so it backs up
 *  one radius behind the nose and returns to the centre. anchored = 0: radio start, nearest point on the circle, returns to the start point. */
static uint8_t poiStart(float radius, float centerX, float centerY, uint8_t ccw, uint8_t iterations, uint8_t anchored) {
	if (!(radius >= POSITION_POI_MIN_RADIUS && radius <= POSITION_POI_MAX_RADIUS) || iterations < 1 || iterations > POSITION_POI_MAX_ITERATIONS || !isfinite(centerX) || !isfinite(centerY)) {
		return poiReject(1);
	}
	if ((float) iterations * 2.0f * (float) M_PI * radius > POSITION_POI_MAX_ARC) {
		return poiReject(2);
	}
	if (!anchored && fastSqrtf(centerX * centerX + centerY * centerY) + radius > POSITION_POI_MAX_HOME_DIST) {
		return poiReject(3);                  // radio range guard (a mission waypoint is the mission's own choice)
	}

	float px = positionCordinateData.xPosition - centerX;
	float py = positionCordinateData.yPosition - centerY;
	float d = fastSqrtf(px * px + py * py);

	poiEntryHeading = sensorAttitudeData.heading;
	float ux, uy;
	if (anchored || d < POSITION_POI_MIN_DIST) {
		float h = convertDegToRadF(poiEntryHeading);      // behind the nose, so the nose already faces the centre
		ux = -cosApproxF(h);
		uy = -sinApproxF(h);
	} else {
		ux = px / d;                          // nearest point on the circle
		uy = py / d;
	}
	float entryX = centerX + radius * ux;
	float entryY = centerY + radius * uy;
	float ex = entryX - positionCordinateData.xPosition;
	float ey = entryY - positionCordinateData.yPosition;
	float maxEntry = anchored ? (radius + POSITION_POI_ANCHOR_ENTRY_MARGIN) : POSITION_POI_MAX_ENTRY_DIST;
	if (fastSqrtf(ex * ex + ey * ey) > maxEntry) {
		return poiReject(4);
	}

	poiCenterX = centerX;
	poiCenterY = centerY;
	poiRadius = radius;
	poiDirection = ccw ? -1.0f : 1.0f;
	poiTargetAngle = (float) iterations * 2.0f * (float) M_PI;
	poiEntryX = entryX;
	poiEntryY = entryY;
	poiReturnToAnchor = (anchored || POSITION_POI_RETURN_TO_START == 1) ? 1 : 0;
	poiReturnX = anchored ? centerX : positionCordinateData.xPosition;
	poiReturnY = anchored ? centerY : positionCordinateData.yPosition;

	// Orbit speed: capped by lateral acceleration and by what the nose can follow (heading slew rate)
	float latLimited = fastSqrtf(POSITION_POI_LAT_ACC * radius);
	float yawLimited = POSITION_POI_YAW_FRACTION * convertDegToRadF(HEADING_COMMAND_ROTATION_RATE) * radius;
	poiOrbitSpeed = fminf(POSITION_POI_SPEED, fminf(latLimited, yawLimited));

	poiResumePending = 0;
	poiResumeOrbit = 0;
	poiYawHolding = 0;
	cancelHeadingCommand();                   // keep the nose while flying to the entry point
	beginPOIEntry();
	return 1;
}

/** Radio start: the centre is R metres ahead of the nose, so the drone is already on the circle facing the centre. */
static uint8_t poiStartAhead(float radius, uint8_t ccw, uint8_t iterations) {
	float h = convertDegToRadF(sensorAttitudeData.heading);
	return poiStart(radius, positionCordinateData.xPosition + radius * cosApproxF(h), positionCordinateData.yPosition + radius * sinApproxF(h), ccw, iterations, 0);
}

/* ---------------------------------------------------------------- sequence */

__ATTR_ITCM_TEXT
static void runPOISequence(float dt) {
	poiPhaseTime += dt;

	switch (poiPhase) {
	case POI_PHASE_ENTRY:
		poiFlyToTarget(dt);
		if (poiLegReached(dt) || poiPhaseTime > POSITION_POI_LEG_TIMEOUT) {
			beginPOIAlign(poiEntryX, poiEntryY, 0);
		}
		break;
	case POI_PHASE_ALIGN: {
		controlPositionCordinatesWithGains(dt, poiHoldX, poiHoldY, 1.0f);   // hold while the nose turns
		poiTrackVelocity();
		float bearing = poiBearingToCenter();
		setAbsoluteHeadingCommand(bearing);
		// Numeric completion: the heading reference has arrived and the real nose is within tolerance
		float refErr = fabsf(wrap180(bearing - fcStatusData.headingRef));
		float noseErr = fabsf(wrap180(bearing - sensorAttitudeData.heading));
		if (refErr <= POSITION_POI_ALIGN_REF_DEG && noseErr <= POSITION_POI_ALIGN_TOL_DEG) {
			poiStableTime += dt;
		} else {
			poiStableTime = 0.0f;
		}
		if (poiStableTime >= POSITION_POI_ALIGN_STABLE_TIME) {
			beginPOIOrbit(poiResumeOrbit);
		} else if (poiPhaseTime > POSITION_POI_ALIGN_TIMEOUT) {
			beginPOIReturn();                 // cannot aim at the centre: skip the orbit
		}
		break;
	}
	case POI_PHASE_ORBIT: {
		float orbitTimeout = 3.0f * (poiTargetAngle * poiRadius / poiOrbitSpeed) + 20.0f;
		if (runPOIOrbit(dt) || poiPhaseTime > orbitTimeout) {
			beginPOISettle();
		}
		break;
	}
	case POI_PHASE_SETTLE:
		controlPositionCordinatesWithGains(dt, poiHoldX, poiHoldY, 1.0f);
		poiTrackVelocity();
		poiAimNose();
		if (getGroundSpeed() <= POSITION_POI_SETTLE_SPEED) {
			poiStableTime += dt;
		} else {
			poiStableTime = 0.0f;
		}
		if (poiStableTime >= POSITION_POI_SETTLE_TIME || poiPhaseTime > POSITION_POI_SETTLE_TIMEOUT) {
			beginPOIReturn();
		}
		break;
	case POI_PHASE_RETURN:
		poiAimNose();                         // the nose stays on the centre while returning
		poiFlyToTarget(dt);
		if (poiLegReached(dt) || poiPhaseTime > POSITION_POI_LEG_TIMEOUT) {
			poiFinish(poiTargetX, poiTargetY);
		}
		break;

	default:
		break;
	}
}

/* ---------------------------------------------------------------- mission interface */

uint8_t decodePOIValue(float value, float *radius, uint8_t *iterations, uint8_t *ccw) {
	if (!(value >= 0.0f) || value > 16777215.0f) {   // also rejects NaN
		return 0;
	}
	uint32_t packed = (uint32_t) (value + 0.5f);
	if (packed & POI_VALUE_RESERVED_MASK) {
		return poiReject(10);
	}
	float r = (float) (packed & POI_VALUE_RADIUS_MASK) / 10.0f;
	uint32_t n = (packed >> POI_VALUE_REV_SHIFT) & POI_VALUE_REV_MASK;
	if (n == 0) {
		n = 1;
	}
	if (r < POSITION_POI_MIN_RADIUS || r > POSITION_POI_MAX_RADIUS || n > POSITION_POI_MAX_ITERATIONS) {
		return poiReject(11);
	}
	if ((float) n * 2.0f * (float) M_PI * r > POSITION_POI_MAX_ARC) {
		return poiReject(12);
	}
	*radius = r;
	*iterations = (uint8_t) n;
	*ccw = (packed & POI_VALUE_DIR_BIT) ? 1 : 0;
	return 1;
}

uint8_t startPOIOperation(double anchorLat, double anchorLon, float radius, uint8_t ccw, uint8_t iterations) {
	if (!isfinite(anchorLat) || !isfinite(anchorLon)) {
		return poiReject(5);
	}
	float cx, cy;
	convertGNSSToXYCordinates(anchorLat, anchorLon, fcStatusData.positionLatHome, fcStatusData.positionLongHome, &cx, &cy);
	if (!poiStart(radius, cx, cy, ccw, iterations, 1)) {
		return 0;
	}
	poiMissionOwned = 1;
	return 1;
}

/** One step of the mission operation. Returns 1 when the drone is back at the anchor, or when nothing is running. */
__ATTR_ITCM_TEXT
uint8_t isPOIOperationComplete(float dt) {
	if (!poiMissionOwned) {
		return 1;                             // cancelled or never started: the operation is over
	}
	if (!fcStatusData.isFlying || !isNavDataReliable()) {
		poiCancel();
		poiReleaseMission();
		return 1;                             // cannot navigate: end the operation, the mission carries on
	}
	// Pilot yaw: the pilot has priority. Hold position, leave the nose alone, freeze the timers, realign afterwards.
	if (POI_PILOT_YAW_ACTIVE()) {
		if (!poiYawHolding) {
			poiYawHolding = 1;
			poiYawHoldX = positionCordinateData.xPosition;
			poiYawHoldY = positionCordinateData.yPosition;
		}
		cancelHeadingCommand();
		controlPositionCordinatesWithGains(dt, poiYawHoldX, poiYawHoldY, 1.0f);
		poiTrackVelocity();
		poiResumePending = 1;
		return 0;
	}
	poiYawHolding = 0;

	if (poiResumePending) {
		poiApplyResume();
	}
	runPOISequence(dt);

	if (poiPhase == POI_PHASE_DONE) {
		poiReleaseMission();
		return 1;
	}
	return 0;
}

/** Mission reset (abort, RTH, failsafe, a new mission): controlled stop. */
void cancelPOIOperation(void) {
	if (!poiMissionOwned) {
		return;
	}
	poiCancel();
	poiReleaseMission();
}

/** The manager re-locked after a pitch/roll override: resume the phase on the next step. */
void poiOperationRelock(void) {
	if (poiMissionOwned && poiIsRunning()) {
		poiPrevValid = 0;                     // avoids a jump in the swept angle
		poiResumePending = 1;
	}
}

/* ---------------------------------------------------------------- radio switch interface */

__ATTR_ITCM_TEXT
void handlePOI(float dt) {
	if (poiMissionOwned) {
		cancelPOIOperation();
		return;                               // the mission owns it
	}
	// Anything unsafe or unwanted cancels: no navigation data, or the yaw stick (the manager catches pitch and roll)
	if (poiPhase != POI_PHASE_DONE && (!fcStatusData.isFlying || !isNavDataReliable() || POI_PILOT_YAW_ACTIVE())) {
		poiCancel();
		return;
	}

	switch (poiPhase) {
	case POI_PHASE_ARMED:
		poiHoldReference(dt);
		// Start once the drone is at rest in the locked hold (a request made while moving simply waits)
		if (getGroundSpeed() <= POSITION_POI_START_MAX_SPEED) {
			if (!poiStartAhead(poiCfgRadius, poiCfgCcw, poiCfgIterations)) {
				poiPhase = POI_PHASE_DONE;    // rejected: no retry until the switch is cycled
			}
		}
		break;
	case POI_PHASE_DONE:
		poiHoldReference(dt);
		break;
	default:
		runPOISequence(dt);
		if (poiPhase == POI_PHASE_DONE) {
			poiHoldReference(dt);
		}
		break;
	}
}

/** Called whenever the radio POI must not be running (stick, RTH, mission, nav off, switch low). */
void resetPOIStates(void) {
	if (poiMissionOwned) {
		return;                               // a mission POI is controlled by the mission, not by the switch
	}
	poiCancel();
	poiPhase = poiRestingPhase();
}

void setPOIParameters(float radius, uint8_t ccw, uint8_t iterations) {
	poiCfgRadius = radius;
	poiCfgCcw = ccw;
	poiCfgIterations = iterations;
}

uint8_t getPOIPhase(void) {
	return (uint8_t) poiPhase;
}
