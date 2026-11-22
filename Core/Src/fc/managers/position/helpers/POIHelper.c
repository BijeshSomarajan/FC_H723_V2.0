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
static float poiCfgSpeed = POSITION_POI_SPEED;

static float poiCenterX = 0.0f, poiCenterY = 0.0f;
static float poiEntryX = 0.0f, poiEntryY = 0.0f, poiEntryHeading = 0.0f;   // entry heading: from the entry point to the centre, constant
static float poiNoseHold = 0.0f;         // nose direction held during ENTRY, SETTLE and RETURN
static float poiAlignHeading = 0.0f;     // heading ALIGN waits for
static float poiHdgTarget = 0.0f;        // orbit nose target: driven by the orbit rate, corrected slowly by the measured bearing
static float poiReturnX = 0.0f, poiReturnY = 0.0f;
static uint8_t poiReturnToAnchor = 0;
static float poiTargetX = 0.0f, poiTargetY = 0.0f;       // current leg target
static float poiHoldX = 0.0f, poiHoldY = 0.0f;           // held during ALIGN and SETTLE
static float poiYawHoldX = 0.0f, poiYawHoldY = 0.0f;
static float poiRadius = 0.0f, poiOrbitSpeed = 0.0f, poiLegSpeed = POSITION_POI_SPEED;
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

/** Measured bearing from the drone to the centre. Only used once per phase change, never every cycle. */
static float poiBearingToCenter(void) {
	float px = positionCordinateData.xPosition - poiCenterX;
	float py = positionCordinateData.yPosition - poiCenterY;
	if ((px * px + py * py) < (POSITION_POI_MIN_DIST * POSITION_POI_MIN_DIST)) {
		return poiEntryHeading;
	}
	return calculateBearing(px, py);
}

/** Holds the current nose direction. Commanded every cycle: the heading helper slews from headingRef, so repeats are harmless. */
static void poiAimNose(void) {
	setAbsoluteHeadingCommand(poiNoseHold);
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

/** Legs: acceleration limit, deceleration-only feedforward and velocity setpoint. */
__ATTR_ITCM_TEXT
static void poiCommandVelocity(float desiredVx, float desiredVy, float dt) {
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

	float invDt = (dt > 1e-4f) ? (1.0f / dt) : 0.0f;
	float ax = decelerating ? (dvx * invDt) : 0.0f;
	float ay = decelerating ? (dvy * invDt) : 0.0f;
	float accMagSq = ax * ax + ay * ay;
	if (accMagSq > (POSITION_POI_FF_ACC_MAX * POSITION_POI_FF_ACC_MAX)) {
		float s = POSITION_POI_FF_ACC_MAX / fastSqrtf(accMagSq);
		ax *= s;
		ay *= s;
	}
	setPositionAccelFeedforward(ax, ay);
	setExpectedPositionVelocity(dt, poiVxCmd, poiVyCmd);
}

/** Orbit: lightly smoothed velocity setpoint and an analytic acceleration feedforward (not differentiated from the noisy setpoint). */
__ATTR_ITCM_TEXT
static void poiCommandVelocityFF(float desiredVx, float desiredVy, float ffx, float ffy, float dt) {
	float alpha = dt / (POSITION_POI_ORBIT_CMD_TAU + dt);
	poiVxCmd += alpha * (desiredVx - poiVxCmd);
	poiVyCmd += alpha * (desiredVy - poiVyCmd);
	float accMagSq = ffx * ffx + ffy * ffy;
	if (accMagSq > (POSITION_POI_FF_ACC_MAX * POSITION_POI_FF_ACC_MAX)) {
		float s = POSITION_POI_FF_ACC_MAX / fastSqrtf(accMagSq);
		ffx *= s;
		ffy *= s;
	}
	setPositionAccelFeedforward(ffx, ffy);
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
		float speed = fminf(poiLegSpeed, braking);
		float inv = 1.0f / distance;
		desiredVx = dx * inv * speed;
		desiredVy = dy * inv * speed;
	}
	poiCommandVelocity(desiredVx, desiredVy, dt);
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
	// First alignment: the constant entry heading. After a pilot move mid-orbit: face the centre from where the drone is now (computed once)
	poiAlignHeading = resumeOrbit ? poiBearingToCenter() : poiEntryHeading;
	poiPhase = POI_PHASE_ALIGN;
	poiPhaseTime = 0.0f;
	poiStableTime = 0.0f;
	poiTrackVelocity();
}

static void beginPOIOrbit(uint8_t resume) {
	if (!resume) {
		poiSwept = 0.0f;
	}
	poiHdgTarget = poiAlignHeading;           // the nose is already there
	poiPrevValid = 0;                         // seeded on the first orbit cycle
	poiPhase = POI_PHASE_ORBIT;
	poiPhaseTime = 0.0f;
	poiTrackVelocity();
}

static void beginPOISettle(void) {
	poiNoseHold = wrap360(poiHdgTarget);      // the nose direction at the end of the orbit (facing the centre)
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

	// Tangential speed and the acceleration of its start/end profile
	float vr = constrainToRangeF(-POSITION_POI_RADIAL_KP * (d - poiRadius), -POSITION_POI_RADIAL_VMAX, POSITION_POI_RADIAL_VMAX);
	float vt = fastSqrtf(fmaxf(poiOrbitSpeed * poiOrbitSpeed - vr * vr, 0.0f));
	float aTan = 0.0f;
	float startSpeed = fastSqrtf(2.0f * POSITION_POI_BRAKE_DECEL * (arcDone + POSITION_POI_ARC_STOP));
	float endSpeed = fastSqrtf(2.0f * POSITION_POI_BRAKE_DECEL * fmaxf(arcRemaining - POSITION_POI_ARC_STOP, 0.0f));
	if (startSpeed < vt) {
		vt = startSpeed;
		aTan = POSITION_POI_BRAKE_DECEL;
	}
	if (endSpeed < vt) {
		vt = endSpeed;
		aTan = -POSITION_POI_BRAKE_DECEL;
	}

	// Heading gate on the smooth nose target, not on the noisy bearing
	float hdgErr = fabsf(wrap180(poiHdgTarget - sensorAttitudeData.heading));
	float hdgScale = constrainToRangeF((POSITION_POI_HDG_ZERO_DEG - hdgErr) / (POSITION_POI_HDG_ZERO_DEG - POSITION_POI_HDG_FULL_DEG), 0.0f, 1.0f);
	vt *= hdgScale;
	if (hdgScale < 1.0f) {
		aTan = 0.0f;
	}

	// Nose target: driven by the orbit rate, corrected slowly by the measured bearing (complementary filter)
	float bearing = calculateBearing(px, py);
	float omegaDeg = poiDirection * (vt / poiRadius) * (180.0f / (float) M_PI);
	poiHdgTarget = wrap360(poiHdgTarget + (omegaDeg + POSITION_POI_HDG_TRACK_K * wrap180(bearing - poiHdgTarget)) * dt);
	setAbsoluteHeadingCommand(poiHdgTarget);

	// Setpoint, and an analytic feedforward: centripetal (toward the centre) plus the profile's tangential acceleration
	// Tangent for increasing angle (clockwise seen from above) is (-ry, rx)
	float tx = poiDirection * (-ry);
	float ty = poiDirection * (rx);
	float centripetal = (vt * vt) / poiRadius;
	poiCommandVelocityFF(tx * vt + rx * vr, ty * vt + ry * vr, -centripetal * rx + aTan * tx, -centripetal * ry + aTan * ty, dt);
	return 0;
}

/* ---------------------------------------------------------------- start */

/** Validates and starts the sequence. anchored = 1: the drone starts at the centre (a mission waypoint), so it backs up
 *  one radius behind the nose and returns to the centre. anchored = 0: radio start, nearest point on the circle, returns to the start point. */
static uint8_t poiStart(float radius, float centerX, float centerY, uint8_t ccw, uint8_t iterations, float speed, uint8_t anchored) {
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

	float ux, uy;
	if (anchored || d < POSITION_POI_MIN_DIST) {
		float h = convertDegToRadF(sensorAttitudeData.heading);   // behind the nose, so the nose already faces the centre
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
	poiEntryHeading = calculateBearing(ux, uy);   // bearing from the entry point to the centre: constant, free of position noise
	poiNoseHold = poiEntryHeading;
	poiAlignHeading = poiEntryHeading;
	poiHdgTarget = poiEntryHeading;
	poiReturnToAnchor = (anchored || POSITION_POI_RETURN_TO_START == 1) ? 1 : 0;
	poiReturnX = anchored ? centerX : positionCordinateData.xPosition;
	poiReturnY = anchored ? centerY : positionCordinateData.yPosition;

	// Speed: the request sets the legs and caps the orbit, which is further limited by lateral acceleration and by what the nose can follow
	speed = fabsf(speed);
	if (!(speed > 0.0f)) {
		speed = POSITION_POI_SPEED;
	}
	speed = constrainToRangeF(speed, POSITION_POI_SPEED_MIN, POSITION_POI_SPEED_MAX);
	poiLegSpeed = speed;
	float latLimited = fastSqrtf(POSITION_POI_LAT_ACC * radius);
	float yawLimited = POSITION_POI_YAW_FRACTION * convertDegToRadF(HEADING_COMMAND_ROTATION_RATE) * radius;
	poiOrbitSpeed = fminf(speed, fminf(latLimited, yawLimited));
	logPrintf("[POI] R:%.1f requested:%.2f orbit:%.2f\n", radius, speed, poiOrbitSpeed);

	poiResumePending = 0;
	poiResumeOrbit = 0;
	poiYawHolding = 0;
	cancelHeadingCommand();                   // keep the nose while flying to the entry point
	beginPOIEntry();
	return 1;
}

/** Radio start: the centre is R metres ahead of the nose, so the drone is already on the circle facing the centre. */
static uint8_t poiStartAhead(float radius, uint8_t ccw, uint8_t iterations, float speed) {
	float h = convertDegToRadF(sensorAttitudeData.heading);
	return poiStart(radius, positionCordinateData.xPosition + radius * cosApproxF(h), positionCordinateData.yPosition + radius * sinApproxF(h), ccw, iterations, speed, 0);
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
		setAbsoluteHeadingCommand(poiAlignHeading);
		// Numeric completion: the heading reference has arrived and the real nose is within tolerance
		float refErr = fabsf(wrap180(poiAlignHeading - fcStatusData.headingRef));
		float noseErr = fabsf(wrap180(poiAlignHeading - sensorAttitudeData.heading));
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
		poiAimNose();                         // the nose stays where the orbit left it, facing the centre
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

uint8_t decodePOIValue(float value, float *radius, uint8_t *iterations, uint8_t *ccw, float *speed) {
	if (!(value >= 0.0f) || value > 16777215.0f) {   // also rejects NaN
		return poiReject(9);
	}
	uint32_t packed = (uint32_t) (value + 0.5f);
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
	uint32_t sp = (packed >> POI_VALUE_SPEED_SHIFT) & POI_VALUE_SPEED_MASK;
	float spd = (float) sp / 10.0f;               // 0 = not given
	if (sp != 0 && (spd < POSITION_POI_SPEED_MIN || spd > POSITION_POI_SPEED_MAX)) {
		return poiReject(13);
	}
	*radius = r;
	*iterations = (uint8_t) n;
	*ccw = (packed & POI_VALUE_DIR_BIT) ? 1 : 0;
	*speed = spd;
	return 1;
}

uint8_t startPOIOperation(double anchorLat, double anchorLon, float radius, uint8_t ccw, uint8_t iterations, float speed) {
	if (!isfinite(anchorLat) || !isfinite(anchorLon)) {
		return poiReject(5);
	}
	float cx, cy;
	convertGNSSToXYCordinates(anchorLat, anchorLon, fcStatusData.positionLatHome, fcStatusData.positionLongHome, &cx, &cy);
	if (!poiStart(radius, cx, cy, ccw, iterations, speed, 1)) {
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
		cancelPOIOperation();                 // only reached with no mission or RTH active, so a mission-owned POI is stale
		return;
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
			if (!poiStartAhead(poiCfgRadius, poiCfgCcw, poiCfgIterations, poiCfgSpeed)) {
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

void setPOIParameters(float radius, uint8_t ccw, uint8_t iterations, float speed) {
	poiCfgRadius = radius;
	poiCfgCcw = ccw;
	poiCfgIterations = iterations;
	poiCfgSpeed = (speed > 0.0f) ? speed : POSITION_POI_SPEED;
}

uint8_t getPOIPhase(void) {
	return (uint8_t) poiPhase;
}
