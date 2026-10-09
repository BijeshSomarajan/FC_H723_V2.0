#include "AltitudeCommandHelper.h"

#include <math.h>
#include <sys/_stdint.h>

#include "../../../control/ControlData.h"
#include "../../../status/FCStatus.h"
#include "../../../util/MathUtil.h"
#include "../../position/common/PositionCommon.h"
#include "../../../FCConfig.h"

AltCommandState altCommandState = ALT_COMMAND_STATE_IDLE;

uint8_t altCommandMode = 0;
uint8_t altCommandActive = 0;
uint8_t altCommandComplete = 0;

AltCommandType altCommandType = ALT_COMMAND_TYPE_NORMAL;

float altCommandTargetAltitude = 0.0f;
float altCommandCurrentAltitude = 0.0f;
float altCommandPhaseStartThrottle = 0.0f;
float altCommandLiftOffThrottle = 0;
float altCommandPhaseStartAltitude = 0.0f;
float altCommandSettleTimer = 0.0f;

__ATTR_ITCM_TEXT
float getAltitudeError(void) {
	return altCommandTargetAltitude - positionCordinateData.zPosition;
}

__ATTR_ITCM_TEXT
uint8_t isTargetAltitudeReached(void) {
	return (fabsf(getAltitudeError()) <= ALT_COMMAND_ALTITUDE_TOLERANCE);
}

__ATTR_ITCM_TEXT
uint8_t isMovingUp(void) {
	if (altCommandType == ALT_COMMAND_TYPE_LANDING) {
		return 0;
	}
	return (getAltitudeError() >= 0.0f);   // live sign, flips automatically on overshoot
}

__ATTR_ITCM_TEXT
static float getVelocityLimit(float lowLimit, float farLimit) {
	float extra = fabsf(getAltitudeError()) - ALT_COMMAND_FAR_DISTANCE;
	if (extra <= 0.0f) {
		return lowLimit;
	}
	return fminf(lowLimit + (ALT_COMMAND_FAR_VELOCITY_GAIN * extra), farLimit);
}

__ATTR_ITCM_TEXT
uint8_t isVelocityLimitReached(void) {
	float zVelocity = positionCordinateData.zVelocity;
	if (isMovingUp()) {
		return (zVelocity >= getVelocityLimit(ALT_COMMAND_MAX_CLIMB_VELOCITY, ALT_COMMAND_FAR_MAX_CLIMB_VELOCITY));
	}
	return (zVelocity <= -getVelocityLimit(ALT_COMMAND_MAX_DESCENT_VELOCITY, ALT_COMMAND_FAR_MAX_DESCENT_VELOCITY));
}


__ATTR_ITCM_TEXT
uint8_t isPhaseDistanceLimitReached(void) {
	return (fabsf(positionCordinateData.zPosition - altCommandPhaseStartAltitude) >= ALT_COMMAND_MAX_PHASE_DISTANCE);
}

__ATTR_ITCM_TEXT
uint8_t isVelocitySettled(void) {
	return (fabsf(positionCordinateData.zVelocity) <= ALT_COMMAND_RESUME_VELOCITY);
}

__ATTR_ITCM_TEXT
uint8_t isPhaseThrottleLimitReached(void) {
	float limit = (altCommandType == ALT_COMMAND_TYPE_TAKEOFF) ? ALT_COMMAND_TAKEOFF_MAX_PHASE_THROTTLE_DELTA : ALT_COMMAND_MAX_PHASE_THROTTLE_DELTA;
	float throttleDelta = fabsf(fcStatusData.currentThrottle - altCommandPhaseStartThrottle);
	return (throttleDelta >= limit);
}

__ATTR_ITCM_TEXT
void adjustPostLiftOffBaseThrottle(float dt) {
	float rate = ALT_COMMAND_POST_LIFTOFF_BASE_THROTTLE_RATE * (fabsf(getAltitudeError()) / ALT_COMMAND_RATE_FULL_ERROR);
	rate = constrainToRangeF(rate, ALT_COMMAND_MIN_BASE_THROTTLE_RATE, ALT_COMMAND_POST_LIFTOFF_BASE_THROTTLE_RATE);
	// Landing completes on throttle, not altitude, so don't let baro drift change its rate
	if (altCommandType == ALT_COMMAND_TYPE_LANDING) {
		rate = ALT_COMMAND_POST_LIFTOFF_BASE_THROTTLE_RATE;
	}
	// Takeoff punch: extra rate near the ground, fading to zero with altitude gained.
	// Squared linear falloff approximates exp(-x/d) without expf; it reaches zero at 3*d.
	else if ((altCommandType == ALT_COMMAND_TYPE_TAKEOFF) && isMovingUp()) {
		float takeoffDelta = altCommandTargetAltitude - altCommandCurrentAltitude;
		float decayDist = constrainToRangeF(takeoffDelta * ALT_COMMAND_TAKEOFF_BOOST_DECAY_FRACTION, ALT_COMMAND_TAKEOFF_BOOST_MIN_DECAY_DIST, ALT_COMMAND_TAKEOFF_BOOST_MAX_DECAY_DIST);
		float gained = fmaxf(positionCordinateData.zPosition - altCommandCurrentAltitude, 0.0f);
		float k = 1.0f - gained / (3.0f * decayDist);
		if (k > 0.0f) {
			rate += (ALT_COMMAND_TAKEOFF_BOOST_FACTOR * ALT_COMMAND_POST_LIFTOFF_BASE_THROTTLE_RATE) * k * k;
		}
	}
	float step = rate * dt;
	if (isMovingUp()) {
		fcStatusData.currentThrottle += step;
	} else {
		fcStatusData.currentThrottle -= step;
	}
	fcStatusData.currentThrottle = constrainToRangeF(fcStatusData.currentThrottle, altCommandLiftOffThrottle, MAX_PERMISSIBLE_THROTTLE_DELTA);
}

__ATTR_ITCM_TEXT
void adjustPreLiftOffBaseThrottle(float dt, uint8_t increasing) {
	if (increasing) {
		float rampRate = altCommandLiftOffThrottle / ALT_COMMAND_TAKEOFF_RAMP_TIME;
		fcStatusData.currentThrottle += rampRate * dt;
		fcStatusData.currentThrottle = constrainToRangeF(fcStatusData.currentThrottle, 0.0f, altCommandLiftOffThrottle + 1);
	} else {
		float rampRate = altCommandLiftOffThrottle / ALT_COMMAND_LANDING_RAMP_TIME;
		fcStatusData.currentThrottle -= rampRate * dt;
		fcStatusData.currentThrottle = constrainToRangeF(fcStatusData.currentThrottle, 0.0f, altCommandLiftOffThrottle + 1);
	}
}

void startAltCommand(float currentAltitude, float targetAltitude, AltCommandType type) {
	altCommandTargetAltitude = targetAltitude;
	altCommandCurrentAltitude = currentAltitude;
	altCommandMode = 1;
	altCommandActive = 1;
	altCommandComplete = 0;
	altCommandType = type;
	altCommandPhaseStartThrottle = fcStatusData.currentThrottle;
	altCommandState = ALT_COMMAND_STATE_ADJUSTING;
	altCommandLiftOffThrottle = fcStatusData.liftOffThrottlePercent * MAX_PERMISSIBLE_THROTTLE_DELTA;
	altCommandPhaseStartAltitude = positionCordinateData.zPosition;
	altCommandSettleTimer = 0.0f;
}

void abortAltCommand(void) {
	resetAltCommandStates();
}

uint8_t isAltCommandActive(void) {
	return altCommandActive;
}

uint8_t isAltCommandMode(void) {
	return altCommandMode;
}

uint8_t isAltCommandComplete(void) {
	return altCommandComplete;
}

void resetAltCommandStates(void) {
	altCommandActive = 0;
	altCommandMode = 0;
	altCommandComplete = 0;
	altCommandType = ALT_COMMAND_TYPE_NORMAL;
	altCommandState = ALT_COMMAND_STATE_IDLE;
	altCommandTargetAltitude = 0.0f;
	altCommandCurrentAltitude = 0.0f;
	altCommandPhaseStartThrottle = 0.0f;
	altCommandPhaseStartAltitude = 0.0f;
	altCommandSettleTimer = 0.0f;
}

__ATTR_ITCM_TEXT
void manageAltCommand(float dt) {
	if (!altCommandMode || (dt <= 0.0f)) {
		return;
	}
	switch (altCommandState) {
	case ALT_COMMAND_STATE_ADJUSTING:
		if (altCommandType == ALT_COMMAND_TYPE_TAKEOFF) {
			if (fcStatusData.currentThrottle < altCommandLiftOffThrottle) {
				altCommandActive = 1;
				adjustPreLiftOffBaseThrottle(dt, 1);
				altCommandPhaseStartThrottle = fcStatusData.currentThrottle;
				altCommandPhaseStartAltitude = positionCordinateData.zPosition;   // leg starts when the drone does
			} else if (isTargetAltitudeReached()) {
				altCommandActive = 0;
				altCommandMode = 0;
				altCommandComplete = 1;
				altCommandState = ALT_COMMAND_STATE_IDLE;
			} else if (isVelocityLimitReached() || isPhaseThrottleLimitReached() || isPhaseDistanceLimitReached()) {
				altCommandActive = 0;
				altCommandState = ALT_COMMAND_STATE_SETTLING;
			} else {
				adjustPostLiftOffBaseThrottle(dt);
				altCommandActive = 1;
			}
		} else if (altCommandType == ALT_COMMAND_TYPE_LANDING) {
			if (fcStatusData.currentThrottle > altCommandLiftOffThrottle) {
				if (isVelocityLimitReached() || isPhaseThrottleLimitReached() || isPhaseDistanceLimitReached()) {
					altCommandActive = 0;
					altCommandState = ALT_COMMAND_STATE_SETTLING;
				} else {
					adjustPostLiftOffBaseThrottle(dt);
					altCommandActive = 1;
				}
			} else if (fcStatusData.currentThrottle > ALT_COMMAND_THROTTLE_ZERO_TOLERANCE) {
				altCommandActive = 1;
				adjustPreLiftOffBaseThrottle(dt, 0);
				altCommandPhaseStartThrottle = fcStatusData.currentThrottle;
			} else {
				altCommandActive = 0;
				altCommandMode = 0;
				altCommandComplete = 1;
				altCommandState = ALT_COMMAND_STATE_IDLE;
			}
		} else {
			if (isTargetAltitudeReached()) {
				altCommandActive = 0;
				altCommandMode = 0;
				altCommandComplete = 1;
				altCommandState = ALT_COMMAND_STATE_IDLE;
			} else if (isVelocityLimitReached() || isPhaseThrottleLimitReached() || isPhaseDistanceLimitReached()) {
				altCommandActive = 0;
				altCommandState = ALT_COMMAND_STATE_SETTLING;
			} else {
				adjustPostLiftOffBaseThrottle(dt);
				altCommandActive = 1;
			}
		}
		break;
	case ALT_COMMAND_STATE_SETTLING:
		altCommandSettleTimer += dt;
		if (isVelocitySettled() || (altCommandSettleTimer >= ALT_COMMAND_SETTLE_TIMEOUT)) {
			altCommandSettleTimer = 0.0f;   // reset on exit, so it is 0 on every entry to SETTLING
			altCommandPhaseStartThrottle = fcStatusData.currentThrottle;
			altCommandPhaseStartAltitude = positionCordinateData.zPosition;
			altCommandActive = 1;
			altCommandState = ALT_COMMAND_STATE_ADJUSTING;
		} else {
			altCommandActive = 0;
		}
		break;
	case ALT_COMMAND_STATE_IDLE:
	default:
		altCommandActive = 0;
		altCommandMode = 0;
		altCommandState = ALT_COMMAND_STATE_IDLE;
		break;
	}
}
