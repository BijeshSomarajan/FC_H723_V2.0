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
uint8_t isVelocityLimitReached(void) {
	float zVelocity = positionCordinateData.zVelocity;
	if (isMovingUp()) {
		return (zVelocity >= ALT_COMMAND_MAX_CLIMB_VELOCITY);
	}
	return (zVelocity <= -ALT_COMMAND_MAX_DESCENT_VELOCITY);
}

__ATTR_ITCM_TEXT
uint8_t isVelocitySettled(void) {
	return (fabsf(positionCordinateData.zVelocity) <= ALT_COMMAND_RESUME_VELOCITY);
}

__ATTR_ITCM_TEXT
uint8_t isPhaseThrottleLimitReached(void) {
	float throttleDelta = fabsf(fcStatusData.currentThrottle - altCommandPhaseStartThrottle);
	return (throttleDelta >= ALT_COMMAND_MAX_PHASE_THROTTLE_DELTA);
}

__ATTR_ITCM_TEXT
void adjustBaseThrottle(float dt) {
	float rate = ALT_COMMAND_BASE_THROTTLE_RATE * (fabsf(getAltitudeError()) / ALT_COMMAND_RATE_FULL_ERROR);
	rate = constrainToRangeF(rate, ALT_COMMAND_MIN_BASE_THROTTLE_RATE, ALT_COMMAND_BASE_THROTTLE_RATE);
	// Landing completes on throttle, not altitude, so don't let baro drift change its rate
	if (altCommandType == ALT_COMMAND_TYPE_LANDING) {
		rate = ALT_COMMAND_BASE_THROTTLE_RATE;
	}
	float step = rate * dt;
	if (isMovingUp()) {
		fcStatusData.currentThrottle += step;
	} else {
		fcStatusData.currentThrottle -= step;
	}
	fcStatusData.currentThrottle = constrainToRangeF(fcStatusData.currentThrottle, 0.0f, MAX_PERMISSIBLE_THROTTLE_DELTA);
}

__ATTR_ITCM_TEXT
void adjustBaseThrottleExponential(float dt, uint8_t increasing) {
	//Added one to
	float liftOffThrottle = (fcStatusData.liftOffThrottlePercent * MAX_PERMISSIBLE_THROTTLE_DELTA) + 1;
	float decay = expf(-ALT_COMMAND_LOW_THROTTLE_DECAY_RATE * dt);
	if (increasing) {
		fcStatusData.currentThrottle += (liftOffThrottle - fcStatusData.currentThrottle) * (1.0f - decay);
		fcStatusData.currentThrottle = constrainToRangeF(fcStatusData.currentThrottle, 0.0f, liftOffThrottle);
	} else {
		fcStatusData.currentThrottle *= decay;
		fcStatusData.currentThrottle = constrainToRangeF(fcStatusData.currentThrottle, 0.0f, fcStatusData.currentThrottle);
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
}

__ATTR_ITCM_TEXT
void manageAltCommand(float dt) {
	if (!altCommandMode || (dt <= 0.0f)) {
		return;
	}
	switch (altCommandState) {
	case ALT_COMMAND_STATE_ADJUSTING:
		if (altCommandType == ALT_COMMAND_TYPE_TAKEOFF) {
			float liftOffThrottle = fcStatusData.liftOffThrottlePercent * MAX_PERMISSIBLE_THROTTLE_DELTA;
			if (fcStatusData.currentThrottle < liftOffThrottle) {
				altCommandActive = 1;
				adjustBaseThrottleExponential(dt, 1);
				altCommandPhaseStartThrottle = fcStatusData.currentThrottle;
			} else if (isTargetAltitudeReached()) {
				altCommandActive = 0;
				altCommandMode = 0;
				altCommandComplete = 1;
				altCommandState = ALT_COMMAND_STATE_IDLE;
			} else if (isVelocityLimitReached() || isPhaseThrottleLimitReached()) {
				altCommandActive = 0;
				altCommandState = ALT_COMMAND_STATE_SETTLING;
			} else {
				altCommandActive = 1;
				adjustBaseThrottle(dt);
			}
		} else if (altCommandType == ALT_COMMAND_TYPE_LANDING) {
			float liftOffThrottle = fcStatusData.liftOffThrottlePercent * MAX_PERMISSIBLE_THROTTLE_DELTA;
			if (fcStatusData.currentThrottle > liftOffThrottle) {
				if (isVelocityLimitReached() || isPhaseThrottleLimitReached()) {
					altCommandActive = 0;
					altCommandState = ALT_COMMAND_STATE_SETTLING;
				} else {
					altCommandActive = 1;
					adjustBaseThrottle(dt);
				}
			} else if (fcStatusData.currentThrottle > ALT_COMMAND_THROTTLE_ZERO_TOLERANCE) {
				altCommandActive = 1;
				adjustBaseThrottleExponential(dt, 0);
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
			} else if (isVelocityLimitReached() || isPhaseThrottleLimitReached()) {
				altCommandActive = 0;
				altCommandState = ALT_COMMAND_STATE_SETTLING;
			} else {
				altCommandActive = 1;
				adjustBaseThrottle(dt);
			}
		}
		break;
	case ALT_COMMAND_STATE_SETTLING:
		if (isVelocitySettled()) {
			altCommandPhaseStartThrottle = fcStatusData.currentThrottle;
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
