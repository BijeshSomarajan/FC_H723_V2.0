#include "AltitudeCommandHelper.h"

#include <math.h>
#include <sys/_stdint.h>

#include "../../../FCConfig.h"
#include "../../../memory/Memory.h"
#include "../../../status/FCStatus.h"
#include "../../../util/MathUtil.h"
#include "../../position/common/PositionCommon.h"
#include "../../position/helpers/PositionManagerHelper.h"
#include "../AltitudeManager.h"

uint8_t altCommandMode = 0;
uint8_t altCommandComplete = 0;
uint8_t altCommandSkipped = 0;
AltCommandType altCommandType = ALT_COMMAND_TYPE_NORMAL;

float altCommandTargetAltitude = 0.0f;
float altCommandLiftOffThrottle = 0.0f;
uint8_t altCommandThrottleRefreshRequired = 0;
uint8_t altCommandStepIssued = 0;
float altCommandStepTimer = 0;
float targetAltForAltChangeCommand;

__ATTR_ITCM_TEXT
uint8_t isTargetAltitudeReached(void) {
	return (fabsf(altCommandTargetAltitude - positionCordinateData.zPosition) <= ALT_COMMAND_TARGET_ALTITUDE_TOLERANCE);
}

__ATTR_ITCM_TEXT
void adjustPreLiftOffBaseThrottle(float dt, uint8_t increasing) {
	if (increasing) {
		float rampRate = altCommandLiftOffThrottle / ALT_COMMAND_PRE_LIFTOFF_THROTTLE_INCREASE_RAMP_TIME;
		fcStatusData.currentThrottle += rampRate * dt;
	} else {
		float rampRate = altCommandLiftOffThrottle / ALT_COMMAND_PRE_LIFTOFF_THROTTLE_DECREASE_RAMP_TIME;
		fcStatusData.currentThrottle -= rampRate * dt;
	}
	fcStatusData.currentThrottle = constrainToRangeF(fcStatusData.currentThrottle, 0.0f, altCommandLiftOffThrottle + 1);
	altCommandThrottleRefreshRequired = 1;
}

void startAltCommand(float currentAltitude, float targetAltitude, AltCommandType type) {
	altCommandTargetAltitude = targetAltitude;
	altCommandMode = 1;
	altCommandComplete = 0;
	altCommandSkipped = 0;
	altCommandType = type;
	altCommandLiftOffThrottle = fcStatusData.liftOffThrottlePercent * MAX_PERMISSIBLE_THROTTLE_DELTA;
	altCommandThrottleRefreshRequired = (type != ALT_COMMAND_TYPE_TAKEOFF);
	altCommandStepIssued = 0;
	altCommandStepTimer = 0;
}

void setTargetAltForAltChangeCommand(float alt) {
	targetAltForAltChangeCommand = alt;
}

float getTargetAltForAltChangeCommand(void) {
	return targetAltForAltChangeCommand;
}

void skipAltCommand(void) {
	resetAltCommandStates();
	altCommandSkipped = 1;
}

void abortAltCommand(void) {
	resetAltCommandStates();
}

uint8_t isAltCommandMode(void) {
	return altCommandMode;
}

uint8_t isAltCommandComplete(void) {
	return altCommandComplete;
}

uint8_t isAltCommandSkipped(void) {
	return altCommandSkipped;
}

void resetAltCommandStates(void) {
	altCommandMode = 0;
	altCommandType = ALT_COMMAND_TYPE_NONE;
	altCommandTargetAltitude = 0.0f;
	altCommandThrottleRefreshRequired = 0;
	altCommandStepIssued = 0;
	altCommandSkipped = 0;
}

__ATTR_ITCM_TEXT
uint8_t isAltCommandThrottleUpdateRequired() {
	uint8_t required = altCommandThrottleRefreshRequired;
	altCommandThrottleRefreshRequired = 0;
	return required;
}

__ATTR_ITCM_TEXT
void handleTakeOff(float dt) {
	if (fcStatusData.currentThrottle < altCommandLiftOffThrottle) {
		adjustPreLiftOffBaseThrottle(dt, 1);
	} else if (isTargetAltitudeReached()) {
		altCommandMode = 0;
		altCommandComplete = 1;
		altCommandThrottleRefreshRequired = 1;
	} else {
		fcStatusData.altitudeRef = altCommandTargetAltitude;
		fcStatusData.altitudeHoldState = ALT_HOLD_STATE_LOCKED;
	}
}

__ATTR_ITCM_TEXT
void handleStepAltChange(float dt) {
	if (altCommandType == ALT_COMMAND_TYPE_LANDING && fcStatusData.currentThrottle < altCommandLiftOffThrottle) {
		fcStatusData.altitudeHoldState = ALT_HOLD_STATE_IDLE;
		if (fcStatusData.currentThrottle >= ALT_COMMAND_THROTTLE_ZERO_TOLERANCE) {
			adjustPreLiftOffBaseThrottle(dt, 0);
		} else {
			altCommandMode = 0;
			altCommandComplete = 1;
		}
	} else if (fcStatusData.altitudeHoldState == ALT_HOLD_STATE_LOCKED) {
		if (!altCommandStepIssued) {
			float remainingDistance = fabsf(altCommandTargetAltitude - fcStatusData.altitudeRef);
			float step = constrainToRangeF(remainingDistance * ALT_COMMAND_DISTANCE_STEP_GAIN, ALT_COMMAND_DISTANCE_STEP_MIN, ALT_COMMAND_DISTANCE_STEP_MAX);
			fcStatusData.altitudeRef += constrainToRangeF(altCommandTargetAltitude - fcStatusData.altitudeRef, -step, step);
			altCommandStepIssued = 1;
			altCommandStepTimer = 0.0f;
		} else {
			altCommandStepTimer += dt;
			uint8_t arrived = (fabsf(positionCordinateData.zPosition - fcStatusData.altitudeRef) <= ALT_COMMAND_TARGET_ALTITUDE_TOLERANCE);
			if (arrived || (altCommandStepTimer >= ALT_COMMAND_STEP_TIMEOUT)) {
				if (isTargetAltitudeReached()) {
					altCommandMode = 0;
					altCommandComplete = 1;
				} else {
					altCommandStepIssued = 0;
				}
				altCommandThrottleRefreshRequired = 1;                 // nudge (rebase) on arrival or timeout
			}
		}
	}
}

__ATTR_ITCM_TEXT
void manageAltCommand(float dt) {
	if (altCommandMode && (dt > 0.0f)) {
		if (altCommandType == ALT_COMMAND_TYPE_TAKEOFF) {
			handleTakeOff(dt);
		} else if (altCommandType != ALT_COMMAND_TYPE_NONE) {
			handleStepAltChange(dt);
		}
	}
}
