#include "HeadingCommandHelper.h"

#include <math.h>

#include "../../../sensors/attitude/AttitudeSensor.h"
#include "../../../status/FCStatus.h"
#include "../../../util/MathUtil.h"

static float headingCommandTarget = 0.0f;
static float headingCommandTimer = 0.0f;
static float headingCommandTimeout = 0.0f;

void setAbsoluteHeadingCommand(float headingDegree) {
	headingCommandTarget = wrap360(headingDegree);
	headingCommandTimer = 0.0f;
	// Allow twice the ideal turn time, so the guard scales with the size of the turn
	headingCommandTimeout = 2.0f * fabsf(wrap180(headingCommandTarget - fcStatusData.headingRef)) / HEADING_COMMAND_ROTATION_RATE + HEADING_COMMAND_TIMEOUT_MARGIN;
	fcStatusData.isHeadingChangeModeActive = 1;
}

void setRelativeHeadingCommand(float deltaDeg) {
	setAbsoluteHeadingCommand(fcStatusData.headingRef + deltaDeg);
}

void cancelHeadingCommand(void) {
	fcStatusData.isHeadingChangeModeActive = 0;
}

void handleHeadingCommand(float dt) {
	if (fcStatusData.isHeadingChangeModeActive) {
		if (!fcStatusData.isFlying) {
			// Nothing to turn on the ground: drop the command so nothing keeps waiting on it
			fcStatusData.isHeadingChangeModeActive = 0;
		} else {
			float err = wrap180(headingCommandTarget - fcStatusData.headingRef);
			float step = HEADING_COMMAND_ROTATION_RATE * dt;
			if (fabsf(err) <= step) {
				fcStatusData.headingRef = headingCommandTarget;
			} else {
				fcStatusData.headingRef = wrap360(fcStatusData.headingRef + ((err > 0.0f) ? step : -step));
			}
			headingCommandTimer += dt;
			// Done once the reference has arrived and the real nose is on it
			if (fcStatusData.headingRef == headingCommandTarget && fabsf(wrap180(headingCommandTarget - sensorAttitudeData.heading)) <= HEADING_COMMAND_DONE_THRESHOLD) {
				fcStatusData.isHeadingChangeModeActive = 0;
			} else if (headingCommandTimer >= headingCommandTimeout) {
				// The nose never arrived (wind, saturated yaw loop, bad heading): do not hold the mission
				fcStatusData.isHeadingChangeModeActive = 0;
			}
		}
	}
}

uint8_t isHeadingCommandComplete(void) {
	return !fcStatusData.isHeadingChangeModeActive;
}
