#include "LandingContactHelper.h"

#include <math.h>
#include <sys/_stdint.h>
#include "../../../memory/Memory.h"

static float landingAzLP = 0.0f;
static uint8_t landingAzLPInit = 0;
static float landingSpikeTimer = 0.0f;
static float landingGateTimer = 0.0f;
static uint8_t landingContactLatched = 0;

void landingContactReset(void) {
	landingAzLPInit = 0;
	landingSpikeTimer = 0.0f;
	landingGateTimer = 0.0f;
	landingContactLatched = 0;
}

/* Call every control/estimator cycle while in the landing phase.
 * azLinear : gravity-removed vertical accel [m/s^2] (IMU earth-linear or EKF bias-corrected)
 * vz       : estimated vertical velocity [m/s]
 * throttle, liftOffThrottle : absolute throttle values, same scale as the state machine
 *            (fcStatusData.currentThrottle, altCommandLiftOffThrottle)
 * Returns 1 once contact is declared (latched until landingContactReset). */
__ATTR_ITCM_TEXT
uint8_t landingContactUpdate(float dt, float azLinear, float vz, float throttle, float liftOffThrottle, uint8_t landingActive) {
	if (!landingActive || dt <= 0.0f) {
		landingContactReset();
		return 0;
	}
	if (landingContactLatched) {
		return 1;
	}
	/* High-pass: deviation from slow mean */
	if (!landingAzLPInit) {
		landingAzLP = azLinear;
		landingAzLPInit = 1;
	}
	landingAzLP += (dt / (LANDING_CONTACT_AZ_LP_TAU_S + dt)) * (azLinear - landingAzLP);
	float azHp = LANDING_CONTACT_AZ_SIGN * (azLinear - landingAzLP);
	/* Spike detect with hold */
	if (azHp > LANDING_CONTACT_AZ_SPIKE_THRESH) {
		landingSpikeTimer = LANDING_CONTACT_SPIKE_HOLD_S;
	} else if (landingSpikeTimer > 0.0f) {
		landingSpikeTimer -= dt;
	}
	uint8_t spikeRecent = (landingSpikeTimer > 0.0f);
	/* Gate: throttle near liftoff and vertical motion stalled */
	uint8_t gateOk = (throttle <= liftOffThrottle + LANDING_CONTACT_THROTTLE_MARGIN) && (fabsf(vz) < LANDING_CONTACT_VZ_MAX);
	if (gateOk) {
		landingGateTimer += dt;
	} else {
		landingGateTimer = 0.0f;
	}
	float required = spikeRecent ? LANDING_CONTACT_DWELL_SPIKE_S : LANDING_CONTACT_DWELL_SLOW_S;
	if (landingGateTimer >= required) {
		landingContactLatched = 1;
	}
	return landingContactLatched;
}
