#include "LandingContactHelper.h"

#include <math.h>
#include <sys/_stdint.h>
#include "../../../memory/Memory.h"

float landingAzLP = 0.0f;
uint8_t landingAzLPInit = 0;
float landingSpikeTimer = 0.0f;
float landingGateTimer = 0.0f;
uint8_t landingContactLatched = 0;
float landingActiveTimer = 0;

void landingContactReset(void) {
	landingAzLPInit = 0;
	landingSpikeTimer = 0.0f;
	landingGateTimer = 0.0f;
	landingContactLatched = 0;
}

__ATTR_ITCM_TEXT
uint8_t landingContactUpdate(float dt, float azLinear, float vz, float effectiveThrottle, float liftOffThrottle, uint8_t landingActive) {
	if (!landingActive || dt <= 0.0f) {
		landingContactReset();
		return 0;
	}
	if (landingContactLatched) {
		return 1;
	}
	landingActiveTimer += dt;
	if (!landingAzLPInit) {
		landingAzLP = azLinear;
		landingAzLPInit = 1;
	}
	landingAzLP += (dt / (LANDING_CONTACT_AZ_LP_TAU_S + dt)) * (azLinear - landingAzLP);
	float azHp = LANDING_CONTACT_AZ_SIGN * (azLinear - landingAzLP);
	if (azHp > LANDING_CONTACT_AZ_SPIKE_THRESH) {
		landingSpikeTimer = LANDING_CONTACT_SPIKE_HOLD_S;
	} else if (landingSpikeTimer > 0.0f) {
		landingSpikeTimer -= dt;
	}
	uint8_t spikeRecent = (landingSpikeTimer > 0.0f);
	uint8_t gateOk = (effectiveThrottle <= liftOffThrottle + LANDING_CONTACT_THROTTLE_MARGIN) && (fabsf(vz) < LANDING_CONTACT_VZ_MAX);
	if (gateOk) {
		landingGateTimer += dt;
	} else {
		landingGateTimer = 0.0f;
	}
	float required = spikeRecent ? LANDING_CONTACT_DWELL_SPIKE_S : LANDING_CONTACT_DWELL_SLOW_S;
	if (landingGateTimer >= required && landingActiveTimer >= LANDING_CONTACT_MIN_ACTIVE_S) {
		landingContactLatched = 1;
	}
	return landingContactLatched;
}
