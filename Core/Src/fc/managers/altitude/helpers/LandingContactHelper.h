#ifndef SRC_FC_MANAGERS_ALTITUDE_HELPERS_LANDINGCONTACTHELPER_H_
#define SRC_FC_MANAGERS_ALTITUDE_HELPERS_LANDINGCONTACTHELPER_H_
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Landing contact detection (Z accel spike + throttle + vz stall)     */
/* ------------------------------------------------------------------ */

#define LANDING_CONTACT_AZ_SIGN              1.0f    // +1 / -1: sign of the touchdown spike in your az signal. VERIFY IN LOGS
#define LANDING_CONTACT_AZ_LP_TAU_S          0.5f    // slow mean tracker for high-pass [s]
#define LANDING_CONTACT_AZ_SPIKE_THRESH      2.0f    // m/s^2 deviation from mean counted as a spike
#define LANDING_CONTACT_SPIKE_HOLD_S         0.30f   // spike remembered this long [s]
#define LANDING_CONTACT_THROTTLE_MARGIN      20.0f   // throttle above liftoff throttle still counted as "close"
#define LANDING_CONTACT_VZ_MAX               0.20f   // m/s, |vz| below this = stalled
#define LANDING_CONTACT_DWELL_SPIKE_S        0.10f   // gate must hold this long if a spike was seen [s]
#define LANDING_CONTACT_DWELL_SLOW_S         0.35f   // gate must hold this long with no spike [s]
#define LANDING_CONTACT_MIN_ACTIVE_S         1.0f // s, no latch until landing has been active this long

void landingContactReset(void);
uint8_t landingContactUpdate(float dt, float azLinear, float vz, float throttlePercent, float liftOffThrottlePercent, uint8_t landingActive);

#endif /* SRC_FC_MANAGERS_ALTITUDE_HELPERS_LANDINGCONTACTHELPER_H_ */
