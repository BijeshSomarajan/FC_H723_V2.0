#ifndef SRC_FC_MANAGERS_ALTITUDE_ALTITUDEMANAGER_H_
#define SRC_FC_MANAGERS_ALTITUDE_ALTITUDEMANAGER_H_

#include <sys/_stdint.h>

uint8_t initAltitudeManager(void);
void doAltitudeManagement(void);
void resetAltitudeManager(void);

//Baro reading frequency
#define ALTITUDE_SENSOR_READ_FREQUENCY 200.0f
#define ALTITUDE_SENSOR_READ_PERIOD 1.0f/ALTITUDE_SENSOR_READ_FREQUENCY

#define ALTITUDE_MANAGEMENT_TASK_FREQUENCY 1000
#define ALTITUDE_MANAGEMENT_TASK_PERIOD 1.0f/ALTITUDE_MANAGEMENT_TASK_FREQUENCY

#define ALTITUDE_MANAGEMENT_VEL_TASK_FREQUENCY 800
#define ALTITUDE_MANAGEMENT_VEL_TASK_PERIOD 1.0f/ALTITUDE_MANAGEMENT_VEL_TASK_FREQUENCY

#define ALTITUDE_MANAGEMENT_ALT_TASK_FREQUENCY 100
#define ALTITUDE_MANAGEMENT_ALT_TASK_PERIOD 1.0f/ALTITUDE_MANAGEMENT_ALT_TASK_FREQUENCY

//Lift Off throttle and Throttle LPF settings
#define ALT_MGR_DEFAULT_LIFTOFF_THROTTLE 300

//Max permissible throttle
#define ALT_MGR_MAX_PERMISSIBLE_THROTTLE   RC_CHANNEL_MIN_VALUE + ALT_MGR_MAX_PERMISSIBLE_THROTTLE_DELTA
#define ALT_MGR_ALT_SPEED_GAIN_DEFAULT  0.35f //meter per second
#define ALT_MGR_ALT_PRE_LIFTOFF_SPEED_FACTOR  2.0f

#define ALT_MGR_THROTTLE_CONTROL_LPF_FREQUENCY 20.0f //The frequency at which the overall throttle is meadured to create the baseline.


/* --------------------------------------------------------------------------
 * Hover-throttle learner
 * --------------------------------------------------------------------------
 * Liftoff throttle is measured in ground effect and does not track battery
 * sag, so it is used only as the SEED. In flight the true hover throttle is
 * learned from the actual mixed throttle whenever the vehicle is essentially
 * not climbing and not heavily tilted.
 *
 * IMPORTANT: the learner reads controlData.throttleControl, which INCLUDES
 * tiltCompThDelta and posBrakeCompThDelta. The lift-factor gate below is what
 * keeps tilt compensation out of the learned hover value - it is NOT
 * redundant with the velocity gate. Do not remove it.
 */
// [1] learn in flight | [0] stay on the liftoff seed forever
#define ALT_CONTROL_HOVER_LEARN_ENABLED        1
// Learner time constant, s. Long: this is a slow trim, not a tracker.
#define ALT_CONTROL_HOVER_LEARN_TAU            8.0f
// Only learn when |zVelocity| is below this (m/s) - i.e. actually hovering.
#define ALT_CONTROL_HOVER_LEARN_VEL_MAX        0.25f
// Only learn when tilt lift factor cos(pitch)*cos(roll) is above this
// (~cos(12deg)); tilted flight needs extra throttle that is NOT hover thrust.
#define ALT_CONTROL_HOVER_LEARN_LIFT_MIN       0.978f
// Sanity band around the liftoff seed - the learner may never wander outside.
#define ALT_CONTROL_HOVER_LEARN_MIN_RATIO      0.60f
#define ALT_CONTROL_HOVER_LEARN_MAX_RATIO      1.60f

// =============================================================================
// Tilt compensation
// =============================================================================
// Compensates for the additional thrust required when the aircraft is tilted.
//
// The compensation is based on hoverThrottle rather than the instantaneous
// altitude-controller throttle output. This keeps tilt compensation decoupled
// from the altitude control loop and avoids feeding controller output back into
// the compensation itself.
//
// Compensation is enabled only above MIN_ANGLE and smoothly fades out when
// returning toward level flight. MAX_ANGLE limits the angle used for the
// compensation calculation. MAX_LIMIT provides an additional safety cap.
//
// Typical cruise tilt is around 5 degrees, so a 3-degree activation threshold
// provides a small deadband for normal attitude corrections while still
// allowing compensation during forward cruise.
// =============================================================================

#define ALT_MGR_TILT_COMP_MIN_ANGLE        0.5f    // Start compensation above this tilt angle (degrees)
#define ALT_MGR_TILT_COMP_MAX_ANGLE        45.0f   // Maximum tilt angle considered for compensation (degrees)
#define ALT_MGR_TILT_COMP_TAU_RISE         0.001f    // Rise time constant; allows compensation to build quickly
#define ALT_MGR_TILT_COMP_TAU_FADE         0.1f    // Fade time constant; removes compensation gradually
#define ALT_MGR_TILT_COMP_MAX_LIMIT        50.0f   // Maximum allowed tilt compensation throttle contribution
#define ALT_MGR_TILT_COMP_GAIN             2.0f    // Overall compensation gain; 1.0 = full calculated compensation
#define ALT_MGR_THROTTLE_THRESHOLD_PERIOD 0.80f

#define ALT_MGR_MAX_ALT_DELTA 2.0f //This is a safety net , max alt delta in normal flying is expected to be within this limit.
/* Autolanding configuration */
#define ALT_MGR_ALT_LANDING_PULSE_INACTIVE_PERIOD       0.65f
#define ALT_MGR_ALT_LANDING_PULSE_ACTIVE_PERIOD         1.5f
#define ALT_MGR_ALT_LANDING_STICK_COMMAND               200

typedef enum {
	ALT_HOLD_STATE_IDLE = 0, ALT_HOLD_STATE_BRAKING, ALT_HOLD_STATE_LOCKED
} ALTITUDE_MGR_STATE;

#define ALTITUDE_MGR_ALT_HOLD_BRAKE_MIN_SPEED   0.2f
#define ALTITUDE_MGR_ALT_HOLD_BRAKE_REF_LPF_FREQ   0.65f
#define ALTITUDE_MGR_ALT_HOLD_BRAKE_MAX_PERIOD   15.0f

//Uses only the Alt Vel loop , Alt ref will be updated continously , This works well , dont turn it off
#define ALT_CONTROL_SKIP_ALT_REF_FOR_NON_NAV_MODE   1

#endif
