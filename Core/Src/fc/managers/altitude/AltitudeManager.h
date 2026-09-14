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

#define ALTITUDE_MANAGEMENT_ACC_TASK_FREQUENCY 1000
#define ALTITUDE_MANAGEMENT_ACC_TASK_PERIOD 1.0f/ALTITUDE_MANAGEMENT_ACC_TASK_FREQUENCY

#define ALTITUDE_MANAGEMENT_VEL_TASK_FREQUENCY 800
#define ALTITUDE_MANAGEMENT_VEL_TASK_PERIOD 1.0f/ALTITUDE_MANAGEMENT_VEL_TASK_FREQUENCY

#define ALTITUDE_MANAGEMENT_ALT_TASK_FREQUENCY 100
#define ALTITUDE_MANAGEMENT_ALT_TASK_PERIOD 1.0f/ALTITUDE_MANAGEMENT_ALT_TASK_FREQUENCY

//Lift Off throttle and Throttle LPF settings
#define ALT_MGR_DEFAULT_LIFTOFF_THROTTLE 300

//Max permissible throttle
#define ALT_MGR_MAX_PERMISSIBLE_THROTTLE   RC_CHANNEL_MIN_VALUE + ALT_MGR_MAX_PERMISSIBLE_THROTTLE_DELTA
#define ALT_MGR_ALT_SPEED_GAIN_DEFAULT  0.35f //meter per second

#define ALT_MGR_MAX_ALT_DELTA 2.5f //Mts
#define ALT_MGR_THROTTLE_AVERAGING_LPF_FREQUENCY 20.0f//5.0f

//Uses only the Alt Vel loop , Alt ref will be updated continously , This works well , dont turn it off
#define ALT_CONTROL_SKIP_ALT_REF_FOR_NON_NAV_MODE   1

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

#define ALT_MGR_TILT_COMP_ENABLED          1       // Enable tilt-based throttle compensation
#define ALT_MGR_TILT_COMP_MIN_ANGLE        1.5f    // Start compensation above this tilt angle (degrees)
#define ALT_MGR_TILT_COMP_MAX_ANGLE        45.0f   // Maximum tilt angle considered for compensation (degrees)
#define ALT_MGR_TILT_COMP_TAU_RISE         0.001f    // Rise time constant; allows compensation to build quickly
#define ALT_MGR_TILT_COMP_TAU_FADE         0.1f    // Fade time constant; removes compensation gradually
#define ALT_MGR_TILT_COMP_MAX_LIMIT        75.0f   // Maximum allowed tilt compensation throttle contribution
#define ALT_MGR_TILT_COMP_GAIN             1.0f    // Overall compensation gain; 1.0 = full calculated compensation

// --- Alt Control Settings ---
// This threshold defines the stick deflection beyond which the altitude control will start to attenuate. Adjust based on testing.
#define ALT_MGR_ALT_CONTROL_SETTING_LATERAL_DT_THRESHOLD 0.4f
// In terms of throttle delta per second. This is used to calculate the rate of stick movement.
#define ALT_MGR_ALT_CONTROL_SETTING_THROTTLE_RATE_MAX 10.0f
// This scaler converts the raw throttle rate into a 0.0 to 1.0 range for gain attenuation. Adjust as needed based on testing.
#define ALT_MGR_ALT_CONTROL_STICK_RATE_SCALER 0.01f
// 1.0f = PID goes to 0 at max stick. 0.7f = PID keeps 30% authority at max stick.
#define ALT_MGR_ALT_CONTROL_STICK_ATTENUATION_GAIN 1.0f
// Higher = more aggressive PID ducking on fast moves
#define ALT_MGR_THROTTLE_RATE_ATTENUATION_GAIN 0.80f

#define ALT_MGR_THROTTLE_THRESHOLD_PERIOD 0.80f

/*
 * If the drone feels "mushy" for too long after centering: Reduce MP_TAU to 0.3s. This will make the drone "snap" into hover faster.
 * If the drone "bobs" up and down when you stop a climb: Increase RI_TAU to 1.2s.
 * This further slows down the re-engagement of the vertical weight compensation (the I-term), making the hand-off even softer.
 */
#define ALT_MGR_ALT_CONTROL_SETTING_MASTER_P_GAIN  0.0f  // Light feel
#define ALT_MGR_ALT_CONTROL_SETTING_MP_TAU         0.01f // Fast lock

#define ALT_MGR_ALT_CONTROL_SETTING_RATE_I_GAIN    0.5f
#define ALT_MGR_ALT_CONTROL_SETTING_RI_TAU         0.5f  // Smooth handover

#define ALT_MGR_ALT_CONTROL_SETTING_RATE_P_GAIN    0.1f
#define ALT_MGR_ALT_CONTROL_SETTING_RP_TAU         0.01f

#define ALT_MGR_ALT_CONTROL_SETTING_ACC_P_GAIN     0.1f // Zero fighting
#define ALT_MGR_ALT_CONTROL_SETTING_AP_TAU         0.01f

#define ALT_MGR_ALT_CONTROL_SETTING_DOB_GAIN       0.0f
#define ALT_MGR_ALT_CONTROL_SETTING_DOB_TAU        0.3f // Smooth handover

/* Autolanding configuration */
#define ALT_MGR_ALT_LANDING_PULSE_INACTIVE_PERIOD       0.75f
#define ALT_MGR_ALT_LANDING_PULSE_ACTIVE_PERIOD         0.75f
#define ALT_MGR_ALT_LANDING_STICK_COMMAND               150

#endif
