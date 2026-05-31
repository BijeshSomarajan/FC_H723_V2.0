#ifndef SRC_FC_MANAGERS_POSITION_POSITIONMANAGER_H_
#define SRC_FC_MANAGERS_POSITION_POSITIONMANAGER_H_

#include "common/PositionCommon.h"

// =============================================================================
// 1. SCHEDULER & EXECUTION TIMING
// =============================================================================
/* Higher: More precise EKF predictions, increases CPU load. Lower: Saves CPU, but degrades tracking accuracy during fast maneuvers. */
#define POSITION_MANAGEMENT_TASK_FREQUENCY             1000
#define POSITION_MANAGEMENT_TASK_PERIOD                (1.0f / POSITION_MANAGEMENT_TASK_FREQUENCY)

/* Higher: Crisper, stiffer velocity corrections; if too high, amplifies sensor noise. Lower: Softer, more sluggish velocity tracking. */
#define POSITION_MANAGEMENT_RATE_CONTROL_FREQUENCY     300
#define POSITION_MANAGEMENT_RATE_CONTROL_PERIOD        (1.0f / POSITION_MANAGEMENT_RATE_CONTROL_FREQUENCY)

/* Higher: Tighter coordinate holding; must keep a healthy gap below rate frequency to avoid control loop fighting. Lower: Loose position holding. */
#define POSITION_MANAGEMENT_POSITION_CONTROL_FREQUENCY 75
#define POSITION_MANAGEMENT_POSITION_CONTROL_PERIOD    (1.0f / POSITION_MANAGEMENT_POSITION_CONTROL_FREQUENCY)

// =============================================================================
// 2. CONVERSION & SCALE GAINS
// =============================================================================
#define POSITION_MGR_X_ACC_OUTPUT_GAIN                 1.0f    // Scaling factor for X IMU acceleration. Do not change unless altering physical units.
#define POSITION_MGR_Y_ACC_OUTPUT_GAIN                 1.0f    // Scaling factor for Y IMU acceleration. Do not change unless altering physical units.
#define POSITION_MGR_Z_ACC_OUTPUT_GAIN                 100.0f  // Converts m/s² to cm/s² for vertical estimation scales.

#define POSITION_MGR_X_POS_OUTPUT_GAIN                 1.0f    // Scaling factor for raw X coordinates (meters). Leave at 1.0f.
#define POSITION_MGR_Y_POS_OUTPUT_GAIN                 1.0f    // Scaling factor for raw Y coordinates (meters). Leave at 1.0f.
#define POSITION_MGR_Z_POS_OUTPUT_GAIN                 100.0f  // Converts vertical position from meters to centimeters.

/* Higher: Allows aggressive tilt angles to fight heavy wind or brake violently. Lower: Safer flight envelope, but drone will drift in stiff breezes. */
#define POSITION_MGR_MAX_POS_COMMAND                   30.0f   // Maximum tilt angle command allowed by position loop (Degrees)

// =============================================================================
// 3. LOW-PASS FILTER CUTOFF FREQUENCIES (Hz)
// =============================================================================
/* Higher: Less control phase lag, sharper response; exposes loop to high-frequency frame vibrations. Lower: Smoother commands, but adds lag that causes overshoot. */
#define POSITION_MGR_X_VEL_LPF_FREQ                    20.00f
#define POSITION_MGR_Y_VEL_LPF_FREQ                    20.00f
#define POSITION_MGR_Z_VEL_LPF_FREQ                    20.00f

/* Higher: Fast EKF response to abrupt acceleration changes; passes more structural motor noise. Lower: Cleans up signal, but introduces phase delay to estimation. */
#define POSITION_MGR_X_ACC_LPF_FREQ                    30.00f
#define POSITION_MGR_Y_ACC_LPF_FREQ                    30.00f
#define POSITION_MGR_Z_ACC_LPF_FREQ                    30.00f

// =============================================================================
// 4. SENSOR DEADBANDS & SATURATION LIMITS
// =============================================================================
/* Higher: Suppresses frame resonance feedback. Lower: Captures raw micro-accelerations; if too low, integrates noise into control loops. */
#define POSITION_MGR_X_ACC_DEADBAND                    0.0f    // m/s²
#define POSITION_MGR_Y_ACC_DEADBAND                    0.0f    // m/s²
#define POSITION_MGR_Z_ACC_DEADBAND                    1.0f    // m/s²

/* Higher: EKF ignores subtle sensor changes, creating lag. Lower: EKF captures micro-movements, but risks integrating sensor bias during steady hover. */
#define POSITION_MGR_X_ESTIMATION_ACC_DEADBAND         0.001f    // m/s²
#define POSITION_MGR_Y_ESTIMATION_ACC_DEADBAND         0.001f    // m/s²
#define POSITION_MGR_Z_ESTIMATION_ACC_DEADBAND         0.1f    // m/s²

/* Higher: Prevents tiny tracking errors from translating to motor twitches. Lower: Tighter control near zero velocity, but can cause micro-oscillations. */
#define POSITION_MGR_X_VEL_DEADBAND                    0.0f   // m/s
#define POSITION_MGR_Y_VEL_DEADBAND                    0.0f   // m/s
#define POSITION_MGR_Z_VEL_DEADBAND                    1.0f    // cm/s

/* Safety ceilings. Limits maximum values processed by low pass filters to prevent math blowups from sensor glitches. */
#define POSITION_MGR_X_VEL_MAX                         5.0f   // m/s
#define POSITION_MGR_Y_VEL_MAX                         5.0f   // m/s
#define POSITION_MGR_Z_VEL_MAX                         500.0f  // cm/s

#define POSITION_MGR_X_ACC_MAX                         50.0f   // m/s²
#define POSITION_MGR_Y_ACC_MAX                         50.0f   // m/s²
#define POSITION_MGR_Z_ACC_MAX                         500.0f  // cm/s²

// =============================================================================
// 5. GNSS EKF MEASUREMENT TRUST (Tuned for Standard GNSS - POST-FIX TUNE)
// =============================================================================
#define POSITION_MGR_Z_ENABLE_DYNAMIC_R                1
#define POSITION_MGR_VENTURI_ESTIMATE_ENABLED          1
#define POSITION_MGR_XY_POS_DYNAMIC_R_BASE             POS_EKF_X_R_MEAS // 0.06f

/* GPS Position Gating & Scaling */
#define POSITION_MGR_GNSS_POS_HACC_SCALE               1.2f
#define POSITION_MGR_GNSS_POS_HACC_MIN                 0.6f
#define POSITION_MGR_GNSS_POS_R_MAX                    50.0f
/* GPS Velocity Damping */
#define POSITION_MGR_XY_VEL_UPDATE_DAMP_STRENGTH       0.12f //0.05f
#define POSITION_MGR_XY_VEL_RESET_DAMP_STRENGTH        0.2f

/* GPS Velocity Gating & Scaling */
#define POSITION_MGR_GNSS_VEL_SACC_SCALE               0.1f
#define POSITION_MGR_GNSS_VEL_SACC_MIN                 0.1f
#define POSITION_MGR_GNSS_VEL_R_MAX                    10.0f
#define POSITION_MGR_GNSS_VEL_DEADBAND                 0.0f

// =============================================================================
// 6. LOITER BRAKING & SETTLING CONFIGURATIONS
// =============================================================================
/* Higher: Allows faster maximum counter-flight velocities when stopping. Lower: Gentler, smoother deceleration profiles over longer physical distances. */
#define POSITION_MGR_POS_HOLD_BRAKE_MAX_VELOCITY       4.0f
/* Higher: Snappier, more rigid pitch/roll corrections while slowing down. Lower: Soft, "spongey" braking feel that risks slipping past target bounds. */
#define POSITION_MGR_POS_HOLD_BRAKE_RATE_PI_GAIN       1.0f
/* Higher: Sharp, immediate tilt opposition the instant sticks center. Lower: Glassy, delayed onset of the initial braking sequence. */
#define POSITION_MGR_POS_HOLD_MIN_BRAKE_STRENGTH       0.1f
/* Higher: Massive deceleration authority during high-speed stops. Lower: Caps braking response, preventing hard pitch-backs but risking overshoot. */
#define POSITION_MGR_POS_HOLD_MAX_BRAKE_LIMIT          1.00f
/* Higher: Dynamically commands a much stiffer brake angle if exiting a high-speed sprint versus a slow hover. Lower: Uniform braking response across all entry speeds. */
#define POSITION_MGR_BRAKE_VEL_SCALE_GAIN              0.15f
/* Higher: More aggressive velocity integration , chances of overshooting. Lower: less aggressive velocity integration , chances snapback */
#define POSITION_MGR_BRAKE_POS_EST_VEL_SCALE_GAIN       0.25f
/* Higher: Gives sluggish or heavy aircraft more time to shed velocity. Lower: Sharp safety cutout; cuts braking command early even if the drone is still sliding. */
#define POSITION_MGR_BRAKE_RAMP_DURATION_SEC           0.5f
/* Higher: Exits braking state earlier, handing off to settling while still moving slightly. Lower: Demands a near-perfect halt before transitioning states. */
#define POSITION_MGR_POS_HOLD_BRAKE_MAX_GROUND_SPEED   0.08f
/* Higher: Swallows sensor jitter early, but allows small residual drifts. Lower: Continuous active braking down to zero; can cause micro-stuttering if EKF has noise. */
#define POSITION_MGR_POS_HOLD_BRAKE_VEL_DEADBAND       0.03f
/* Higher: Gives aerodynamic wake and frame inertia more time to stabilize before locking spatial coordinates. Lower: Snaps position lock instantly, risking an offset lock. */
#define POSITION_MGR_POS_HOLD_POST_BRAKE_SETTLING_PERIOD 0.3f
/* Higher: Swallows EKF/GPS velocity jitter earlier, but leaves minor residual drift. Lower: Keeps braking active down to a near-perfect halt. */
#define POSITION_MGR_BRAKE_TERMINAL_VEL_DEADBAND       0.03f   // m/s

#define POSITION_MGR_POS_HOLD_BALLISTIC_SCALE                1.2f
#define POSITION_MGR_POS_HOLD_EKF_LAG_SEC                    0.05f
#define POSITION_MGR_POS_HOLD_NATURAL_DECEL                  7.5f
#define POSITION_MGR_POS_HOLD_MAX_BRAKE_OFFSET               3.0f/10
#define POSITION_MGR_POS_HOLD_BRAKE_REF_EST_VELOCITY_GAIN    0.33f
#define POSITION_MGR_POS_HOLD_BRAKE_ACTIVE_PERIOD            0.75f // 0.85f
#define POSITION_MGR_POS_HOLD_BRAKE_SETTLING_PERIOD          0.25f // 1.0f
#define POSITION_MGR_POS_HOLD_BRAKE_STRENGTH                 1.25f //1.5f

// Define the number of samples you want to capture (e.g., 50 samples)
#define POSITION_MGR_HOME_SAMPLE_TARGET   50

// =============================================================================
// 7. RETURN TO HOME (RTH) NAVIGATION PROFILE
// =============================================================================
/* Higher: High-velocity transit back to home base. Lower: Safe, deliberate, conservative RTH cruising speed (saves battery but fights wind poorly). */
#define POSITION_MGR_RTH_CRUISE_SPEED                  15.0f

/* Higher: Starts a gradual, smooth slowing down sequence far away from home. Lower: Screams toward home at full cruise speed until the last second, risking large overshoots. */
#define POSITION_MGR_RTH_NEAR_HOME_RADIUS              1.5f

/* Higher: Declares success early even if slightly offset. Lower: Drone will circle or hunt endlessly above home if GPS noise prevents entering a tiny radius. */
#define POSITION_MGR_RTH_HOME_RADIUS                   0.6f

/* Higher: Aggressive velocity changes and abrupt cruise adaptation. Lower: Silky, gentle, linear speed ramps that protect physical battery voltage. */
#define POSITION_MGR_RTH_MAX_ACCEL                     5.0f

/* Higher: Prolongs position hover stabilization before embarking on the RTH vector path. Lower: Darts home the exact millisecond RTH is commanded. */
#define POSITION_MGR_RTH_SETTLING_PERIOD               0.5f

/* Higher: Drone must stay inside home radius longer to verify a stable hover before triggering landing phase. Lower: Rapid completion trigger. */
#define POSITION_MGR_RTH_COMPLETE_PERIOD               4.0f

/* Higher: Allows completion trigger while drone is still sliding or oscillating over home. Lower: Strict hover requirement to complete transit safely. */
#define POSITION_MGR_RTH_COMPLETE_MAX_GROUND_SPEED    0.4f

typedef enum {
	POS_HOLD_STATE_IDLE = 0, POS_HOLD_STATE_BRAKING, POS_HOLD_STATE_SETTLING, POS_HOLD_STATE_LOCKED
} POSITION_MGR_STATE;

uint8_t initPositionManager(void);
void doPositionManagement(void);
void updatePositionManagerZPosition(float zPos, float dt);
void updatePositionManagerXYPosition(float xPos, float yPos, float dt);
void resetPositionManager(void);

#endif /* SRC_FC_MANAGERS_POSITION_POSITIONMANAGER_H_ */
