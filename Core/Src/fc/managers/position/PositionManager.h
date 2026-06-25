
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

#define POSITION_MANAGEMENT_OFLOW_READ_FREQUENCY 100.0f
#define POSITION_MANAGEMENT_OFLOW_READ_PERIOD    (1.0f / POSITION_MANAGEMENT_OFLOW_READ_FREQUENCY)

// =============================================================================
// SENSOR DEADBANDS , SATURATION LIMITS & LPFs
// =============================================================================
//CAUTION - Setting it to non zero will make EKF blind.
#define POSITION_MGR_X_EST_INPUT_ACC_DEADBAND         0.0f    // m/s²
#define POSITION_MGR_Y_EST_INPUT_ACC_DEADBAND         0.0f    // m/s²
#define POSITION_MGR_Z_EST_INPUT_ACC_DEADBAND         0.0f    // m/s²

#define POSITION_MGR_X_EST_OUTPUT_ACC_DEADBAND        0.012f    // m/s²
#define POSITION_MGR_Y_EST_OUTPUT_ACC_DEADBAND        0.012f    // m/s²
#define POSITION_MGR_Z_EST_OUTPUT_ACC_DEADBAND        0.020f    // m/s²

#define POSITION_MGR_X_EST_OUTPUT_VEL_DEADBAND        0.008f    // m/s
#define POSITION_MGR_Y_EST_OUTPUT_VEL_DEADBAND        0.008f    // m/s
#define POSITION_MGR_Z_EST_OUTPUT_VEL_DEADBAND        0.012f    // m/s

#define POSITION_MGR_X_EST_OUTPUT_ACC_LPF_FREQ        30.00f
#define POSITION_MGR_Y_EST_OUTPUT_ACC_LPF_FREQ        30.00f
#define POSITION_MGR_Z_EST_OUTPUT_ACC_LPF_FREQ        30.00f

#define POSITION_MGR_X_EST_OUTPUT_VEL_LPF_FREQ        60.00f
#define POSITION_MGR_Y_EST_OUTPUT_VEL_LPF_FREQ        60.00f
#define POSITION_MGR_Z_EST_OUTPUT_VEL_LPF_FREQ        60.00f

#define POSITION_MGR_X_EST_OUTPUT_VEL_MAX             16.0f   // m/s
#define POSITION_MGR_Y_EST_OUTPUT_VEL_MAX             16.0f   // m/s
#define POSITION_MGR_Z_EST_OUTPUT_VEL_MAX             16.0f   // m/s

#define POSITION_MGR_X_EST_OUTPUT_ACC_MAX             150.0f  // m/s² (~15G)
#define POSITION_MGR_Y_EST_OUTPUT_ACC_MAX             150.0f  // m/s²
#define POSITION_MGR_Z_EST_OUTPUT_ACC_MAX             200.0f  // m/s² (~20G)

// =============================================================================
// 5. GNSS EKF MEASUREMENT TRUST (Tuned for Standard GNSS - POST-FIX TUNE)
// =============================================================================
// Define the number of samples to capture (e.g., 50 samples)
#define POSITION_MGR_HOME_POS_STAB_COUNT   50

// =============================================================================
// 6. LOITER BRAKING & SETTLING CONFIGURATIONS
// =============================================================================
#define POSITION_MGR_POS_HOLD_BRAKE_ACTIVE_PERIOD      0.15f
#define POSITION_MGR_POS_HOLD_BRAKE_SETTLING_PERIOD    0.25f
#define POSITION_MGR_POS_HOLD_BRAKE_STRENGTH           0.5f
#define POSITION_MGR_POS_HOLD_BRAKE_MAX_VELOCITY       4.0f
#define POSITION_MGR_POS_HOLD_BRAKE_MAX_GROUND_SPEED   0.15f
#define POSITION_MGR_POS_HOLD_BRAKE_RATE_PI_GAIN       1.0f
#define POSITION_MGR_POS_HOLD_BRAKE_THROTTLE_GAIN      6.0f
#define POSITION_MGR_POS_HOLD_BRAKE_THROTTLE_LIMIT     100.0f

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
void resetPositionManager(void);

#endif /* SRC_FC_MANAGERS_POSITION_POSITIONMANAGER_H_ */
