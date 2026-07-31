
#ifndef SRC_FC_MANAGERS_POSITION_POSITIONMANAGER_H_
#define SRC_FC_MANAGERS_POSITION_POSITIONMANAGER_H_

#include "../position/common/PositionCommon.h"

/*
* ============================================================================
 * Local Navigation Coordinate Frame
 * ============================================================================
 *
 *             +X (North)
 *                  ^
 *                  |
 *                  |
 * -Y (West) <------+------> +Y (East)
 *                  |
 *                  |
 *                  v
 *             -X (South)
 */

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

#define POSITION_MGR_X_EST_OUTPUT_ACC_DEADBAND        0.0f    // m/s²
#define POSITION_MGR_Y_EST_OUTPUT_ACC_DEADBAND        0.0f    // m/s²
#define POSITION_MGR_Z_EST_OUTPUT_ACC_DEADBAND        0.020f    // m/s²

#define POSITION_MGR_X_EST_OUTPUT_VEL_DEADBAND        0.0f    // m/s
#define POSITION_MGR_Y_EST_OUTPUT_VEL_DEADBAND        0.0f    // m/s
#define POSITION_MGR_Z_EST_OUTPUT_VEL_DEADBAND        0.01f    // m/s

#define POSITION_MGR_X_EST_OUTPUT_ACC_LPF_FREQ        60.00f
#define POSITION_MGR_Y_EST_OUTPUT_ACC_LPF_FREQ        60.00f
#define POSITION_MGR_Z_EST_OUTPUT_ACC_LPF_FREQ        20.00f

#define POSITION_MGR_X_EST_OUTPUT_VEL_LPF_FREQ        80.00f
#define POSITION_MGR_Y_EST_OUTPUT_VEL_LPF_FREQ        80.00f
#define POSITION_MGR_Z_EST_OUTPUT_VEL_LPF_FREQ        30.00f

#define POSITION_MGR_X_EST_OUTPUT_VEL_MAX             16.0f   // m/s
#define POSITION_MGR_Y_EST_OUTPUT_VEL_MAX             16.0f   // m/s
#define POSITION_MGR_Z_EST_OUTPUT_VEL_MAX             16.0f   // m/s

#define POSITION_MGR_X_EST_OUTPUT_ACC_MAX             150.0f  // m/s² (~15G)
#define POSITION_MGR_Y_EST_OUTPUT_ACC_MAX             150.0f  // m/s²
#define POSITION_MGR_Z_EST_OUTPUT_ACC_MAX             200.0f  // m/s² (~20G)

// =============================================================================
// 5. LOITER BRAKING & SETTLING CONFIGURATIONS
// =============================================================================
#define POSITION_MGR_POS_HOLD_BRAKE_ACTIVE_PERIOD      2.0f   // was 0.15f
#define POSITION_MGR_POS_HOLD_BRAKE_SETTLING_PERIOD    0.5f   // was 0.25f
#define POSITION_MGR_POS_HOLD_BRAKE_STRENGTH           0.25f  //Was 0.5f
#define POSITION_MGR_POS_HOLD_BRAKE_MAX_VELOCITY       1.2f
#define POSITION_MGR_POS_HOLD_BRAKE_MAX_GROUND_SPEED   0.15f
#define POSITION_MGR_POS_HOLD_BRAKE_RATE_PI_GAIN       1.0f
#define POSITION_MGR_POS_HOLD_SETTLING_TIMEOUT         3.0f   // hard cap
#define POSITION_MGR_POS_HOLD_BRAKE_DECEL              0.275   // 0.30f


typedef enum {
	POS_HOLD_STATE_IDLE = 0, POS_HOLD_STATE_BRAKING, POS_HOLD_STATE_SETTLING, POS_HOLD_STATE_LOCKED
} POSITION_MGR_STATE;

uint8_t initPositionManager(void);
void doPositionManagement(void);
void resetPositionManager(void);

#endif /* SRC_FC_MANAGERS_POSITION_POSITIONMANAGER_H_ */
