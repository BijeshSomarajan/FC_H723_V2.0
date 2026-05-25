#ifndef SRC_FC_MANAGERS_POSITION_POSITIONMANAGER_H_
#define SRC_FC_MANAGERS_POSITION_POSITIONMANAGER_H_
#include "common/PositionCommon.h"

#define POSITION_MANAGEMENT_TASK_FREQUENCY 1000
#define POSITION_MANAGEMENT_TASK_PERIOD 1.0f/POSITION_MANAGEMENT_TASK_FREQUENCY

#define POSITION_MANAGEMENT_RATE_CONTROL_FREQUENCY 300
#define POSITION_MANAGEMENT_RATE_CONTROL_PERIOD 1.0f/POSITION_MANAGEMENT_RATE_CONTROL_FREQUENCY

#define POSITION_MANAGEMENT_POSITION_CONTROL_FREQUENCY 75
#define POSITION_MANAGEMENT_POSITION_CONTROL_PERIOD 1.0f/POSITION_MANAGEMENT_POSITION_CONTROL_FREQUENCY

#define POSITION_MGR_X_ACC_OUTPUT_GAIN 1.0f  // m/sec2
#define POSITION_MGR_Y_ACC_OUTPUT_GAIN 1.0f  // m/sec2
#define POSITION_MGR_Z_ACC_OUTPUT_GAIN 100.0f  // cm/sec2

#define POSITION_MGR_X_POS_OUTPUT_GAIN 1.0f  // m
#define POSITION_MGR_Y_POS_OUTPUT_GAIN 1.0f  // m
#define POSITION_MGR_Z_POS_OUTPUT_GAIN 100.0f  // cm

#define POSITION_MGR_MAX_POS_COMMAND  30.0f // Degrees

#define POSITION_MGR_X_VEL_LPF_FREQ 15.00f
#define POSITION_MGR_Y_VEL_LPF_FREQ 15.00f
#define POSITION_MGR_Z_VEL_LPF_FREQ 20.00f

#define POSITION_MGR_X_ACC_LPF_FREQ 30.00f
#define POSITION_MGR_Y_ACC_LPF_FREQ 30.00f
#define POSITION_MGR_Z_ACC_LPF_FREQ 30.00f

#define POSITION_MGR_X_ACC_DEADBAND 0.0f //0.01f    // Meter/Sec2
#define POSITION_MGR_Y_ACC_DEADBAND 0.0f //0.01f    // Meter/Sec2
#define POSITION_MGR_Z_ACC_DEADBAND 1.0f            // Meter/Sec2

#define POSITION_MGR_X_ESTIMATION_ACC_DEADBAND   0.0f // Meter/Sec2
#define POSITION_MGR_Y_ESTIMATION_ACC_DEADBAND   0.0f // Meter/Sec2
#define POSITION_MGR_Z_ESTIMATION_ACC_DEADBAND   0.1f // Meter/Sec2

#define POSITION_MGR_X_VEL_DEADBAND     0.01f   //0.005f // Meter/Sec
#define POSITION_MGR_Y_VEL_DEADBAND     0.01f   //0.005f // Meter/Sec
#define POSITION_MGR_Z_VEL_DEADBAND     1.0f   // Cm/Sec

#define POSITION_MGR_X_VEL_MAX          50.0f  // M/Sec
#define POSITION_MGR_Y_VEL_MAX          50.0f  // M/Sec
#define POSITION_MGR_Z_VEL_MAX          500.0f // cm/Sec2

#define POSITION_MGR_X_ACC_MAX          50.0f  // M/Sec2
#define POSITION_MGR_Y_ACC_MAX          50.0f  // M/Sec2
#define POSITION_MGR_Z_ACC_MAX          500.0f // Cm/Sec2

#define POSITION_MGR_Z_ENABLE_DYNAMIC_R 1

// --------------------------------------------------
// GPS POSITION MEASUREMENT TRUST (Standard GNSS Tuned)
// --------------------------------------------------
#define POSITION_MGR_XY_POS_DYNAMIC_R_BASE   0.6f   //0.6 - Works
#define POSITION_MGR_GNSS_POS_HACC_SCALE     2.0f   //2.0 - Works
#define POSITION_MGR_GNSS_POS_HACC_MIN       0.8f
#define POSITION_MGR_GNSS_POS_R_MAX          100.0f
// --------------------------------------------------
// GPS VELOCITY MEASUREMENT TRUST (Standard GNSS Tuned)
// --------------------------------------------------
#define POSITION_MGR_XY_VEL_UPDATE_DAMP_STRENGTH   2.0f   // Standard GNSS speeds are derived from Doppler shift
#define POSITION_MGR_XY_VEL_RESET_DAMP_STRENGTH    3.0f
#define POSITION_MGR_GNSS_VEL_SACC_SCALE           2.0f
#define POSITION_MGR_GNSS_VEL_SACC_MIN             0.15f   // Doppler velocity is still highly accurate, floor at 0.2m/s
#define POSITION_MGR_GNSS_VEL_R_MAX                100.0f   // High ceiling to reject sudden speed tracking anomalies
#define POSITION_MGR_GNSS_VEL_DEADBAND             0.025f

//Loiter configurations
#define POSITION_MGR_POS_HOLD_BRAKE_MAX_VELOCITY             15.0f
#define POSITION_MGR_POS_HOLD_BRAKE_RATE_PI_GAIN             1.2f

#define POSITION_MGR_POS_HOLD_BRAKE_REF_EST_VELOCITY_GAIN    0.33f
#define POSITION_MGR_POS_HOLD_BRAKE_MAX_GROUND_SPEED         0.4f
#define POSITION_MGR_POS_HOLD_BRAKE_ACTIVE_PERIOD            0.75f // 0.85f
#define POSITION_MGR_POS_HOLD_BRAKE_SETTLING_PERIOD          0.25f // 1.0f
#define POSITION_MGR_POS_HOLD_BRAKE_STRENGTH                 1.25f //1.5f
#define POSITION_MGR_POS_HOLD_BALLISTIC_SCALE                1.2f

#define POSITION_MGR_POS_HOLD_EKF_LAG_SEC                    0.05f
#define POSITION_MGR_POS_HOLD_NATURAL_DECEL                  7.5f
#define POSITION_MGR_POS_HOLD_MAX_BRAKE_OFFSET               3.0f
//RTH Configurations
#define POSITION_MGR_RTH_CRUISE_SPEED          15.0f   // m/s
#define POSITION_MGR_RTH_NEAR_HOME_RADIUS      1.5f   // meters
#define POSITION_MGR_RTH_HOME_RADIUS           0.6f   // meters
#define POSITION_MGR_RTH_MAX_ACCEL             5.0f   // m/s²
#define POSITION_MGR_RTH_SETTLING_PERIOD       0.5f   // m/s²
#define POSITION_MGR_RTH_COMPLETE_PERIOD       4.0f
#define POSITION_MGR_RTH_COMPLETE_MAX_GROUND_SPEED    0.4f

#define POSITION_MGR_VENTURI_ESTIMATE_ENABLED 1

typedef enum {
	POS_HOLD_STATE_IDLE = 0, POS_HOLD_STATE_BRAKING, POS_HOLD_STATE_SETTLING, POS_HOLD_STATE_LOCKED
} POSITION_MGR_STATE;

uint8_t initPositionManager(void);
void doPositionManagement(void);
void updatePositionManagerZPosition(float zPos, float dt);
void updatePositionManagerXYPosition(float xPos, float yPos, float dt);
void resetPositionManager(void);

void updateLateralVel();
void predictLateralVel(float dt);
void updatePositionCommand(float dt);

#endif /* SRC_FC_MANAGERS_POSITION_POSITIONMANAGER_H_ */
