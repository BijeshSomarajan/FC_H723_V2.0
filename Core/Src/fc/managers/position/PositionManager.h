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

#define POSITION_MGR_MAX_POS_COMMAND  15.0f // Degrees

#define POSITION_MGR_X_VEL_LPF_FREQ 25.00f
#define POSITION_MGR_Y_VEL_LPF_FREQ 25.00f
#define POSITION_MGR_Z_VEL_LPF_FREQ 40.00f

#define POSITION_MGR_X_ACC_LPF_FREQ 40.00f
#define POSITION_MGR_Y_ACC_LPF_FREQ 40.00f
#define POSITION_MGR_Z_ACC_LPF_FREQ 40.00f

#define POSITION_MGR_X_ACC_DEADBAND 0.0f //0.01f    // Meter/Sec2
#define POSITION_MGR_Y_ACC_DEADBAND 0.0f //0.01f    // Meter/Sec2
#define POSITION_MGR_Z_ACC_DEADBAND 1.0f            // Meter/Sec2

#define POSITION_MGR_X_ESTIMATION_ACC_DEADBAND   0.0f //0.1f  // Meter/Sec2
#define POSITION_MGR_Y_ESTIMATION_ACC_DEADBAND   0.0f //0.1f  // Meter/Sec2
#define POSITION_MGR_Z_ESTIMATION_ACC_DEADBAND   0.1f         // Meter/Sec2

#define POSITION_MGR_X_VEL_DEADBAND     0.0f   //0.005f // Meter/Sec
#define POSITION_MGR_Y_VEL_DEADBAND     0.0f   //0.005f // Meter/Sec
#define POSITION_MGR_Z_VEL_DEADBAND     1.0f   // Cm/Sec

#define POSITION_MGR_X_VEL_MAX          50.0f  // M/Sec
#define POSITION_MGR_Y_VEL_MAX          50.0f  // M/Sec
#define POSITION_MGR_Z_VEL_MAX          500.0f // cm/Sec2

#define POSITION_MGR_X_ACC_MAX          50.0f  // M/Sec2
#define POSITION_MGR_Y_ACC_MAX          50.0f  // M/Sec2
#define POSITION_MGR_Z_ACC_MAX          500.0f // Cm/Sec2

#define POSITION_MGR_Z_ENABLE_DYNAMIC_R 1

#define POSITION_MGR_XY_VEL_RESET_DAMP_STRENGTH   3.0f //0.5f
#define POSITION_MGR_XY_VEL_UPDATE_DAMP_STRENGTH  2.0f //0.5f
#define POSITION_MGR_GNSS_VEL_SACC_SCALE     1.2f //1.0f
#define POSITION_MGR_GNSS_VEL_SACC_MIN       0.05f
#define POSITION_MGR_GNSS_VEL_R_MAX          5.0f //10.0f


// Base measurement noise for X and Y position (m^2)
// A value of 0.04f represents a standard deviation of 0.2m (20cm)
#define POSITION_MGR_XY_POS_DYNAMIC_R_BASE  0.5f // 0.04f
#define POSITION_MGR_GNSS_POS_HACC_SCALE    1.0f // 2.0f    // Higher scale for position
#define POSITION_MGR_GNSS_POS_HACC_MIN      0.1f // 10cm floor
#define POSITION_MGR_GNSS_POS_R_MAX        20.0f // Position can tolerate higher R

//Loiter configurations
#define POSITION_MGR_POS_HOLD_MAX_VELOCITY     15.0f
#define POSITION_MGR_POS_HOLD_ELAPSE_TIME      0.5f
#define POSITION_MGR_POS_HOLD_BRAKE_GAIN       0.6f

#define POSITION_MGR_VENTURI_ESTIMATE_ENABLED 0

typedef enum {
    POS_HOLD_STATE_IDLE = 0,
    POS_HOLD_STATE_BRAKING,
	POS_MGR_SETTLING,
    POS_HOLD_STATE_LOCKED
} POSITION_MGR_STATE;

uint8_t initPositionManager(void);
void doPositionManagement(void);
void updatePositionManagerZPosition(float zPos, float dt);
void updatePositionManagerXYPosition(float xPos, float yPos, float dt);
void resetPositionManager(void);

void updateLateralVel() ;
void predictLateralVel(float dt);
void updatePositionCommand(float dt);

#endif /* SRC_FC_MANAGERS_POSITION_POSITIONMANAGER_H_ */
