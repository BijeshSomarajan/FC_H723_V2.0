#ifndef SRC_FC_MANAGERS_POSITION_POSITIONMANAGER_H_
#define SRC_FC_MANAGERS_POSITION_POSITIONMANAGER_H_
#include <sys/_stdint.h>

typedef struct _POSITION_CORDINATE_DATA POSITION_CORDINATE_DATA;
struct _POSITION_CORDINATE_DATA {
	float xPosition, yPosition, zPosition;
	float xVelocity, yVelocity, zVelocity;
	float xAcceleration, yAcceleration, zAcceleration;
	float xAccelerationBias, yAccelerationBias, zAccelerationBias;

	float positionProcessDt;
	float positionXYUpdateDt;
	float positionZUpdateDt;
};
extern POSITION_CORDINATE_DATA positionCordinateData;

typedef struct _POSITION_COMMAND_DATA POSITION_COMMAND_DATA;
struct _POSITION_COMMAND_DATA {
	float pitchCommand, rollCommand;
	float positionCommandDt;
};
extern POSITION_COMMAND_DATA positionCommandData;


#define POSITION_MANAGEMENT_TASK_FREQUENCY 1000
#define POSITION_MGR_ACC_OUTPUT_GAIN  100.0f  // cm/sec2
#define POSITION_MGR_X_POS_OUTPUT_GAIN 100.0f  // cm/sec2
#define POSITION_MGR_Y_POS_OUTPUT_GAIN 100.0f  // cm/sec2
#define POSITION_MGR_Z_POS_OUTPUT_GAIN 100.0f  // cm/sec2

#define POSITION_MGR_X_ACC_LPF_FREQ   40.00f
#define POSITION_MGR_Y_ACC_LPF_FREQ   40.00f
#define POSITION_MGR_Z_ACC_LPF_FREQ   40.00f

#define POSITION_MGR_X_ACC_DEADBAND     1.0f    // Meter/Sec2
#define POSITION_MGR_Y_ACC_DEADBAND     1.0f    // Meter/Sec2
#define POSITION_MGR_Z_ACC_DEADBAND     1.0f    // Meter/Sec2

#define POSITION_MGR_X_VEL_DEADBAND     1.0f    // Meter/Sec
#define POSITION_MGR_Y_VEL_DEADBAND     1.0f    // Meter/Sec
#define POSITION_MGR_Z_VEL_DEADBAND     1.0f    // Meter/Sec

#define POSITION_MGR_X_VEL_MAX          500.0f  // cm/Sec2
#define POSITION_MGR_Y_VEL_MAX          500.0f  // cm/Sec2
#define POSITION_MGR_Z_VEL_MAX          500.0f  // cm/Sec2

#define POSITION_MGR_X_ACC_MAX          500.0f  // cm/Sec2
#define POSITION_MGR_Y_ACC_MAX          500.0f  // cm/Sec2
#define POSITION_MGR_Z_ACC_MAX          500.0f  // cm/Sec2

#define POSITION_MGR_XY_VEL_DAMP_STRENGTH   0.75f

// --- PITCH BRAKE CONFIG ---
#define POSITION_MGR_PITCH_BRAKE_DELAY    0.30f
#define POSITION_MGR_PITCH_BRAKE_WIDTH    0.15f
#define POSITION_MGR_PITCH_BRAKE_FADE_IN  0.01f
#define POSITION_MGR_PITCH_BRAKE_GAIN     2.00f
#define POSITION_MGR_PITCH_BRAKE_LIMIT    50.0f
// --- ROLL BRAKE CONFIG ---
#define POSITION_MGR_ROLL_BRAKE_DELAY     0.15f
#define POSITION_MGR_ROLL_BRAKE_WIDTH     0.1f
#define POSITION_MGR_ROLL_BRAKE_FADE_IN   0.01f
#define POSITION_MGR_ROLL_BRAKE_GAIN      0.8f
#define POSITION_MGR_ROLL_BRAKE_LIMIT     30.0f


uint8_t initPositionManager(void);
void doPositionManagement(void);
void updatePositionManagerZPosition(float zPos, float dt);
void updatePositionManagerXYPosition(float xPos, float yPos, float dt);
void dampPositionManagerXYVelocity(float dt);
void resetPositionManager(void);

void updateLateralVel() ;
void predictLateralVel(float dt);

void updatePositionCommand(float dt);

#endif /* SRC_FC_MANAGERS_POSITION_POSITIONMANAGER_H_ */
