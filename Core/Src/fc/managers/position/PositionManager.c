#include "PositionManager.h"

#include "../../dsp/LowPassFilter.h"
#include "../../FCConfig.h"
#include "../../imu/IMU.h"
#include "../../logger/Logger.h"
#include "../../memory/Memory.h"
#include "../../timers/DeltaTimer.h"
#include "../../timers/Scheduler.h"
#include "../../util/MathUtil.h"
#include "estimator/PositionEstimator.h"

POSITION_EKF positionEkf;
LOWPASSFILTER positionMgrAccXLPF, positionMgrAccYLPF, positionMgrAccZLPF;
POSITION_DATA positionData;

void managePositionTask(void);

uint8_t initPositionManager(void) {
	logString("[Position Manager] Init > Start\n");
	uint8_t status = positionEKFInit(&positionEkf);
	if (status) {
		logString("[Position Manager] EKF Init > Success\n");

		lowPassFilterInit(&positionMgrAccXLPF, POSITION_MGR_X_ACC_LPF_FREQ);
		lowPassFilterInit(&positionMgrAccYLPF, POSITION_MGR_Y_ACC_LPF_FREQ);
		lowPassFilterInit(&positionMgrAccZLPF, POSITION_MGR_Z_ACC_LPF_FREQ);

		schedulerAddTask(managePositionTask, POSITION_MANAGEMENT_TASK_FREQUENCY, POSITION_MANAGEMENT_TASK_PRIORITY);
		logString("[Position Manager] All tasks   > Started\n");
	} else {
		logString("[Position Manager] EKF Init > Failed\n");
	}
	return 1;
}

__ATTR_ITCM_TEXT
void upadatePositionVelocity(float vx, float vy, float vz, float dt) {
	float vel;
	// X Axis
	vel = applyDeadBandFloat(0.0f, vx, POSITION_MGR_X_VEL_DEADBAND);
	vel = constrainToRangeF(vel, -POSITION_MGR_X_VEL_MAX, POSITION_MGR_X_VEL_MAX);
	positionData.xVelocity = vel;

	// Y Axis
	vel = applyDeadBandFloat(0.0f, vy, POSITION_MGR_Y_VEL_DEADBAND);
	vel = constrainToRangeF(vel, -POSITION_MGR_Y_VEL_MAX, POSITION_MGR_Y_VEL_MAX);
	positionData.yVelocity = vel;

	// Z Axis
	vel = applyDeadBandFloat(0.0f, vz, POSITION_MGR_Z_VEL_DEADBAND);
	vel = constrainToRangeF(vel, -POSITION_MGR_Z_VEL_MAX, POSITION_MGR_Z_VEL_MAX);
	positionData.zVelocity = vel;
}

__ATTR_ITCM_TEXT
void upadatePositionAcceleration(float ax, float ay, float az, float dt) {
	float acc;
	// X Axis
	acc = applyDeadBandFloat(0.0f, ax, POSITION_MGR_X_ACC_DEADBAND);
	acc = constrainToRangeF(acc, -POSITION_MGR_X_ACC_MAX, POSITION_MGR_X_ACC_MAX);
	positionData.xAcceleration = lowPassFilterUpdate(&positionMgrAccXLPF, acc, dt);

	// Y Axis
	acc = applyDeadBandFloat(0.0f, ay - positionData.yAccelerationBias, POSITION_MGR_Y_ACC_DEADBAND);
	acc = constrainToRangeF(acc, -POSITION_MGR_Y_ACC_MAX, POSITION_MGR_Y_ACC_MAX);
	positionData.yAcceleration = lowPassFilterUpdate(&positionMgrAccYLPF, acc, dt);

	// Z Axis
	acc = applyDeadBandFloat(0.0f, az - positionData.zAccelerationBias, POSITION_MGR_Z_ACC_DEADBAND);
	acc = constrainToRangeF(acc, -POSITION_MGR_Z_ACC_MAX, POSITION_MGR_Z_ACC_MAX);
	positionData.zAcceleration = lowPassFilterUpdate(&positionMgrAccZLPF, acc, dt);
}

__ATTR_ITCM_TEXT
void managePositionTask(void) {
	float dt = getDeltaTime(POSITION_MANAGER_TIMER_CHANNEL);

	if (dt <= 0.0f || dt > 0.1f) dt = 0.001f; // Safety guard for dt
	float axEarth = imuData.axEarth * POSITION_MGR_ACC_OUTPUT_GAIN;
	float ayEarth = imuData.ayEarth * POSITION_MGR_ACC_OUTPUT_GAIN;
	float azEarth = imuData.azEarth * POSITION_MGR_ACC_OUTPUT_GAIN;

	// 1. Prediction (Using raw or slightly scaled earth-frame acc)
	positionEKFPredict(&positionEkf, axEarth, ayEarth, azEarth, dt);

	float *x = positionEkf.x;
	positionData.xPosition = x[0]; // X Pos
	positionData.xAccelerationBias = x[2]; // X Bias

	positionData.yPosition = x[3]; // Y Pos
	positionData.yAccelerationBias = x[5]; // Y Bias

	positionData.zPosition = x[6]; // Z Pos
	positionData.zAccelerationBias = x[8]; // Z Bias

	// 2. Filtered Acceleration
	upadatePositionAcceleration(axEarth - positionData.xAccelerationBias, ayEarth - positionData.yAccelerationBias, azEarth - positionData.zAccelerationBias, dt);

	// 3. Filtered Velocity
	upadatePositionVelocity(x[1], x[4], x[7], dt);

	positionData.positionProcessDt = dt;
}

void resetPositionManager(void) {
	lowPassFilterReset(&positionMgrAccXLPF);
	lowPassFilterReset(&positionMgrAccYLPF);
	lowPassFilterReset(&positionMgrAccZLPF);
	positionEKFReset(&positionEkf, 0, 0, 0);
}

__ATTR_ITCM_TEXT
void updatePositionManagerZPosition(float zPos, float dt) {
	positionData.positionZUpdateDt = dt;
	positionEKFUpdateZMeasure(&positionEkf, zPos);
	dampPositionManagerXYVelocity(dt);
}

__ATTR_ITCM_TEXT
void updatePositionManagerXYPosition(float xPos, float yPos, float dt) {
	positionData.positionXYUpdateDt = dt;
	positionEKFUpdateXYMeasure(&positionEkf, xPos, yPos);
}

__ATTR_ITCM_TEXT
void dampPositionManagerXYVelocity(float dt) {
	positionEKFApplyXYDamping(&positionEkf, POSITION_MGR_XY_VEL_DAMP_STRENGTH);
}

