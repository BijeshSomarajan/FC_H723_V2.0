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
void estimateAcceleration(float axEarth, float ayEarth, float azEarth, float dt) {
	float acc;
	// X Axis
	acc = applyDeadBandFloat(0.0f, axEarth, POSITION_MGR_X_ACC_DEADBAND);
	acc = constrainToRangeF(acc, -POSITION_MGR_X_ACC_MAX, POSITION_MGR_X_ACC_MAX);
	positionData.xAcceleration = lowPassFilterUpdate(&positionMgrAccXLPF, acc, dt);

	// Y Axis
	acc = applyDeadBandFloat(0.0f, ayEarth, POSITION_MGR_Y_ACC_DEADBAND);
	acc = constrainToRangeF(acc, -POSITION_MGR_Y_ACC_MAX, POSITION_MGR_Y_ACC_MAX);
	positionData.yAcceleration = lowPassFilterUpdate(&positionMgrAccYLPF, acc, dt);

	// Z Axis
	acc = applyDeadBandFloat(0.0f, azEarth, POSITION_MGR_Z_ACC_DEADBAND);
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
	// 2. Filtered Acceleration for Telemetry/Logging
	estimateAcceleration(axEarth, ayEarth, azEarth, dt);	// 3. Map EKF State to Position Data Structure
	// Using a pointer for cleaner access
	float *x = positionEkf.x;
	positionData.xPosition = x[0]; // X Pos
	positionData.xVelocity = x[1]; // X Vel

	positionData.yPosition = x[3]; // Y Pos
	positionData.yVelocity = x[4]; // Y Vel

	positionData.zPosition = x[6]; // Z Pos
	positionData.zVelocity = x[7]; // Z Vel

	positionData.positionProcessDt = dt;
}

void resetPositionManager(void) {
	lowPassFilterReset(&positionMgrAccXLPF);
	lowPassFilterReset(&positionMgrAccYLPF);
	lowPassFilterReset(&positionMgrAccZLPF);
	positionEKFReset(&positionEkf, 0, 0, 0);
}

__ATTR_ITCM_TEXT
void updatePositionManagerZMeasure(float zPos, float dt) {
	positionData.positionZUpdateDt = dt;
	positionEKFUpdateZMeasure(&positionEkf, zPos);
}

__ATTR_ITCM_TEXT
void updatePositionManagerXYMeasure(float xPos, float yPos, float dt) {
	positionData.positionXYUpdateDt = dt;
	positionEKFUpdateXYMeasure(&positionEkf, xPos, yPos);
}

