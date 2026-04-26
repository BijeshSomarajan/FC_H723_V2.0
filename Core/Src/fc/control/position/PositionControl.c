#include "PositionControl.h"

#include "../../calibration/Calibration.h"
#include "../../managers/position/common/PositionCommon.h"
#include "../../memory/Memory.h"
#include "../ControlData.h"
#include "../Pid.h"

PID positionXPID, positionYPID, positionXRatePID, positionYRatePID;

/**
 * Initializes the attitude control
 */
uint8_t initPositionControl() {
	/* Attitude Master PID initiation*/
	pidInit(&positionXPID, getScaledCalibrationValue(CALIB_PROP_POS_HOLD_PID_KP_ADDR), 0, 0, 0);
	pidInit(&positionYPID, getScaledCalibrationValue(CALIB_PROP_POS_HOLD_PID_KP_ADDR), 0, 0, 0);
	//Set overall limit
	pidSetPIDOutputLimits(&positionXPID, -getCalibrationValue(CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR));
	pidSetPIDOutputLimits(&positionYPID, -getCalibrationValue(CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR));
	//Set P limit
	pidSetPOutputLimits(&positionXPID, -getCalibrationValue(CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR));
	pidSetPOutputLimits(&positionYPID, -getCalibrationValue(CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR));

	/* Attitude Rate PID Settings*/
	pidInit(&positionXRatePID, getScaledCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_KP_ADDR), getScaledCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_KI_ADDR), getScaledCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_KD_ADDR), POSITION_CONTROL_D_RATE_LPF_FREQ);
	pidInit(&positionYRatePID, getScaledCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_KP_ADDR), getScaledCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_KI_ADDR), getScaledCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_KD_ADDR), POSITION_CONTROL_D_RATE_LPF_FREQ);
	//Set overall limit
	pidSetPIDOutputLimits(&positionXRatePID, -getCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR));
	pidSetPIDOutputLimits(&positionYRatePID, -getCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR));
	//Set P limit
	pidSetPOutputLimits(&positionXRatePID, -getCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR));
	pidSetPOutputLimits(&positionYRatePID, -getCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR));
	//Set I limit
	pidSetIOutputLimits(&positionXRatePID, -getCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR) * POSITION_CONTROL_RATE_PID_I_LIMIT_RATIO, getCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR) * POSITION_CONTROL_RATE_PID_I_LIMIT_RATIO);
	pidSetIOutputLimits(&positionYRatePID, -getCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR) * POSITION_CONTROL_RATE_PID_I_LIMIT_RATIO, getCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR) * POSITION_CONTROL_RATE_PID_I_LIMIT_RATIO);

	//Set D limit
	pidSetDOutputLimits(&positionXRatePID, -getCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR) * POSITION_CONTROL_RATE_PID_D_LIMIT_RATIO, getCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR) * POSITION_CONTROL_RATE_PID_D_LIMIT_RATIO);
	pidSetDOutputLimits(&positionYRatePID, -getCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR) * POSITION_CONTROL_RATE_PID_D_LIMIT_RATIO, getCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR) * POSITION_CONTROL_RATE_PID_D_LIMIT_RATIO);

	return 1;
}

void resetPositionControl(uint8_t hard) {
	if (hard) {
		pidReset(&positionXPID);
		pidReset(&positionYPID);
		pidReset(&positionXRatePID);
		pidReset(&positionYRatePID);
		controlData.positionXControl = 0;
		controlData.positionYControl = 0;
	}
	pidResetI(&positionXRatePID);
	pidResetI(&positionYRatePID);
}

__ATTR_ITCM_TEXT
void controlPositionWithGains(float dt, float expectedX, float expectedY, float masterPGain, float ratePGain, float rateIGain, float rateDGain) {
	/*
	 positionCordinateData.xPosition = +100;
	 positionCordinateData.xVelocity = +100;
	 positionCordinateData.yPosition = 0;
	 positionCordinateData.yVelocity = 0;
	 */
	pidUpdateWithGains(&positionXPID, positionCordinateData.xPosition, expectedX, dt, masterPGain, 0.0f, 0.0f);
	pidUpdateWithGains(&positionXRatePID, positionCordinateData.xVelocity, positionXPID.pid, dt, ratePGain, rateIGain, rateDGain);

	pidUpdateWithGains(&positionYPID, positionCordinateData.yPosition, expectedY, dt, masterPGain, 0.0f, 0.0f);
	pidUpdateWithGains(&positionYRatePID, positionCordinateData.yVelocity, positionYPID.pid, dt, ratePGain, rateIGain, rateDGain);

	controlData.positionXControl = positionXRatePID.pid;
	controlData.positionYControl = positionYRatePID.pid;
}

#define POSITION_MGR_KVFF            0.08f
#define POSITION_MGR_KAFF            0.0f

#define POSITION_MGR_ACC_FF_MAX      3.5f
#define POSITION_MGR_ACC_ALPHA       0.33f   // slightly faster response

#define POSITION_MGR_VEL_MAX         3.0f   // m/s
#define POSITION_MGR_VEL_RATE_LIMIT  4.0f   // m/s²

void controlPositionWithGainsOld(float dt, float expectedX, float expectedY, float masterPGain, float ratePGain, float rateIGain, float rateDGain) {
	static float lastVelTargetX = 0.0f;
	static float lastVelTargetY = 0.0f;
	static float filteredAccX = 0.0f;
	static float filteredAccY = 0.0f;
	// Use actual dt. 1e-6f is plenty to prevent divide-by-zero
	float safeDt = fmaxf(dt, 1e-6f);
	// ===================== X AXIS =====================
	// 1. Position -> Velocity target
	pidUpdateWithGains(&positionXPID, positionCordinateData.xPosition, expectedX, dt, masterPGain, 0.0f, 0.0f);
	float velTargetX = positionXPID.pid;
	// 2. Clamp velocity target (Saturation)
	velTargetX = constrainToRangeF(velTargetX, -POSITION_MGR_VEL_MAX, POSITION_MGR_VEL_MAX);
	// 3. Slew Rate Limit (The "Smoothness" Engine)
	float deltaVelX = (velTargetX - lastVelTargetX);
	float maxDeltaVel = POSITION_MGR_VEL_RATE_LIMIT * dt;
	deltaVelX = constrainToRangeF(deltaVelX, -maxDeltaVel, maxDeltaVel);
	velTargetX = lastVelTargetX + deltaVelX;
	// 4. Rate PID (Inner Loop)
	pidUpdateWithGains(&positionXRatePID, positionCordinateData.xVelocity, velTargetX, dt, ratePGain, rateIGain, rateDGain);
	// 5. Acceleration FF (Derivative)
	float rawAccX = (velTargetX - lastVelTargetX) / safeDt;
	lastVelTargetX = velTargetX; // Update last target AFTER calculation
	// LPF (Derivative Filtering)
	filteredAccX = (POSITION_MGR_ACC_ALPHA * rawAccX) + ((1.0f - POSITION_MGR_ACC_ALPHA) * filteredAccX);
	float accFFX = constrainToRangeF(filteredAccX, -POSITION_MGR_ACC_FF_MAX, POSITION_MGR_ACC_FF_MAX);
	// 6. Combined Output
	controlData.positionXControl = positionXRatePID.pid + (POSITION_MGR_KVFF * velTargetX) + (POSITION_MGR_KAFF * accFFX);
	// ===================== Y AXIS (Same Logic) =====================
	pidUpdateWithGains(&positionYPID, positionCordinateData.yPosition, expectedY, dt, masterPGain, 0.0f, 0.0f);
	float velTargetY = positionYPID.pid;
	velTargetY = constrainToRangeF(velTargetY, -POSITION_MGR_VEL_MAX, POSITION_MGR_VEL_MAX);
	float deltaVelY = (velTargetY - lastVelTargetY);
	float maxDeltaVelY = POSITION_MGR_VEL_RATE_LIMIT * dt;
	deltaVelY = constrainToRangeF(deltaVelY, -maxDeltaVelY, maxDeltaVelY);
	velTargetY = lastVelTargetY + deltaVelY;
	pidUpdateWithGains(&positionYRatePID, positionCordinateData.yVelocity, velTargetY, dt, ratePGain, rateIGain, rateDGain);
	float rawAccY = (velTargetY - lastVelTargetY) / safeDt;
	lastVelTargetY = velTargetY;
	filteredAccY = (POSITION_MGR_ACC_ALPHA * rawAccY) + ((1.0f - POSITION_MGR_ACC_ALPHA) * filteredAccY);
	float accFFY = constrainToRangeF(filteredAccY, -POSITION_MGR_ACC_FF_MAX, POSITION_MGR_ACC_FF_MAX);
	controlData.positionYControl = positionYRatePID.pid + (POSITION_MGR_KVFF * velTargetY) + (POSITION_MGR_KAFF * accFFY);
}

