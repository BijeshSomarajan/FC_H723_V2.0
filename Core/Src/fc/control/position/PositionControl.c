#include "PositionControl.h"

#include "../../calibration/Calibration.h"
#include "../../managers/position/common/PositionCommon.h"
#include "../../dsp/Interpolator.h"
#include "../../memory/Memory.h"
#include "../ControlData.h"
#include "../Pid.h"

PID positionXPID, positionYPID, positionXRatePID, positionYRatePID;
float positionControlXVelDist = 0.0f, positionControlYVelDist = 0.0f;
float positionControlXAccDist = 0.0f, positionControlYAccDist = 0.0f;

/**
 * Initializes the attitude control
 */
uint8_t initPositionControl(float masterControlFrequency, float rateControlFrequency) {
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

__ATTR_ITCM_TEXT
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
	positionControlXVelDist = 0.0f;
	positionControlYVelDist = 0.0f;
	positionControlXAccDist = 0.0f;
	positionControlYAccDist = 0.0f;
}

__ATTR_ITCM_TEXT
void controlPositionRateWithGains(float dt, float ratePGain, float rateIGain, float rateDGain) {
	/*---------------- Position → Velocity target ----------------*/
	float velocityTargetX = positionXPID.pid;
	float velocityTargetY = positionYPID.pid;

#if POSITION_CONTROL_VEL_FF_ENABLED == 1
	float velFFX = (-positionCordinateData.xVelocity * POSITION_CONTROL_VEL_FF_GAIN);
	float velFFY = (-positionCordinateData.yVelocity * POSITION_CONTROL_VEL_FF_GAIN);
#endif
	/*---------------- Velocity PID ----------------*/
	pidUpdateWithGains(&positionXRatePID, positionCordinateData.xVelocity, velocityTargetX, dt, ratePGain, rateIGain, rateDGain);
	pidUpdateWithGains(&positionYRatePID, positionCordinateData.yVelocity, velocityTargetY, dt, ratePGain, rateIGain, rateDGain);

	float outputX = positionXRatePID.pid;
	float outputY = positionYRatePID.pid;

#if POSITION_CONTROL_DOB_ENABLED == 1

	/*---------------- Velocity disturbance (LOW frequency) ----------------*/
	float velErrX = positionCordinateData.xVelocity - velocityTargetX;
	float velErrY = positionCordinateData.yVelocity - velocityTargetY;

	float alphaVel = dt / (POSITION_CONTROL_DOB_VEL_TAU + dt);

	positionControlXVelDist += alphaVel * (velErrX - positionControlXVelDist);
	positionControlYVelDist += alphaVel * (velErrY - positionControlYVelDist);

	positionControlXVelDist = constrainToRangeF(positionControlXVelDist, -POSITION_CONTROL_DOB_STATE_LIMIT, POSITION_CONTROL_DOB_STATE_LIMIT);
	positionControlYVelDist = constrainToRangeF(positionControlYVelDist, -POSITION_CONTROL_DOB_STATE_LIMIT, POSITION_CONTROL_DOB_STATE_LIMIT);

	/*---------------- Acceleration disturbance (HIGH frequency) ----------------*/
	float expectedAccX = outputX * POSITION_CONTROL_DOB_ACCEL_MODEL_K;
	float expectedAccY = outputY * POSITION_CONTROL_DOB_ACCEL_MODEL_K;

	float distAccX = positionCordinateData.xAcceleration - expectedAccX;
	float distAccY = positionCordinateData.yAcceleration - expectedAccY;

	distAccX = constrainToRangeF(distAccX, -POSITION_CONTROL_DOB_ACC_LIMIT, POSITION_CONTROL_DOB_ACC_LIMIT);
	distAccY = constrainToRangeF(distAccY, -POSITION_CONTROL_DOB_ACC_LIMIT, POSITION_CONTROL_DOB_ACC_LIMIT);

	float alphaAcc = dt / (POSITION_CONTROL_DOB_ACC_TAU + dt);

	positionControlXAccDist += alphaAcc * (distAccX - positionControlXAccDist);
	positionControlYAccDist += alphaAcc * (distAccY - positionControlYAccDist);

	positionControlXAccDist = constrainToRangeF(positionControlXAccDist, -POSITION_CONTROL_DOB_STATE_LIMIT, POSITION_CONTROL_DOB_STATE_LIMIT);
	positionControlYAccDist = constrainToRangeF(positionControlYAccDist, -POSITION_CONTROL_DOB_STATE_LIMIT, POSITION_CONTROL_DOB_STATE_LIMIT);

	/*---------------- Combine disturbance estimate ----------------*/
	float xComp = positionControlXVelDist * POSITION_CONTROL_DOB_VEL_GAIN + positionControlXAccDist * POSITION_CONTROL_DOB_ACC_GAIN;
	float yComp = positionControlYVelDist * POSITION_CONTROL_DOB_VEL_GAIN + positionControlYAccDist * POSITION_CONTROL_DOB_ACC_GAIN;

	xComp = constrainToRangeF(xComp, -POSITION_CONTROL_DOB_OUTPUT_LIMIT, POSITION_CONTROL_DOB_OUTPUT_LIMIT);
	yComp = constrainToRangeF(yComp, -POSITION_CONTROL_DOB_OUTPUT_LIMIT, POSITION_CONTROL_DOB_OUTPUT_LIMIT);

	/*---------------- Apply DOB correction ----------------*/
	outputX -= xComp;
	outputY -= yComp;
#endif

	/*---------------- Feedforward (applied LAST, not inside DOB loop) ----------------*/
#if POSITION_CONTROL_VEL_FF_ENABLED == 1
	outputX += velFFX;
	outputY += velFFY;
#endif

	/*---------------- Final clamp ----------------*/
	float rateLimit = getCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR);
	outputX = constrainToRangeF(outputX, -rateLimit, rateLimit);
	outputY = constrainToRangeF(outputY, -rateLimit, rateLimit);
	controlData.positionXControl = outputX;
	controlData.positionYControl = outputY;
}

__ATTR_ITCM_TEXT
void setExpectedPositionVelocity(float dt, float expectedVelX, float expectedVelY) {
	positionXPID.pid = constrainToRangeF(expectedVelX, -getCalibrationValue(CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR));
	positionYPID.pid = constrainToRangeF(expectedVelY, -getCalibrationValue(CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR));
}

__ATTR_ITCM_TEXT
void controlPositionCordinatesWithGains(float dt, float expectedX, float expectedY, float masterPGain) {
	pidUpdateWithGains(&positionXPID, positionCordinateData.xPosition, expectedX, dt, masterPGain, 0.0f, 0.0f);
	pidUpdateWithGains(&positionYPID, positionCordinateData.yPosition, expectedY, dt, masterPGain, 0.0f, 0.0f);
}
