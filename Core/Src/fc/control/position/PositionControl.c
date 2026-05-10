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
void _controlPositionRateWithGainsFF(float dt, float ratePGain, float rateIGain, float rateDGain) {
	// Base velocity target from position PID
	float velocityTargetX = positionXPID.pid;
	float velocityTargetY = positionYPID.pid;
#if POSITION_CONTROL_VEL_FEED_FWD_ENABLED == 1
	// Velocity Feedforward (disturbance rejection)
	velocityTargetX += (-positionCordinateData.xVelocity * POSITION_CONTROL_VEL_FEED_FWD_GAIN);
	velocityTargetY += (-positionCordinateData.yVelocity * POSITION_CONTROL_VEL_FEED_FWD_GAIN);
	// Clamp AFTER adding FF
	velocityTargetX = constrainToRangeF(velocityTargetX, -getCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR));
	velocityTargetY = constrainToRangeF(velocityTargetY, -getCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR));
#endif
	// Velocity PID
	pidUpdateWithGains(&positionXRatePID, positionCordinateData.xVelocity, velocityTargetX, dt, ratePGain, rateIGain, rateDGain);
	pidUpdateWithGains(&positionYRatePID, positionCordinateData.yVelocity, velocityTargetY, dt, ratePGain, rateIGain, rateDGain);
	controlData.positionXControl = positionXRatePID.pid;
	controlData.positionYControl = positionYRatePID.pid;
}

__ATTR_ITCM_TEXT
void _controlPositionRateWithGainsDist(float dt, float ratePGain, float rateIGain, float rateDGain) {
	float velocityTargetX = positionXPID.pid;
	float velocityTargetY = positionYPID.pid;
	pidUpdateWithGains(&positionXRatePID, positionCordinateData.xVelocity, velocityTargetX, dt, ratePGain, rateIGain, rateDGain);
	pidUpdateWithGains(&positionYRatePID, positionCordinateData.yVelocity, velocityTargetY, dt, ratePGain, rateIGain, rateDGain);
	float outputX = positionXRatePID.pid;
	float outputY = positionYRatePID.pid;
	/*---------------- Velocity disturbance ----------------*/
	float velErrX = positionCordinateData.xVelocity - velocityTargetX;
	float velErrY = positionCordinateData.yVelocity - velocityTargetY;
	float alphaVel = dt / (POSITION_CONTROL_DIST_EST_VEL_TAU + dt);
	positionControlXVelDist += alphaVel * (velErrX - positionControlXVelDist);
	positionControlYVelDist += alphaVel * (velErrY - positionControlYVelDist);
	/* Clamp state */
	positionControlXVelDist = constrainToRangeF(positionControlXVelDist, -POSITION_CONTROL_DIST_EST_STATE_LIMIT, POSITION_CONTROL_DIST_EST_STATE_LIMIT);
	positionControlYVelDist = constrainToRangeF(positionControlYVelDist, -POSITION_CONTROL_DIST_EST_STATE_LIMIT, POSITION_CONTROL_DIST_EST_STATE_LIMIT);
	/*---------------- Acc disturbance ----------------*/
	float expectedAccX = outputX * POSITION_CONTROL_DIST_EST_ACCEL_MODEL_K;
	float expectedAccY = outputY * POSITION_CONTROL_DIST_EST_ACCEL_MODEL_K;
	float distAccX = positionCordinateData.xAcceleration - expectedAccX;
	float distAccY = positionCordinateData.yAcceleration - expectedAccY;
	/* Clamp raw disturbance */
	distAccX = constrainToRangeF(distAccX, -POSITION_CONTROL_DIST_EST_ACC_LIMIT, POSITION_CONTROL_DIST_EST_ACC_LIMIT);
	distAccY = constrainToRangeF(distAccY, -POSITION_CONTROL_DIST_EST_ACC_LIMIT, POSITION_CONTROL_DIST_EST_ACC_LIMIT);
	float alphaAcc = dt / (POSITION_CONTROL_DIST_EST_ACC_TAU + dt);
	positionControlXAccDist += alphaAcc * (distAccX - positionControlXAccDist);
	positionControlYAccDist += alphaAcc * (distAccY - positionControlYAccDist);
	/* Clamp state */
	positionControlXAccDist = constrainToRangeF(positionControlXAccDist, -POSITION_CONTROL_DIST_EST_STATE_LIMIT, POSITION_CONTROL_DIST_EST_STATE_LIMIT);
	positionControlYAccDist = constrainToRangeF(positionControlYAccDist, -POSITION_CONTROL_DIST_EST_STATE_LIMIT, POSITION_CONTROL_DIST_EST_STATE_LIMIT);

	/*---------------- Combine ----------------*/
	float xComp = positionControlXVelDist * POSITION_CONTROL_DIST_EST_VEL_GAIN + positionControlXAccDist * POSITION_CONTROL_DIST_EST_ACC_GAIN;
	float yComp = positionControlYVelDist * POSITION_CONTROL_DIST_EST_VEL_GAIN + positionControlYAccDist * POSITION_CONTROL_DIST_EST_ACC_GAIN;

	/* Final clamp */
	xComp = constrainToRangeF(xComp, -POSITION_CONTROL_DIST_EST_TOTAL_OUTPUT_LIMIT, POSITION_CONTROL_DIST_EST_TOTAL_OUTPUT_LIMIT);
	yComp = constrainToRangeF(yComp, -POSITION_CONTROL_DIST_EST_TOTAL_OUTPUT_LIMIT, POSITION_CONTROL_DIST_EST_TOTAL_OUTPUT_LIMIT);

	outputX -= xComp;
	outputY -= yComp;

	/* Final safety clamp */
	float rateLimit = getCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR);
	outputX = constrainToRangeF(outputX, -rateLimit, rateLimit);
	outputY = constrainToRangeF(outputY, -rateLimit, rateLimit);

	controlData.positionXControl = outputX;
	controlData.positionYControl = outputY;
}

void controlPositionRateWithGains(float dt, float ratePGain, float rateIGain, float rateDGain) {
#if POSITION_CONTROL_DIST_EST_ENABLED == 1
	_controlPositionRateWithGainsDist(dt, ratePGain, rateIGain, rateDGain);
#else
	_controlPositionRateWithGainsFF(dt, ratePGain, rateIGain, rateDGain);
#endif
}

__ATTR_ITCM_TEXT
void setExpectedPositionVelocity(float dt, float expectedVelX, float expectedVelY) {
	positionXPID.pid = constrainToRangeF(expectedVelX, -getCalibrationValue(CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR));
	positionYPID.pid = constrainToRangeF(expectedVelY, -getCalibrationValue(CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR));
}

__ATTR_ITCM_TEXT
void controlPositionCordinatesWithGains(float dt, float expectedX, float expectedY, float masterPGain) {
#if POSITION_CONTROL_NONLINEAR_BOOST_ENABLED == 1
	float errorX = expectedX - positionCordinateData.xPosition;
	float errorY = expectedY - positionCordinateData.yPosition;
	if (fabsf(errorX) < POSITION_CONTROL_NONLINEAR_BOOST_DEADBAND) {
		errorX = 0.0f;
	}
	if (fabsf(errorY) < POSITION_CONTROL_NONLINEAR_BOOST_DEADBAND) {
		errorY = 0.0f;
	}
	float boostX = 0.0f;
	float boostY = 0.0f;
	float absErrX = fabsf(errorX);
	float absErrY = fabsf(errorY);
	if (absErrX > POSITION_CONTROL_NONLINEAR_BOOST_START) {
		boostX = POSITION_CONTROL_NONLINEAR_BOOST_GAIN * fastSqrtf(absErrX - POSITION_CONTROL_NONLINEAR_BOOST_START);
		boostX = copysignf(boostX, errorX);
	}
	if (absErrY > POSITION_CONTROL_NONLINEAR_BOOST_START) {
		boostY = POSITION_CONTROL_NONLINEAR_BOOST_GAIN * fastSqrtf(absErrY - POSITION_CONTROL_NONLINEAR_BOOST_START);
		boostY = copysignf(boostY, errorY);
	}
	positionXPID.pid = constrainToRangeF((masterPGain * errorX) + boostX, -getCalibrationValue(CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR));
	positionYPID.pid = constrainToRangeF((masterPGain * errorY) + boostY, -getCalibrationValue(CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR));
#else
	pidUpdateWithGains(&positionXPID, positionCordinateData.xPosition, expectedX, dt, masterPGain, 0.0f, 0.0f);
	pidUpdateWithGains(&positionYPID, positionCordinateData.yPosition, expectedY, dt, masterPGain, 0.0f, 0.0f);
#endif
}
