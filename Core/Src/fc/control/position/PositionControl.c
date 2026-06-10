#include "PositionControl.h"

#include "../../calibration/Calibration.h"
#include "../../managers/position/common/PositionCommon.h"
#include "../../dsp/Interpolator.h"
#include "../../memory/Memory.h"
#include "../ControlData.h"
#include "../Pid.h"
#include "../../FCConfig.h"

PID positionXPID, positionYPID, positionXRatePID, positionYRatePID;
float positionControlXVelDist = 0.0f, positionControlYVelDist = 0.0f;
float positionControlXAccDist = 0.0f, positionControlYAccDist = 0.0f;
float posHoldRatePIDLimit, posHoldPIDLimit;
float positionControlDOBExecutedX = 0.0f;
float positionControlDOBExecutedY = 0.0f;
/**
 * Initializes the attitude control
 */
uint8_t initPositionControl(float masterControlFrequency, float rateControlFrequency) {
	/* Attitude Master PID initiation*/
	pidInit(&positionXPID, get1KXScaledCalibrationValue(CALIB_PROP_POS_HOLD_PID_KP_ADDR), 0, 0, 0);
	pidInit(&positionYPID, get1KXScaledCalibrationValue(CALIB_PROP_POS_HOLD_PID_KP_ADDR), 0, 0, 0);

	posHoldPIDLimit = get10XScaledCalibrationValue(CALIB_PROP_POS_HOLD_PID_LIMIT_ADDR);
	//Set overall limit
	pidSetPIDOutputLimits(&positionXPID, -posHoldPIDLimit, posHoldPIDLimit);
	pidSetPIDOutputLimits(&positionYPID, -posHoldPIDLimit, posHoldPIDLimit);
	//Set P limit
	pidSetPOutputLimits(&positionXPID, -posHoldPIDLimit, posHoldPIDLimit);
	pidSetPOutputLimits(&positionYPID, -posHoldPIDLimit, posHoldPIDLimit);

	posHoldRatePIDLimit = get10XScaledCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_LIMIT_ADDR);

	/* Attitude Rate PID Settings*/
	pidInit(&positionXRatePID, get1KXScaledCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_KP_ADDR), get1KXScaledCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_KI_ADDR), get1KXScaledCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_KD_ADDR), POSITION_CONTROL_D_RATE_LPF_FREQ);
	pidInit(&positionYRatePID, get1KXScaledCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_KP_ADDR), get1KXScaledCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_KI_ADDR), get1KXScaledCalibrationValue(CALIB_PROP_POS_HOLD_RATE_PID_KD_ADDR), POSITION_CONTROL_D_RATE_LPF_FREQ);
	//Set overall limit
	pidSetPIDOutputLimits(&positionXRatePID, -posHoldRatePIDLimit, posHoldRatePIDLimit);
	pidSetPIDOutputLimits(&positionYRatePID, -posHoldRatePIDLimit, posHoldRatePIDLimit);
	//Set P limit
	pidSetPOutputLimits(&positionXRatePID, -posHoldRatePIDLimit, posHoldRatePIDLimit);
	pidSetPOutputLimits(&positionYRatePID, -posHoldRatePIDLimit, posHoldRatePIDLimit);
	//Set I limit
	pidSetIOutputLimits(&positionXRatePID, -posHoldRatePIDLimit * POSITION_CONTROL_RATE_PID_I_LIMIT_RATIO, posHoldRatePIDLimit * POSITION_CONTROL_RATE_PID_I_LIMIT_RATIO);
	pidSetIOutputLimits(&positionYRatePID, -posHoldRatePIDLimit * POSITION_CONTROL_RATE_PID_I_LIMIT_RATIO, posHoldRatePIDLimit * POSITION_CONTROL_RATE_PID_I_LIMIT_RATIO);

	//Set D limit
	pidSetDOutputLimits(&positionXRatePID, -posHoldRatePIDLimit * POSITION_CONTROL_RATE_PID_D_LIMIT_RATIO, posHoldRatePIDLimit * POSITION_CONTROL_RATE_PID_D_LIMIT_RATIO);
	pidSetDOutputLimits(&positionYRatePID, -posHoldRatePIDLimit * POSITION_CONTROL_RATE_PID_D_LIMIT_RATIO, posHoldRatePIDLimit * POSITION_CONTROL_RATE_PID_D_LIMIT_RATIO);

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
	positionControlDOBExecutedX = 0.0f;
	positionControlDOBExecutedY = 0.0f;
}

float xDobTest , yDobTest;
float velFFXTest , velFFYTest;

void controlPositionRateWithGains(float dt, float ratePGain, float rateIGain, float rateDGain) {
	/*---------------- Position → Velocity target ----------------*/
	float velocityTargetX = positionXPID.pid;
	float velocityTargetY = positionYPID.pid;

#if POSITION_CONTROL_VEL_FF_ENABLED == 1
	float velFFX = (velocityTargetX * POSITION_CONTROL_VEL_FF_GAIN);
	float velFFY = (velocityTargetY * POSITION_CONTROL_VEL_FF_GAIN);
	velFFXTest = velFFX;
	velFFYTest = velFFY;
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
	float expectedAccX = positionControlDOBExecutedX * POSITION_CONTROL_DOB_ACCEL_MODEL_K;
	float expectedAccY = positionControlDOBExecutedY * POSITION_CONTROL_DOB_ACCEL_MODEL_K;

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

	xDobTest = xComp;
	yDobTest = yComp;
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
	outputX = constrainToRangeF(outputX, -posHoldRatePIDLimit, posHoldRatePIDLimit);
	outputY = constrainToRangeF(outputY, -posHoldRatePIDLimit, posHoldRatePIDLimit);

#if DISABLE_POSITION_CONTROL_FOR_DEBUG == 1
	controlData.positionXControl = 0;
	controlData.positionYControl = 0;
#else
	controlData.positionXControl = outputX;
	controlData.positionYControl = outputY;
#endif

	// 2. Capture the absolute final output running to the mixer for the next loop iteration
	positionControlDOBExecutedX = controlData.positionXControl;
	positionControlDOBExecutedY = controlData.positionYControl;
}

__ATTR_ITCM_TEXT
void setExpectedPositionVelocity(float dt, float expectedVelX, float expectedVelY) {
	positionXPID.pid = constrainToRangeF(expectedVelX, -posHoldPIDLimit, posHoldPIDLimit);
	positionYPID.pid = constrainToRangeF(expectedVelY, -posHoldPIDLimit, posHoldPIDLimit);
}

__ATTR_ITCM_TEXT
void controlPositionCordinatesWithGains(float dt, float expectedX, float expectedY, float masterPGain) {
	pidUpdateWithGains(&positionXPID, positionCordinateData.xPosition, expectedX, dt, masterPGain, 0.0f, 0.0f);
	pidUpdateWithGains(&positionYPID, positionCordinateData.yPosition, expectedY, dt, masterPGain, 0.0f, 0.0f);
}
