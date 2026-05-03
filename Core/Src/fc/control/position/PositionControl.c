#include "PositionControl.h"

#include "../../calibration/Calibration.h"
#include "../../managers/position/common/PositionCommon.h"
#include "../../dsp/Interpolator.h"
#include "../../memory/Memory.h"
#include "../ControlData.h"
#include "../Pid.h"

PID positionXPID, positionYPID, positionXRatePID, positionYRatePID;
INTERPOLATOR positionMgrVelXINTRP, positionMgrVelYINTRP;

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

	interpolatorInit(&positionMgrVelXINTRP, 0, 1.0f / rateControlFrequency);
	interpolatorInit(&positionMgrVelYINTRP, 0, 1.0f / rateControlFrequency);

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
	interpolatorReset(&positionMgrVelXINTRP, 0);
	interpolatorReset(&positionMgrVelYINTRP, 0);
}

__ATTR_ITCM_TEXT
void controlPositionRateWithGains(float dt, float ratePGain, float rateIGain, float rateDGain) {

#if POSITION_CONTROL_USE_VEL_INTERPOLATION == 1
	float velocityTargetX = interpolatorUpdate(&positionMgrVelXINTRP, dt);
	float velocityTargetY = interpolatorUpdate(&positionMgrVelYINTRP, dt);
#else
	float velocityTargetX = constrainToRangeF(positionXPID.pid, -POSITION_CONTROL_RATE_VEL_MAX, POSITION_CONTROL_RATE_VEL_MAX) ;
	float velocityTargetY = constrainToRangeF(positionYPID.pid, -POSITION_CONTROL_RATE_VEL_MAX, POSITION_CONTROL_RATE_VEL_MAX) ;
#endif

	pidUpdateWithGains(&positionXRatePID, positionCordinateData.xVelocity, velocityTargetX, dt, ratePGain, rateIGain, rateDGain);
	pidUpdateWithGains(&positionYRatePID, positionCordinateData.yVelocity, velocityTargetY, dt, ratePGain, rateIGain, rateDGain);

	controlData.positionXControl = positionXRatePID.pid;
	controlData.positionYControl = positionYRatePID.pid;
}

__ATTR_ITCM_TEXT
void controlPositionCordinatesWithGains(float dt, float expectedX, float expectedY, float masterPGain) {
	pidUpdateWithGains(&positionXPID, positionCordinateData.xPosition, expectedX, dt, masterPGain, 0.0f, 0.0f);
	pidUpdateWithGains(&positionYPID, positionCordinateData.yPosition, expectedY, dt, masterPGain, 0.0f, 0.0f);

#if POSITION_CONTROL_USE_VEL_INTERPOLATION == 1
	interpolatorSetTarget(&positionMgrVelXINTRP, positionXPID.pid);
	interpolatorSetTarget(&positionMgrVelYINTRP, positionYPID.pid);
#endif
}

__ATTR_ITCM_TEXT
void controlPositionWithGains(float dt, float expectedX, float expectedY, float masterPGain, float ratePGain, float rateIGain, float rateDGain) {
	pidUpdateWithGains(&positionXPID, positionCordinateData.xPosition, expectedX, dt, masterPGain, 0.0f, 0.0f);
	pidUpdateWithGains(&positionXRatePID, positionCordinateData.xVelocity, positionXPID.pid, dt, ratePGain, rateIGain, rateDGain);

	pidUpdateWithGains(&positionYPID, positionCordinateData.yPosition, expectedY, dt, masterPGain, 0.0f, 0.0f);
	pidUpdateWithGains(&positionYRatePID, positionCordinateData.yVelocity, positionYPID.pid, dt, ratePGain, rateIGain, rateDGain);

	controlData.positionXControl = positionXRatePID.pid;
	controlData.positionYControl = positionYRatePID.pid;
}

