#include "AltitudeControl.h"

#include <sys/_stdint.h>

#include "../../calibration/Calibration.h"
#include "../../memory/Memory.h"
#include "../../managers/position/PositionManager.h"
#include "../ControlData.h"
#include "../Pid.h"
#include "../../FCConfig.h"

PID altPID;
PID altRatePID;
PID altAccPID;
float altMasterPLimit = 0;
float altRateILimit = 0;
float altAccPIDLimit = 0;
float altControlZDisturbanceEstimate = 0.0f;

uint8_t initAltitudeControl() {
	/** Master PID **/
	pidInit(&altPID, get1KXScaledCalibrationValue(CALIB_PROP_ALT_HOLD_PID_KP_ADDR), 0, 0, 0);
	pidSetPIDOutputLimits(&altPID, -get10XScaledCalibrationValue(CALIB_PROP_ALT_HOLD_PID_LIMIT_ADDR), get10XScaledCalibrationValue(CALIB_PROP_ALT_HOLD_PID_LIMIT_ADDR));
	altMasterPLimit = get10XScaledCalibrationValue(CALIB_PROP_ALT_HOLD_PID_LIMIT_ADDR);
	pidSetPOutputLimits(&altPID, -altMasterPLimit, altMasterPLimit);

	/** Rate PID **/
	pidInit(&altRatePID, get1KXScaledCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_KP_ADDR), get1KXScaledCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_KI_ADDR), get1KXScaledCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_KD_ADDR), ALT_CONTROL_RATE_PID_D_LPF_FREQ);
	pidSetPIDOutputLimits(&altRatePID, -get10XScaledCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_LIMIT_ADDR), get10XScaledCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_LIMIT_ADDR));
	pidSetPOutputLimits(&altRatePID, -get10XScaledCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_LIMIT_ADDR), get10XScaledCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_LIMIT_ADDR));

	altRateILimit = get10XScaledCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_LIMIT_ADDR) * ALT_CONTROL_RATE_PID_I_LIMIT_RATIO;
	pidSetIOutputLimits(&altRatePID, -altRateILimit, altRateILimit);
	pidSetDOutputLimits(&altRatePID, -get10XScaledCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_LIMIT_ADDR) * ALT_CONTROL_RATE_PID_D_LIMIT_RATIO, get10XScaledCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_LIMIT_ADDR) * ALT_CONTROL_RATE_PID_D_LIMIT_RATIO);

	/** Acc PID **/
	altAccPIDLimit = get10XScaledCalibrationValue(CALIB_PROP_ALT_HOLD_ACC_PID_LIMIT_ADDR);

	pidInit(&altAccPID, get1KXScaledCalibrationValue(CALIB_PROP_ALT_HOLD_ACC_PID_KP_ADDR), 0, get1KXScaledCalibrationValue(CALIB_PROP_ALT_HOLD_ACC_PID_KD_ADDR), ALT_CONTROL_ACC_PID_D_LPF_FREQ);
	pidSetPIDOutputLimits(&altAccPID, -altAccPIDLimit, altAccPIDLimit);
	pidSetPOutputLimits(&altAccPID, -altAccPIDLimit, altAccPIDLimit);
	pidSetDOutputLimits(&altAccPID, -altAccPIDLimit * ALT_CONTROL_ACC_PID_D_LIMIT_RATIO, altAccPIDLimit * ALT_CONTROL_ACC_PID_D_LIMIT_RATIO);

	resetAltitudeControl(1);
	return 1;
}

void resetAltitudeControlMPLimits(void) {
	pidSetPOutputLimits(&altPID, -altMasterPLimit, altMasterPLimit);
}

void resetAltitudeControlRILimits(void) {
	pidSetIOutputLimits(&altRatePID, -altRateILimit, altRateILimit);
}

float getAltitudeControlMPValue() {
	return altPID.p;
}

float getAltitudeControlRIValue() {
	return altRatePID.i;
}

void applyAltitudeControlMPMinLimitToValue(float value) {
	value = constrainToRangeF(value, -altMasterPLimit, altMasterPLimit);
	pidSetPOutputLimits(&altPID, value, altMasterPLimit);
}

void applyAltitudeControlRIMinLimitToValue(float value) {
	value = constrainToRangeF(value, -altRateILimit, altRateILimit);
	pidSetIOutputLimits(&altRatePID, value, altRateILimit);
}

void resetAltitudeControlMaster(void) {
	pidReset(&altPID);
}

void resetAltitudeControlRate(void) {
	pidReset(&altRatePID);
}

void resetAltitudeControl(uint8_t hard) {
	if (hard) {
		pidReset(&altPID);
		pidReset(&altRatePID);
		pidReset(&altAccPID);
		controlData.altitudeControl = 0;
	}
	altControlZDisturbanceEstimate = 0.0f;
	pidResetI(&altPID);
	pidResetI(&altRatePID);
	pidResetI(&altAccPID);
}

void setAltitudeRIControl(float value) {
	altRatePID.i = constrainToRangeF(value, -altRateILimit, altRateILimit);
}

void resetAltitudeRateControl() {
	pidReset(&altRatePID);
	pidResetI(&altRatePID);
}

void resetAltitudeRIControl() {
	pidResetI(&altRatePID);
}

void resetAltitudeMasterControl() {
	pidReset(&altPID);
	pidResetI(&altPID);
}

__ATTR_ITCM_TEXT
void controlAltitudeAltWithGains(float dt, float expectedAltitude, float currentAltitude, ALTITUDE_CONTROL_GAINS altControlGains) {
	pidUpdateWithGains(&altPID, currentAltitude, expectedAltitude, dt, altControlGains.masterPGain, 0.0f, 0.0f);
}

__ATTR_ITCM_TEXT
void controlAltitudeVelWithGains(float dt, ALTITUDE_CONTROL_GAINS altControlGains) {
	pidUpdateWithGains(&altRatePID, positionCordinateData.zVelocity, altPID.pid, dt, altControlGains.ratePGain, altControlGains.rateIGain, altControlGains.rateDGain);

}


__ATTR_ITCM_TEXT
void controlAltitudeAccWithGains(float dt, ALTITUDE_CONTROL_GAINS altControlGains) {
	pidUpdateWithGains(&altAccPID, positionCordinateData.zAcceleration, altRatePID.pid, dt, altControlGains.accPGain, 0.0f, altControlGains.accDGain);
	float output = altAccPID.pid;
#if ALT_CONTROL_VEL_FEED_FWD_ENABLED == 1
	output += altPID.pid * ALT_CONTROL_VEL_FEED_FWD_GAIN; // Feeding back the velocity requited
#endif
#if	ALT_CONTROL_ACC_DISTURBANCE_EST_ENABLED == 1
	float expectedAcc = altRatePID.pid;
	float measuredAcc = positionCordinateData.zAcceleration;
	float disturbance = measuredAcc - expectedAcc;
	float alpha = dt / (ALT_CONTROL_ACC_DISTURBANCE_TAU + dt);
	altControlZDisturbanceEstimate += alpha * (disturbance - altControlZDisturbanceEstimate);
	output -= altControlZDisturbanceEstimate * ALT_CONTROL_ACC_DISTURBANCE_FF_GAIN;
	output = constrainToRangeF(output, -altAccPIDLimit, altAccPIDLimit);
#endif
#if DISABLE_ALT_CONTROL_FOR_DEBUG == 1
    output = 0;
#endif
	controlData.altitudeControl = output;
	controlData.altitudeControlDt = dt;
}
