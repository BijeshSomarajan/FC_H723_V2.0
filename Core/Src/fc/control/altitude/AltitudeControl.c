#include "AltitudeControl.h"

#include <sys/_stdint.h>

#include "../../calibration/Calibration.h"
#include "../../memory/Memory.h"
#include "../../managers/position/PositionManager.h"
#include "../ControlData.h"
#include "../Pid.h"

PID altPID;
PID altRatePID;
PID altAccPID;
uint8_t altControlAccEnabled = 0;
float altRateLimit  = 0;
uint8_t initAltitudeControl() {
	/** Master PID **/
	pidInit(&altPID, getScaledCalibrationValue(CALIB_PROP_ALT_HOLD_PID_KP_ADDR), 0, 0, 0);
	pidSetPIDOutputLimits(&altPID, -getCalibrationValue(CALIB_PROP_ALT_HOLD_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_ALT_HOLD_PID_LIMIT_ADDR));
	pidSetPOutputLimits(&altPID, -getCalibrationValue(CALIB_PROP_ALT_HOLD_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_ALT_HOLD_PID_LIMIT_ADDR));

	/** Rate PID **/
	pidInit(&altRatePID, getScaledCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_KP_ADDR), getScaledCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_KI_ADDR), getScaledCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_KD_ADDR), ALT_CONTROL_RATE_PID_D_LPF_FREQ);
	pidSetPIDOutputLimits(&altRatePID, -getCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_LIMIT_ADDR));
	pidSetPOutputLimits(&altRatePID, -getCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_LIMIT_ADDR));

	altRateLimit = getCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_LIMIT_ADDR) * ALT_CONTROL_PID_I_LIMIT_RATIO;
	pidSetIOutputLimits(&altRatePID, -altRateLimit, altRateLimit);
	pidSetDOutputLimits(&altRatePID, -getCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_LIMIT_ADDR) * ALT_CONTROL_RATE_PID_D_LIMIT_RATIO, getCalibrationValue(CALIB_PROP_ALT_HOLD_RATE_PID_LIMIT_ADDR) * ALT_CONTROL_RATE_PID_D_LIMIT_RATIO);

	/** Acc PID - Enabled only if set**/
	if (getScaledCalibrationValue(CALIB_PROP_ALT_HOLD_ACC_PID_KP_ADDR) > 0) {
		altControlAccEnabled = 1;
		pidInit(&altAccPID, getScaledCalibrationValue(CALIB_PROP_ALT_HOLD_ACC_PID_KP_ADDR), 0, getScaledCalibrationValue(CALIB_PROP_ALT_HOLD_ACC_PID_KD_ADDR), ALT_CONTROL_ACC_PID_D_LPF_FREQ);
		pidSetPIDOutputLimits(&altAccPID, -getCalibrationValue(CALIB_PROP_ALT_HOLD_ACC_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_ALT_HOLD_ACC_PID_LIMIT_ADDR));
		pidSetPOutputLimits(&altAccPID, -getCalibrationValue(CALIB_PROP_ALT_HOLD_ACC_PID_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_ALT_HOLD_ACC_PID_LIMIT_ADDR));
		pidSetDOutputLimits(&altAccPID, -getCalibrationValue(CALIB_PROP_ALT_HOLD_ACC_PID_LIMIT_ADDR) * ALT_CONTROL_ACC_PID_D_LIMIT_RATIO, getCalibrationValue(CALIB_PROP_ALT_HOLD_ACC_PID_LIMIT_ADDR) * ALT_CONTROL_ACC_PID_D_LIMIT_RATIO);
	}

	resetAltitudeControl(1);
	return 1;
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
	pidResetI(&altPID);
	pidResetI(&altRatePID);
	pidResetI(&altAccPID);
}

void suspendAltitudeRateIControl() {
	altRatePID.suspendITerm = 1;
}

void resumeAltitudeRateIControl() {
	altRatePID.suspendITerm = 0;
}

void setAltitudeRateIControl(float value) {
    altRatePID.i = constrainToRangeF(value,-altRateLimit, altRateLimit);
}

void resetAltitudeRateControl() {
	pidReset(&altRatePID);
	pidResetI(&altRatePID);
}

void resetAltitudeRateIControl() {
	pidResetI(&altRatePID);
}

void resetAltitudeMasterControl() {
	pidReset(&altPID);
	pidResetI(&altPID);
}

__ATTR_ITCM_TEXT
void controlAltitude(float dt, float expectedAltitude,float currentAltitude) {
	pidUpdate(&altPID, currentAltitude, expectedAltitude, dt);
	pidUpdate(&altRatePID, positionData.zVelocity, altPID.pid, dt);
	if (altControlAccEnabled == 1) {
		pidUpdate(&altAccPID, positionData.zAcceleration, altRatePID.pid, dt);
		controlData.altitudeControl = altAccPID.pid;
	} else {
		controlData.altitudeControl = altRatePID.pid;
	}
	controlData.altitudeControl = 0;
	controlData.altitudeControlDt = dt;
}

__ATTR_ITCM_TEXT
void controlAltitudeWithGains(float dt, float expectedAltitude, float currentAltitude, ALTITUDE_CONTROL_GAINS altControlGains) {
	pidUpdateWithGains(&altPID, currentAltitude, expectedAltitude, dt, altControlGains.masterPGain, 0.0f, 0.0f);
	pidUpdateWithGains(&altRatePID, positionData.zVelocity, altPID.pid, dt, altControlGains.ratePGain, altControlGains.rateIGain, altControlGains.rateDGain);
	if (altControlAccEnabled == 1) {
		pidUpdateWithGains(&altAccPID, positionData.zAcceleration, altRatePID.pid, dt, altControlGains.accPGain, 0.0f, altControlGains.accDGain);
		controlData.altitudeControl = altAccPID.pid;
	} else {
		controlData.altitudeControl = altRatePID.pid;
	}
    //controlData.altitudeControl = 0;
	controlData.altitudeControlDt = dt;
}

