#include "AltitudeControl.h"

#include <math.h>
#include <sys/_stdint.h>

#include "../../calibration/Calibration.h"
#include "../../logger/Logger.h"
#include "../../managers/position/common/PositionCommon.h"
#include "../../memory/Memory.h"
#include "../../status/FCStatus.h"
#include "../../util/MathUtil.h"
#include "../ControlData.h"
#include "../Pid.h"

PID altPID;
PID altRatePID;

float altMasterPLimit = 0;
float altRateILimit = 0;
float altControlLimit = 0;
float altControlZDisturbanceEstimate = 0.0f;

// Lag-filtered expectation of what the previous throttle output should have
// produced as vertical acceleration. Feeds the disturbance observer.
float altControlExpectedAccFilt = 0.0f;

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

	//Overall Limit
	altControlLimit = get10XScaledCalibrationValue(CALIB_PROP_ALT_HOLD_CONTROL_LIMIT_ADDR);
	char buf[64];
	sprintf(buf, "ACL:%f , RCL:%f ,FCL:%f\n", altMasterPLimit, altRateILimit, altControlLimit);
	logString(buf);

	altControlExpectedAccFilt = 0.0f;

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
		controlData.altitudeControl = 0;
	}
	altControlZDisturbanceEstimate = 0.0f;
	altControlExpectedAccFilt = 0.0f;
	controlData.altitudeDOBControl = 0;
	pidResetI(&altPID);
	pidResetI(&altRatePID);
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

/* Clear ONLY the disturbance state. altControlExpectedAccFilt is deliberately
 * NOT cleared: it tracks commanded throttle and stays valid across a stick
 * transition. Zeroing it while throttle is ~480 would make the next sample
 * compute disturbance against a zero expectation - a large false reading. */
void resetAltitudeDOBControl(void) {
	altControlZDisturbanceEstimate = 0.0f;
	controlData.altitudeDOBControl = 0.0f;
}

__ATTR_ITCM_TEXT
void controlAltitudeAltWithGains(float dt, float expectedAltitude, float currentAltitude, ALTITUDE_CONTROL_GAINS altControlGains) {
	pidUpdateWithGains(&altPID, currentAltitude, expectedAltitude, dt, altControlGains.masterPGain, 0.0f, 0.0f);
}


__ATTR_ITCM_TEXT
void setExpectedAltitudeVelocity(float dt, float expectedVel) {
	altPID.pid =constrainToRangeF(expectedVel, -altMasterPLimit, altMasterPLimit);
}

#if ALT_CONTROL_ACC_DISTURBANCE_EST_ENABLED == 1
__ATTR_ITCM_TEXT
static void updateAltitudeDOB(float thrustGain, float dt, ALTITUDE_CONTROL_GAINS altControlGains) {
	/* What acceleration SHOULD the previous throttle output have produced?
	 * (inverse plant model - not the setpoint, which would just re-add the
	 *  acc PID's own error).
	 * NOTE this is still the SELF-REFERENTIAL form. The measured-better form
	 *      expectedAcc = (throttleControl * cos(pitch)*cos(roll) - hoverThrottle) / thrustGain
	 * dropped the DOB/output correlation from 0.80 to 0.10 in flight logs.
	 * Left as-is deliberately during the descent work - restore it after. */
	float expectedAcc = (thrustGain > 0.001f) ? (controlData.altitudeControl / thrustGain) : 0.0f;
	/* Thrust does not arrive instantly. Without this lag model the DOB reads
	 * its own actuator lag as an external disturbance and amplifies its own
	 * commands - the same failure the position DOB had before ATT_TAU. */
	float alphaThrust = dt / (ALT_CONTROL_THRUST_TAU + dt);
	altControlExpectedAccFilt += alphaThrust * (expectedAcc - altControlExpectedAccFilt);
	/* Anything the model cannot explain is external force: wind, sag,
	 * ground effect, payload change. */
	float disturbance = positionCordinateData.zAcceleration - altControlExpectedAccFilt;
	disturbance = constrainToRangeF(disturbance, -ALT_CONTROL_DOB_ACC_LIMIT, ALT_CONTROL_DOB_ACC_LIMIT);
	/* Pilot authority: dobGain -> 0 while the throttle stick is active, so the
	 * estimate's TARGET becomes 0 and the estimate DRAINS instead of learning
	 * a descent-regime modelling error (drag balances the thrust deficit, so
	 * measured zAcc returns to ~0 while the model still predicts a sustained
	 * negative - booked as a false upward disturbance that fights the pilot).
	 * Draining, not freezing: a frozen estimate keeps applying a stale value
	 * into a muted control loop, which is unrecoverable if it is wrong. */
	float disturbanceTarget = disturbance * altControlGains.dobGain;
	float alphaDist = dt / (ALT_CONTROL_ACC_DISTURBANCE_TAU + dt);
	altControlZDisturbanceEstimate += alphaDist * (disturbanceTarget - altControlZDisturbanceEstimate);
	/* Cancel it, converted back into throttle units by the same model.
	 * NEGATIVE: the mixer ADDS altitudeDOBControl, so the cancelling sign now
	 * lives here - it used to be carried by "output -= ...". Without the minus
	 * the observer REINFORCES every disturbance it sees. */
	float dobOutput = -altControlZDisturbanceEstimate * thrustGain * ALT_CONTROL_ACC_DISTURBANCE_FF_GAIN;
	controlData.altitudeDOBControl = constrainToRangeF(dobOutput, -ALT_CONTROL_DOB_OUTPUT_LIMIT, ALT_CONTROL_DOB_OUTPUT_LIMIT);
}
#endif

/*
 * Back-calculation anti-windup for the altitude-rate loop.
 *
 * The rate PID output is converted to throttle units using thrustGain.
 * If the resulting throttle command saturates, the saturation error is
 * converted back into PID-output units and fed back into the integrator.
 *
 * This prevents the rate-loop I term from continuing to accumulate while
 * the altitude control output is already at its actuator limit.
 */
__ATTR_ITCM_TEXT
static float altitudeControlApplyRateSaturation(float output, float thrustGain, float dt) {
	float outputClamped = constrainToRangeF(output, -altControlLimit, altControlLimit);
	float saturationError = outputClamped - output;
	if (fabsf(saturationError) < 0.0001f || thrustGain <= 0.001f || dt <= 0.0f) {
		return outputClamped;
	}
	float integratorCorrection = (saturationError / thrustGain) * ALT_CONTROL_RATE_PID_I_ANTIWINDUP_GAIN * dt;
	altRatePID.i = constrainToRangeF(altRatePID.i + integratorCorrection, altRatePID.limitIMin, altRatePID.limitIMax);
	return outputClamped;
}

__ATTR_ITCM_TEXT
void controlAltitudeVelWithGains(float dt, ALTITUDE_CONTROL_GAINS altControlGains) {
	pidUpdateWithGains(&altRatePID, positionCordinateData.zVelocity, altPID.pid, dt, altControlGains.ratePGain, altControlGains.rateIGain, altControlGains.rateDGain);
	float thrustGain = fcStatusData.hoverThrottle / GRAVITY_MSS; /* K */
	float output = altRatePID.pid * thrustGain;
	/* ---- 2. Disturbance observer ----------------------------------------- */
#if ALT_CONTROL_ACC_DISTURBANCE_EST_ENABLED == 1
	updateAltitudeDOB(thrustGain, dt, altControlGains);
#endif
	/* ---- 3. Output limit -------------------------------------------------- */
	output = altitudeControlApplyRateSaturation(output, thrustGain, dt);
	controlData.altitudeControl = output;
	controlData.altitudeControlDt = dt;
}

