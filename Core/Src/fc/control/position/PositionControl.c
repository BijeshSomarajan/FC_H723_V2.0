#include "PositionControl.h"

#include "../../calibration/Calibration.h"
#include "../../managers/position/common/PositionCommon.h"
#include "../../memory/Memory.h"
#include "../../util/MathUtil.h"
#include "../ControlData.h"
#include "../Pid.h"

PID positionXPID, positionYPID, positionXRatePID, positionYRatePID;
float positionControlXVelDist = 0.0f, positionControlYVelDist = 0.0f;
float positionControlXAccDist = 0.0f, positionControlYAccDist = 0.0f;
float posHoldRatePIDLimit, posHoldPIDLimit;
float previousEffectivePositionControlXw = 0.0f;
float previousEffectivePositionControlYw = 0.0f;
float dobExpectedAccXFilt = 0.0f, dobExpectedAccYFilt = 0.0f;
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
		positionXRatePID.pInput = positionCordinateData.xVelocity;
		positionYRatePID.pInput = positionCordinateData.yVelocity;
		controlData.positionXControl = 0;
		controlData.positionYControl = 0;
	}

	pidResetI(&positionXRatePID);
	pidResetI(&positionYRatePID);
	positionControlXVelDist = 0.0f;
	positionControlYVelDist = 0.0f;
	positionControlXAccDist = 0.0f;
	positionControlYAccDist = 0.0f;
	/* Observer describes the airframe, not the controller. Keep it current
	 * instead of zeroing, so distAcc starts at zero on every re-entry. */
	dobExpectedAccXFilt = positionCordinateData.xAcceleration;
	dobExpectedAccYFilt = positionCordinateData.yAcceleration;

}

/* Tilt command that, per the DOB plant model, produces a commanded
 * acceleration. accelCmd is earth-frame m/s^2 (signed). Returns a control
 * value in the same units as positionXControl, i.e. degrees of tilt.
 *
 * This is the term the rate PID integrator otherwise has to manufacture by
 * accumulating velocity error across the whole brake - which is what
 * overshoots when the velocity target reaches zero. Supplying it directly
 * keeps the integrator near zero and the loop out of angle saturation. */
__ATTR_ITCM_TEXT
float positionControlAccelFF(float accelCmd) {
#if POSITION_CONTROL_ACCEL_FF_ENABLED == 1
	float k = POSITION_CONTROL_DOB_ACCEL_MODEL_K;
	if (k < 1e-6f) {
		return 0.0f; /* guard: never divide by a zero model */
	}
	return (accelCmd / k) * POSITION_CONTROL_ACCEL_FF_GAIN;
#else
	(void)accelCmd;
	return 0.0f;
#endif
}


/* Single-axis disturbance observer.
 *
 * Estimates an external velocity+acceleration disturbance and returns the
 * compensating control delta to SUBTRACT from that axis' rate output.
 *
 *   velDist : slow-filtered velocity error state  (per-axis, persistent)
 *   accDist : filtered (measured - modelled) accel  (per-axis, persistent)
 *   measVel : this axis' measured velocity
 *   velTarget : this axis' velocity setpoint
 *   measAcc : this axis' measured acceleration
 *   prevCtrl: last effective control on this axis (drives the plant model)
 *
 * Pure extraction of the former inline block: identical math, identical
 * state updates, run once per axis. Returns the value that was previously
 * computed inside the `outputX -= (...)` expression. */
__ATTR_ITCM_TEXT
static float positionControlDOBAxis(float *velDist, float *accDist, float measVel, float velTarget, float measAcc, float prevCtrl, float *dobExpectedAccFilt, float dt) {
	// Velocity Disturbance
	float velErr = measVel - velTarget;
	float alphaVel = dt / (POSITION_CONTROL_DOB_VEL_TAU + dt);

	*velDist += alphaVel * (velErr - *velDist);
	*velDist = constrainToRangeF(*velDist, -POSITION_CONTROL_DOB_STATE_LIMIT, POSITION_CONTROL_DOB_STATE_LIMIT);

	// Acceleration Disturbance
	float alphaAtt = dt / (POSITION_CONTROL_DOB_ATT_TAU + dt);
	*dobExpectedAccFilt += alphaAtt * ((prevCtrl * POSITION_CONTROL_DOB_ACCEL_MODEL_K) - *dobExpectedAccFilt);
	float distAcc = constrainToRangeF(measAcc - *dobExpectedAccFilt, -POSITION_CONTROL_DOB_ACC_LIMIT, POSITION_CONTROL_DOB_ACC_LIMIT);

	float alphaAcc = dt / (POSITION_CONTROL_DOB_ACC_TAU + dt);
	*accDist += alphaAcc * (distAcc - *accDist);

	// Clamp acceleration state
	*accDist = constrainToRangeF(*accDist, -POSITION_CONTROL_DOB_STATE_LIMIT, POSITION_CONTROL_DOB_STATE_LIMIT);

	// Compensation delta to subtract from the rate output
	return constrainToRangeF((*velDist) * POSITION_CONTROL_DOB_VEL_GAIN + (*accDist) * POSITION_CONTROL_DOB_ACC_GAIN, -POSITION_CONTROL_DOB_OUTPUT_LIMIT, POSITION_CONTROL_DOB_OUTPUT_LIMIT);
}


/* Joint (2D) vector saturation with back-calculation anti-windup.
 *
 * The rate output is a vector; its MAGNITUDE is limited to posHoldRatePIDLimit,
 * and the clipped-off part bleeds each axis' rate integrator. This is
 * inherently two-axis - the magnitude couples X and Y - so it stays one call
 * rather than a per-axis helper. Operates in place on the two output refs and
 * mutates the two rate PID integrators exactly as the former inline block. */
__ATTR_ITCM_TEXT
static void positionControlApplyRateSaturation(float *outX, float *outY, float dt) {
	float magSq = ((*outX) * (*outX)) + ((*outY) * (*outY));
	float limit = posHoldRatePIDLimit;
	float limitSq = limit * limit;

	if (magSq > limitSq) {
		float mag = fastSqrtf(magSq);
		// Divide-by-zero protection
		if (mag > 1e-6f) {
			float scale = limit / mag;

			float saturatedX = (*outX) * scale;
			float saturatedY = (*outY) * scale;

			// Back-Calculation (Anti-Windup Bleeding)
			float diffX = saturatedX - (*outX);
			float diffY = saturatedY - (*outY);

			float xRatePidI = positionXRatePID.i + (diffX * POSITION_CONTROL_RATE_PID_I_ANTIWINDUP_GAIN * dt);
			float yRatePidI = positionYRatePID.i + (diffY * POSITION_CONTROL_RATE_PID_I_ANTIWINDUP_GAIN * dt);

			// Hard Integrator Clamp
			positionXRatePID.i = constrainToRangeF(xRatePidI, positionXRatePID.limitIMin, positionXRatePID.limitIMax);
			positionYRatePID.i = constrainToRangeF(yRatePidI, positionYRatePID.limitIMin, positionYRatePID.limitIMax);

			*outX = saturatedX;
			*outY = saturatedY;
		}
	}
}

__ATTR_ITCM_TEXT
void controlPositionRateWithGains(float dt, float ratePGain, float rateIGain, float rateDGain, float accFFX, float accFFY) {
	/*---------------- 1. PID Update ----------------*/
	float velocityTargetX = positionXPID.pid;
	float velocityTargetY = positionYPID.pid;

	float xVelocity = positionCordinateData.xVelocity;
	float yVelocity = positionCordinateData.yVelocity;

	pidUpdateWithGains(&positionXRatePID, xVelocity, velocityTargetX, dt, ratePGain, rateIGain, rateDGain);
	pidUpdateWithGains(&positionYRatePID, yVelocity, velocityTargetY, dt, ratePGain, rateIGain, rateDGain);

	float outputX = positionXRatePID.pid;
	float outputY = positionYRatePID.pid;

	/*---------------- 2. Disturbance Observer (DOB) ----------------*/
#if POSITION_CONTROL_DOB_ENABLED == 1
	outputX -= positionControlDOBAxis(&positionControlXVelDist, &positionControlXAccDist, xVelocity, velocityTargetX, positionCordinateData.xAcceleration, controlData.previousEffectiveXControl, &dobExpectedAccXFilt, dt);
	outputY -= positionControlDOBAxis(&positionControlYVelDist, &positionControlYAccDist, yVelocity, velocityTargetY, positionCordinateData.yAcceleration, controlData.previousEffectiveYControl, &dobExpectedAccYFilt, dt);
#endif

	/*---------------- 3. Feedforward ----------------*/
#if POSITION_CONTROL_VEL_FF_ENABLED == 1
	outputX += (velocityTargetX * POSITION_CONTROL_VEL_FF_GAIN);
	outputY += (velocityTargetY * POSITION_CONTROL_VEL_FF_GAIN);
#endif

#if POSITION_CONTROL_ACCEL_FF_ENABLED == 1
	outputX += positionControlAccelFF(accFFX);
	outputY += positionControlAccelFF(accFFY);
#endif

	/*---------------- 4. Vector Saturation & Anti-Windup ----------------*/
	positionControlApplyRateSaturation(&outputX, &outputY, dt);

	/*---------------- 5. Final Output Assignment ----------------*/

	controlData.positionXControl = outputX;
	controlData.positionYControl = outputY;

	controlData.previousEffectiveXControl = controlData.positionXControl;
	controlData.previousEffectiveYControl = controlData.positionYControl;

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
