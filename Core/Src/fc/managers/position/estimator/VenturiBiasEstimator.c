#ifndef SRC_FC_MANAGERS_POSITION_ESTIMATOR_VENTURIBIASESTIMATOR_C_
#define SRC_FC_MANAGERS_POSITION_ESTIMATOR_VENTURIBIASESTIMATOR_C_

#include "../../position/estimator/VenturiBiasEstimator.h"

#include "../../../util/MathUtil.h"
#include "../../../util/CommonUtil.h"
#include "../../../sensors/rc/RCSensor.h"
#include "../../../status/FCStatus.h"
#include "../../../memory/Memory.h"
#include "../../../FCConfig.h"
#include "../../../sensors/attitude/AttitudeSensor.h"
#include "../../../calibration/Calibration.h"
#include "../../../dsp/LowPassFilter.h"


VENTURI_ESTIMATE_DATA venturiEstimateData;
LOWPASSFILTER venturiBiasLPF;

uint8_t initVenturiBiasEstimator(void) {
	lowPassFilterInit(&venturiBiasLPF, VENTURI_EST_BIAS_LPF_FREQ);
	resetVenturiBiasEstimator();
	return 1;
}

/*
 * One axis of the speed model. Identical maths to the original single-axis
 * version, operating on whichever state pair is handed in.
 * angleDeg : already deadbanded and clamped, signed, degrees
 * speed    : in/out signed speed state for this axis [m/s]
 * dwell    : in/out brake dwell timer for this axis  [s]
 */

__ATTR_ITCM_TEXT
static void venturiUpdateAxis(float angleDeg, float *speed, float *dwell, float dt) {
	/* 1. Signed acceleration mapping */
	float lateralAccel = tanApprox(convertDegToRadF(angleDeg)) * GRAVITY_MSS * VENTURI_EST_ACCEL_GAIN;

	/* 2. Drag and integration */
	//float drag = (*speed) * VENTURI_EST_DRAG_GAIN;
	float drag = VENTURI_EST_DRAG_GAIN_Q * (*speed) * fabsf(*speed);

	float acceleration = lateralAccel - drag;
	float prevSpeed = *speed;
	*speed += (acceleration * dt);

	/* 3. Zero-cross braking protection: acceleration opposing current travel */
	if ((prevSpeed > 0.0f && lateralAccel < 0.0f) || (prevSpeed < 0.0f && lateralAccel > 0.0f)) {
		if (((prevSpeed > 0.0f && *speed <= 0.0f) || (prevSpeed < 0.0f && *speed >= 0.0f)) && fabsf(prevSpeed) > VENTURI_EST_BRAKE_ARM_SPEED) {
			*speed = 0.0f;
			*dwell = VENTURI_EST_BRAKE_DWELL;
		}
	}

	/* 3b. Dwell hold: a zero-cross during braking means this axis stopped.
	 *     Hold at zero so continued brake tilt is not read as reverse flight. */
	if (*dwell > 0.0f) {
		*dwell -= dt;
		*speed = 0.0f;
	}

	/* 4. Deadband drain: tilt inside the deadband -> bleed the speed memory */
	if (angleDeg == 0.0f) {
		*speed -= ((*speed) * VENTURI_EST_DAMPING_GAIN * dt);
		if (fabsf(*speed) < 0.001f) {
			*speed = 0.0f;
		}
	}

	/* 5. Runaway clamp */
	*speed = constrainToRangeF(*speed, -VENTURI_EST_SPEED_MAX, VENTURI_EST_SPEED_MAX);
}

__ATTR_ITCM_TEXT
float getVenturiBiasEstimate(float dt) {
	/* 1. Safety guard: reset and bypass if the vehicle cannot fly or is below
	 *    the liftoff threshold */
	if (!fcStatusData.canFly || fcStatusData.throttlePercent <= fcStatusData.liftOffThrottlePercent) {
		resetVenturiBiasEstimator();
		return 0.0f;
	}

	/* 2. Condition both tilt inputs identically */
	float imuPitch = applyDeadBandFloat(0.0f, sensorAttitudeData.pitch, VENTURI_EST_PITCH_ANGLE_MIN);
	imuPitch = constrainToRangeF(imuPitch, -VENTURI_EST_PITCH_ANGLE_MAX, VENTURI_EST_PITCH_ANGLE_MAX);

	float imuRoll = applyDeadBandFloat(0.0f, sensorAttitudeData.roll, VENTURI_EST_ROLL_ANGLE_MIN);
	imuRoll = constrainToRangeF(imuRoll, -VENTURI_EST_ROLL_ANGLE_MAX, VENTURI_EST_ROLL_ANGLE_MAX);

	/* 3. Advance both speed states independently */
	venturiUpdateAxis(imuPitch, &venturiEstimateData.lateralSpeedPitch, &venturiEstimateData.brakeDwellPitch, dt);
	venturiUpdateAxis(imuRoll, &venturiEstimateData.lateralSpeedRoll, &venturiEstimateData.brakeDwellRoll, dt);

	/* 4. Quadratic Bernoulli translation on the speed MAGNITUDE.
	 *    speedSq = |v|^2 = vPitch^2 + vRoll^2 - no sqrt needed, the bias is
	 *    quadratic in speed anyway. */
	float vP = venturiEstimateData.lateralSpeedPitch;
	float vR = venturiEstimateData.lateralSpeedRoll;
	float speedSq = (vP * vP) + (vR * vR);

	venturiEstimateData.lateralSpeedMag = fastSqrtf(speedSq);   /* logging only */

	float bias = speedSq * VENTURI_EST_BIAS_GAIN;
	bias = constrainToRangeF(bias, 0.0f, VENTURI_EST_BIAS_VALUE_MAX);

	/* 5. Output LPF (pneumatic settling) */
	venturiEstimateData.venturiBias = lowPassFilterUpdate(&venturiBiasLPF, bias, dt);
	return venturiEstimateData.venturiBias;
}


void resetVenturiBiasEstimator(void) {
	venturiEstimateData.venturiBias = 0.0f;
	venturiEstimateData.lateralSpeedPitch = 0.0f;
	venturiEstimateData.brakeDwellPitch = 0.0f;
	venturiEstimateData.lateralSpeedRoll = 0.0f;
	venturiEstimateData.brakeDwellRoll = 0.0f;
	lowPassFilterReset(&venturiBiasLPF);
}

#endif
