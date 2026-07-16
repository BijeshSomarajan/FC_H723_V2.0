#ifndef SRC_FC_MANAGERS_POSITION_ESTIMATOR_VENTURIBIASESTIMATOR_C_
#define SRC_FC_MANAGERS_POSITION_ESTIMATOR_VENTURIBIASESTIMATOR_C_

#include "../../../util/MathUtil.h"
#include "../../../util/CommonUtil.h"
#include "../../../sensors/rc/RCSensor.h"
#include "../../../status/FCStatus.h"
#include "../../../memory/Memory.h"
#include "../../../FCConfig.h"
#include "../../../sensors/attitude/AttitudeSensor.h"
#include "../../../calibration/Calibration.h"
#include "../../../dsp/LowPassFilter.h"
#include "../common/PositionCommon.h"   /* positionCordinateData - ADJUST PATH IF NEEDED */

#include "VenturiBiasEstimator.h"

VENTURI_ESTIMATE_DATA venturiEstimateData;
LOWPASSFILTER venturiBiasLPF;

uint8_t initVenturiBiasEstimator(void) {
	lowPassFilterInit(&venturiBiasLPF, VENTURI_EST_BIAS_LPF_FREQ);
	resetVenturiBiasEstimator();
	return 1;
}

/**
 * @brief Signed still-air speed model for ONE body axis.
 *
 * Restores the original signed integration semantics (v2 magnitude model
 * broke braking: reverse tilt must DECELERATE the state, not accelerate it):
 *  - tilt accelerates the state in its own sign
 *  - drag opposes current speed
 *  - zero-cross clamp prevents braking overshoot through zero
 *  - deadband bleed drains residual speed when the axis is level
 */
__ATTR_ITCM_TEXT
static float venturiIntegrateAxisSpeed(float speed, float tiltDeg, float dt) {
	tiltDeg = applyDeadBandFloat(0.0f, tiltDeg, VENTURI_EST_PITCH_ANGLE_MIN);
	tiltDeg = constrainToRangeF(tiltDeg, -VENTURI_EST_PITCH_ANGLE_MAX, VENTURI_EST_PITCH_ANGLE_MAX);

	float tiltAccel = tanApprox(convertDegToRadF(tiltDeg)) * GRAVITY_MSS * VENTURI_EST_ACCEL_GAIN;
	float drag = speed * VENTURI_EST_DRAG_GAIN;

	float prevSpeed = speed;
	speed += (tiltAccel - drag) * dt;

	/* Zero-cross braking clamp: if tilt opposes motion and the step crossed
	 * zero, stop at zero instead of building speed in the opposite sense. */
	if ((prevSpeed > 0.0f && tiltAccel < 0.0f && speed <= 0.0f) || (prevSpeed < 0.0f && tiltAccel > 0.0f && speed >= 0.0f)) {
		speed = 0.0f;
	}

	/* Deadband bleed when this axis is level */
	if (tiltDeg == 0.0f) {
		speed -= (speed * VENTURI_EST_DAMPING_GAIN * dt);
		if (fabsf(speed) < 0.001f) {
			speed = 0.0f;
		}
	}

	return constrainToRangeF(speed, -VENTURI_EST_SPEED_MAX, VENTURI_EST_SPEED_MAX);
}

/**
 * @brief Estimates the Venturi/Bernoulli baro pressure-drop bias [m].
 *
 * v3 changes:
 *  - Signed per-axis speed states (pitch -> longitudinal, roll -> lateral),
 *    combined as a magnitude only at the bias stage. Braking now decays the
 *    state correctly; crosswind/lateral flight is modeled.
 *  - The EKF ground-speed gate is applied ONLY when the XY velocity estimate
 *    is actually valid (GNSS/flow lock). In baro-only mode the gate is
 *    bypassed - previously it silently zeroed all compensation there.
 *
 * The returned value is intended to be SUBTRACTED directly from the baro
 * measurement before EKF fusion (see updateZPositionSL), not fused as a
 * BP pseudo-measurement.
 */
__ATTR_ITCM_TEXT
float getVenturiBiasEstimate(float dt) {
	/* 1. Safety guard: reset and bypass if not flying */
	if (!fcStatusData.canFly || fcStatusData.throttlePercent <= fcStatusData.liftOffThrottlePercent) {
		resetVenturiBiasEstimator();
		return 0.0f;
	}
	/* 2. Per-axis signed speed model */
	venturiEstimateData.lateralSpeed = venturiIntegrateAxisSpeed(venturiEstimateData.lateralSpeed, sensorAttitudeData.pitch, dt);
	venturiEstimateData.lateralSpeedY = venturiIntegrateAxisSpeed(venturiEstimateData.lateralSpeedY, sensorAttitudeData.roll, dt);
	float speedMag = fastSqrtf((venturiEstimateData.lateralSpeed * venturiEstimateData.lateralSpeed) + (venturiEstimateData.lateralSpeedY * venturiEstimateData.lateralSpeedY));
	/* 3. Ground-speed gate - ONLY when the XY estimate is trustworthy.
	 * Map VENTURI_EST_XY_VEL_VALID() in the header to your actual validity
	 * flag (GNSS lock / optical-flow healthy). In pure baro mode it must
	 * evaluate to 0 so the tilt model passes through un-gated. */
	float airspeedProxy = speedMag;
#if VENTURI_EST_USE_EKF_SPEED == 1
	if (fcStatusData.isNavModeActive) {
		float ekfSpeed = fastSqrtf((positionCordinateData.xVelocity * positionCordinateData.xVelocity) + (positionCordinateData.yVelocity * positionCordinateData.yVelocity));
		airspeedProxy = fminf(airspeedProxy, ekfSpeed);
	}
#endif
	venturiEstimateData.speedMagnitude = airspeedProxy; /* telemetry */
	/* 4. Quadratic Bernoulli bias translation + clamp */
	float bias = (airspeedProxy * airspeedProxy) * VENTURI_EST_BIAS_GAIN;
	bias = constrainToRangeF(bias, 0.0f, VENTURI_EST_BIAS_VALUE_MAX);
	/* 5. LPF smoothing to match pneumatic lag */
	venturiEstimateData.venturiBias = lowPassFilterUpdate(&venturiBiasLPF, bias, dt);
	return venturiEstimateData.venturiBias;
}

void resetVenturiBiasEstimator(void) {
	venturiEstimateData.venturiBias = 0.0f;
	venturiEstimateData.lateralSpeed = 0.0f;
	venturiEstimateData.lateralSpeedY = 0.0f;
	venturiEstimateData.speedMagnitude = 0.0f;
	venturiEstimateData.pitchAngleAbsFiltered = 0.0f;
	lowPassFilterReset(&venturiBiasLPF);
}

#endif
