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
 * @brief Updates the aerodynamic Venturi/Bernoulli pressure drop compensation bias.
 *
 * REWORKED (wind robustness):
 *  1. Tilt magnitude now uses BOTH pitch and roll, so crosswind / lateral
 *     flight is modeled instead of ignored.
 *  2. The tilt-driven speed model is treated as an unsigned magnitude
 *     (the bias is quadratic in speed, so sign carried no information).
 *  3. The model speed is GATED against the EKF ground speed with fminf().
 *     Rationale:
 *       - Wind hover:      tilt high, ground speed ~0  -> bias ~0 (was: saturated 0.5 m!)
 *       - Downwind drift:  tilt ~0,  ground speed high -> bias ~0 (correct: airspeed ~0)
 *       - Still-air cruise: tilt and ground speed agree -> bias as before
 *       - Upwind cruise:   bias is UNDER-estimated (ground < air speed).
 *         This is deliberately conservative; a wrong-but-small bias is far
 *         less harmful than a wrong-but-saturated one.
 *
 * @param dt Delta time since last execution loop in seconds.
 * @return float Low-pass filtered baro position-bias compensation value [m].
 */
__ATTR_ITCM_TEXT
float getVenturiBiasEstimate(float dt) {
	/* 1. Safety guard: reset and bypass if not flying */
	if (!fcStatusData.canFly || fcStatusData.throttlePercent <= fcStatusData.liftOffThrottlePercent) {
		resetVenturiBiasEstimator();
		return 0.0f;
	}

	/* 2. Tilt magnitude from BOTH axes (deadband + clamp per axis) */
	float pitch = applyDeadBandFloat(0.0f, sensorAttitudeData.pitch, VENTURI_EST_PITCH_ANGLE_MIN);
	float roll  = applyDeadBandFloat(0.0f, sensorAttitudeData.roll,  VENTURI_EST_PITCH_ANGLE_MIN);
	pitch = constrainToRangeF(pitch, -VENTURI_EST_PITCH_ANGLE_MAX, VENTURI_EST_PITCH_ANGLE_MAX);
	roll  = constrainToRangeF(roll,  -VENTURI_EST_PITCH_ANGLE_MAX, VENTURI_EST_PITCH_ANGLE_MAX);

	float tanP = tanApprox(convertDegToRadF(pitch));
	float tanR = tanApprox(convertDegToRadF(roll));
	float tiltAccelMag = fastSqrtf((tanP * tanP) + (tanR * tanR)) * GRAVITY_MSS * VENTURI_EST_ACCEL_GAIN;

	/* 3. Still-air speed model, magnitude only (lateralSpeed is now >= 0) */
	float drag = venturiEstimateData.lateralSpeed * VENTURI_EST_DRAG_GAIN;
	venturiEstimateData.lateralSpeed += (tiltAccelMag - drag) * dt;

	/* 4. Deadband bleed: sticks/attitude level -> drain remaining speed memory */
	if (tiltAccelMag <= 0.0f) {
		venturiEstimateData.lateralSpeed -= (venturiEstimateData.lateralSpeed * VENTURI_EST_DAMPING_GAIN * dt);
	}
	venturiEstimateData.lateralSpeed = constrainToRangeF(venturiEstimateData.lateralSpeed, 0.0f, VENTURI_EST_SPEED_MAX);
	if (venturiEstimateData.lateralSpeed < 0.001f) {
		venturiEstimateData.lateralSpeed = 0.0f;
	}

	/* 5. Ground-speed gate against the EKF horizontal velocity.
	 *    This is the wind-hover fix: tilt alone can no longer generate bias. */
	float airspeedProxy = venturiEstimateData.lateralSpeed;
#if VENTURI_EST_USE_EKF_SPEED == 1
	/* NOTE: verify field names / add your XY-velocity-valid flag here.
	 * If XY velocity can be unavailable (no GNSS/flow), fall through to the
	 * pure tilt model ONLY when the estimate is flagged invalid. */
	float ekfSpeed = fastSqrtf((positionCordinateData.xVelocity * positionCordinateData.xVelocity)
	                         + (positionCordinateData.yVelocity * positionCordinateData.yVelocity));
	airspeedProxy = fminf(airspeedProxy, ekfSpeed);
#endif

	/* 6. Quadratic Bernoulli bias translation + clamp */
	float bias = (airspeedProxy * airspeedProxy) * VENTURI_EST_BIAS_GAIN;
	bias = constrainToRangeF(bias, 0.0f, VENTURI_EST_BIAS_VALUE_MAX);

	/* 7. LPF smoothing to match pneumatic lag */
	venturiEstimateData.venturiBias = lowPassFilterUpdate(&venturiBiasLPF, bias, dt);
	return venturiEstimateData.venturiBias;
}

void resetVenturiBiasEstimator(void) {
	venturiEstimateData.venturiBias = 0.0f;
	venturiEstimateData.lateralSpeed = 0.0f;
	venturiEstimateData.pitchAngleAbsFiltered = 0.0f;
	lowPassFilterReset(&venturiBiasLPF);
}

#endif
