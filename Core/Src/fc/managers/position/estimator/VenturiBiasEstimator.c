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

#include "VenturiBiasEstimator.h"

VENTURI_ESTIMATE_DATA venturiEstimateData;
LOWPASSFILTER venturiPitchAngleLPF, venturiBiasGainLPF, venturiBiasLPF;

uint8_t initVenturiBiasEstimator(void) {
	float liftOffThrottlePercent = (float) getCalibrationValue(CALIB_PROP_RC_LIFTOFF_THROTTLE_ADDR) / (float) MAX_PERMISSIBLE_THROTTLE_DELTA;
	venturiEstimateData.thrustGain = DRONE_MASS_KG * 9.81 / liftOffThrottlePercent;
	venturiEstimateData.thrustGain *= VENTURI_EST_THRUST_GAIN_FACTOR;

	lowPassFilterInit(&venturiPitchAngleLPF, VENTURI_EST_PITCH_ANGLE_LPF_FREQ);
	lowPassFilterInit(&venturiBiasGainLPF, VENTURI_EST_BIAS_GAIN_LPF_FREQ);
	lowPassFilterInit(&venturiBiasLPF, VENTURI_EST_BIAS_LPF_RISE_FREQ);
	resetVenturiBiasEstimator();
	return 1;
}

float updateVenturiBiasEstimatePhysicalOld(float dt) {
	if (fcStatusData.throttlePercent <= fcStatusData.liftOffThrottlePercent) {
		resetVenturiBiasEstimator();
		return 0;
	};

	float imuPitch = constrainToRangeF(applyDeadBandFloat(0, sensorAttitudeData.pitch, VENTURI_EST_PITCH_ANGLE_MIN), -VENTURI_EST_PITCH_ANGLE_MAX, VENTURI_EST_PITCH_ANGLE_MAX);
	float imuPitchAbs = fabsf(imuPitch);
	venturiEstimateData.pitchAngleAbsFiltered = lowPassFilterUpdate(&venturiPitchAngleLPF, imuPitchAbs, dt);
	float imuPitchAbsRadians = convertDegToRadF(venturiEstimateData.pitchAngleAbsFiltered);
	venturiEstimateData.effectiveThrottle = fmaxf(fcStatusData.throttlePercent - fcStatusData.liftOffThrottlePercent, 0.0f);

	float thrust = fastSqrtf(venturiEstimateData.effectiveThrottle) * sinApproxF(imuPitchAbsRadians) * venturiEstimateData.thrustGain * VENTURI_EST_THRUST_GAIN_FACTOR;
	float drag = venturiEstimateData.lateralSpeed * VENTURI_EST_DRAG_FEEDBACK_GAIN;
	float acceleration = thrust - drag;
	venturiEstimateData.lateralSpeed += (acceleration * dt);

	venturiEstimateData.lateralSpeed = constrainToRangeF(venturiEstimateData.lateralSpeed, 0, VENTURI_EST_SPEED_MAX);

	float biasGain = (imuPitch < 0) ? VENTURI_EST_BIAS_GAIN_BWD : VENTURI_EST_BIAS_GAIN_FWD;
	biasGain = lowPassFilterUpdate(&venturiBiasGainLPF, biasGain, dt);
	float bias = constrainToRangeF(venturiEstimateData.lateralSpeed * biasGain, 0, VENTURI_EST_BIAS_VALUE_MAX);

	if (venturiEstimateData.pitchAngleAbsFiltered < VENTURI_EST_PITCH_ANGLE_FADING_TSH && !venturiEstimateData.wasBiasFadingApplied) {
		lowPassFilterSetCutOff(&venturiBiasLPF, VENTURI_EST_BIAS_LPF_FADE_FREQ);
		venturiEstimateData.wasBiasFadingApplied = 1;
	} else if (venturiEstimateData.pitchAngleAbsFiltered > VENTURI_EST_PITCH_ANGLE_FADING_TSH && venturiEstimateData.wasBiasFadingApplied) {
		lowPassFilterSetCutOff(&venturiBiasLPF, VENTURI_EST_BIAS_LPF_RISE_FREQ);
		venturiEstimateData.wasBiasFadingApplied = 0;
	}
	venturiEstimateData.venturiBias = lowPassFilterUpdate(&venturiBiasLPF, bias, dt);

	return venturiEstimateData.venturiBias;
}

__ATTR_ITCM_TEXT
float updateVenturiBiasEstimatePhysical(float dt) {
	// 1. Safety & State Check
	// Removed throttle-based reset to allow the estimator to persist during descents
	if (!fcStatusData.canFly || fcStatusData.throttlePercent <= fcStatusData.liftOffThrottlePercent) {
		resetVenturiBiasEstimator();
		return 0.0f;
	}
	// 2. Input Processing
	float imuPitch = constrainToRangeF(applyDeadBandFloat(0.0f, sensorAttitudeData.pitch, VENTURI_EST_PITCH_ANGLE_MIN), -VENTURI_EST_PITCH_ANGLE_MAX, VENTURI_EST_PITCH_ANGLE_MAX);

	float pitchFiltered = lowPassFilterUpdate(&venturiPitchAngleLPF, imuPitch, dt);
	float pitchRadians = convertDegToRadF(pitchFiltered);

	// 3. Acceleration Modeling (Coordinated Flight Assumption)
	// a = g * tan(theta). thrustGain acts as a scaler/correction factor for the airframe.
	float lateralAccel = tanApprox(pitchRadians) * 9.80665f * VENTURI_EST_THRUST_GAIN_FACTOR;

	// 4. Velocity Integration with Drag
	float drag = venturiEstimateData.lateralSpeed * VENTURI_EST_DRAG_FEEDBACK_GAIN;
	float acceleration = lateralAccel - drag;

	venturiEstimateData.lateralSpeed += (acceleration * dt);
	venturiEstimateData.lateralSpeed = constrainToRangeF(venturiEstimateData.lateralSpeed, -VENTURI_EST_SPEED_MAX, VENTURI_EST_SPEED_MAX);

	// 5. Bernoulli Bias Calculation (Quadratic: Bias = k * v^2)
	float speed = venturiEstimateData.lateralSpeed;
	float biasGain = (pitchFiltered < 0.0f) ? VENTURI_EST_BIAS_GAIN_BWD : VENTURI_EST_BIAS_GAIN_FWD;
	biasGain = lowPassFilterUpdate(&venturiBiasGainLPF, biasGain, dt);

	// v * fabsf(v) preserves the sign so the EKF knows the direction of the pressure dip
	float bias = (speed * fabsf(speed)) * biasGain;
	bias = constrainToRangeF(bias, -VENTURI_EST_BIAS_VALUE_MAX, VENTURI_EST_BIAS_VALUE_MAX);

	// 6. Dynamic Hysteresis (Fading Logic)
	float absPitch = fabsf(pitchFiltered);
	if ((absPitch < VENTURI_EST_PITCH_ANGLE_FADING_TSH) && !venturiEstimateData.wasBiasFadingApplied) {
		lowPassFilterSetCutOff(&venturiBiasLPF, VENTURI_EST_BIAS_LPF_FADE_FREQ);
		venturiEstimateData.wasBiasFadingApplied = 1;
	} else if ((absPitch >= VENTURI_EST_PITCH_ANGLE_FADING_TSH) && venturiEstimateData.wasBiasFadingApplied) {
		lowPassFilterSetCutOff(&venturiBiasLPF, VENTURI_EST_BIAS_LPF_RISE_FREQ);
		venturiEstimateData.wasBiasFadingApplied = 0;
	}

	// 7. Final Filtered Output
	venturiEstimateData.venturiBias = lowPassFilterUpdate(&venturiBiasLPF, bias, dt);
	return venturiEstimateData.venturiBias;
}

float updateVenturiBiasEstimateAlgebraic(float dt) {
	if (fcStatusData.throttlePercent <= fcStatusData.liftOffThrottlePercent) {
		resetVenturiBiasEstimator();
		return 0;
	};

	float imuPitch = constrainToRangeF(applyDeadBandFloat(0, sensorAttitudeData.pitch, VENTURI_EST_PITCH_ANGLE_MIN), -VENTURI_EST_PITCH_ANGLE_MAX, VENTURI_EST_PITCH_ANGLE_MAX);
	float imuPitchAbs = fabsf(imuPitch);
	venturiEstimateData.pitchAngleAbsFiltered = lowPassFilterUpdate(&venturiPitchAngleLPF, imuPitchAbs, dt);
	float imuPitchAbsRadians = convertDegToRadF(venturiEstimateData.pitchAngleAbsFiltered);
	venturiEstimateData.effectiveThrottle = fmaxf(fcStatusData.throttlePercent - fcStatusData.liftOffThrottlePercent, 0.0f);

	float pitchDrag = fastSqrtf(tanApprox(imuPitchAbsRadians)) * VENTURI_EST_PITCH_DRAG_GAIN;
	float aeroDrag = venturiEstimateData.lateralSpeed * VENTURI_EST_AERO_DRAG_FEEDBACK_GAIN;
	float dragSpeed = pitchDrag + aeroDrag;
	float thrustSpeed = venturiEstimateData.effectiveThrottle * sinApproxF(imuPitchAbsRadians) * venturiEstimateData.thrustGain * VENTURI_EST_THRUST_GAIN_FACTOR;
	venturiEstimateData.lateralSpeed = dragSpeed + thrustSpeed;

	venturiEstimateData.lateralSpeed = constrainToRangeF(venturiEstimateData.lateralSpeed, 0, VENTURI_EST_SPEED_MAX);

	float biasGain = (imuPitch < 0) ? VENTURI_EST_BIAS_GAIN_BWD : VENTURI_EST_BIAS_GAIN_FWD;
	biasGain = lowPassFilterUpdate(&venturiBiasGainLPF, biasGain, dt);
	float bias = constrainToRangeF(venturiEstimateData.lateralSpeed * biasGain, 0, VENTURI_EST_BIAS_VALUE_MAX);

	if (venturiEstimateData.pitchAngleAbsFiltered < VENTURI_EST_PITCH_ANGLE_FADING_TSH && !venturiEstimateData.wasBiasFadingApplied) {
		lowPassFilterSetCutOff(&venturiBiasLPF, VENTURI_EST_BIAS_LPF_FADE_FREQ);
		venturiEstimateData.wasBiasFadingApplied = 1;
	} else if (venturiEstimateData.pitchAngleAbsFiltered > VENTURI_EST_PITCH_ANGLE_FADING_TSH && venturiEstimateData.wasBiasFadingApplied) {
		lowPassFilterSetCutOff(&venturiBiasLPF, VENTURI_EST_BIAS_LPF_RISE_FREQ);
		venturiEstimateData.wasBiasFadingApplied = 0;
	}
	venturiEstimateData.venturiBias = lowPassFilterUpdate(&venturiBiasLPF, bias, dt);

	return venturiEstimateData.venturiBias;
}

float updateVenturiBiasEstimate(float dt) {
#if VENTURI_EST_USE_PHYSICAL_MODEL == 1
	return updateVenturiBiasEstimatePhysical(dt);
#else
  return updateVenturiBiasEstimateAlgebraic(dt);
#endif
}

void resetVenturiBiasEstimator(void) {
	venturiEstimateData.venturiBias = 0.0f;
	venturiEstimateData.lateralSpeed = 0.0f;
	venturiEstimateData.pitchAngleAbsFiltered = 0.0f;
	venturiEstimateData.wasBiasFadingApplied = 0;
	lowPassFilterReset(&venturiPitchAngleLPF);
	lowPassFilterReset(&venturiBiasLPF);
	lowPassFilterReset(&venturiBiasGainLPF);
	lowPassFilterSetCutOff(&venturiBiasLPF, VENTURI_EST_BIAS_LPF_RISE_FREQ);
}

#endif
