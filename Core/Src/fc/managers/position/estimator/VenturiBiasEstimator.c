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
LOWPASSFILTER venturiPitchAngleLPF, venturiPitchRCLPF, venturiBiasLPF;

uint8_t initVenturiBiasEstimator(void) {
	float liftOffThrottlePercent = (float) getCalibrationValue(CALIB_PROP_RC_LIFTOFF_THROTTLE_ADDR) / (float) MAX_PERMISSIBLE_THROTTLE_DELTA;
	venturiEstimateData.venturiAccelGain = DRONE_MASS_KG * 9.81 / liftOffThrottlePercent;
	lowPassFilterInit(&venturiPitchAngleLPF, VENTURI_EST_PITCH_ANGLE_LPF_FREQ);
	lowPassFilterInit(&venturiPitchRCLPF, VENTURI_EST_PITCH_RC_LPF_FREQ);
	lowPassFilterInit(&venturiBiasLPF, VENTURI_EST_BIAS_LPF_FREQ);
	return 1;
}

float updateVenturiBiasAerodynamic(float dt) {
	if (fcStatusData.throttlePercent <= fcStatusData.liftOffThrottlePercent) {
		resetVenturiBiasEstimator();
		return 0;
	};

	float pitch = constrainToRangeF(fabsf(sensorAttitudeData.pitch), 0, VENTURI_EST_PITCH_ANGLE_MAX);
	venturiEstimateData.venturiAbsPitchAngleFiltered = lowPassFilterUpdate(&venturiPitchAngleLPF, pitch, dt);

	float rcPitchAbs = constrainToRangeF(fabsf(rcData.RC_EFFECTIVE_DATA[RC_PITCH_CHANNEL_INDEX]), 0, VENTURI_EST_PITCH_RC_MAX);
	venturiEstimateData.ventiriAbsPitchRCFiltered = rcPitchAbs;

	float effectiveAngle = venturiEstimateData.venturiAbsPitchAngleFiltered;
	if (rcPitchAbs > 2.0f && venturiEstimateData.venturiAbsPitchAngleFiltered <= VENTURI_SMALL_ANGLE_TSH) {
		effectiveAngle = VENTURI_SMALL_ANGLE_DEFAULT;
	}

	float estSpeed = fastSqrtf(tanApprox(convertDegToRad(effectiveAngle))) * VENTURI_K_AERO_DRAG_SCALAR;
	venturiEstimateData.lateralVelocity = constrainToRangeF(estSpeed, 0, VENTURI_EST_SPEED_MAX);

	float directionScalar = (sensorAttitudeData.pitch < 0) ? 0.85f : 1.25f;
	float bias = constrainToRangeF(venturiEstimateData.lateralVelocity * VENTURI_EST_BIAS_GAIN * directionScalar, 0, VENTURI_EST_BIAS_VALUE_MAX);
	venturiEstimateData.venturiBias = lowPassFilterUpdate(&venturiBiasLPF, bias, dt);

	return venturiEstimateData.venturiBias;
}

float updateVenturiBiasAerodynamicOld(float dt) {
	if (fcStatusData.throttlePercent <= fcStatusData.liftOffThrottlePercent) {
		resetVenturiBiasEstimator();
		return 0;
	};
	float pitch = constrainToRangeF(fabs(applyDeadBandFloat(0, sensorAttitudeData.pitch, VENTURI_EST_PITCH_ANGLE_MIN)), 0, VENTURI_EST_PITCH_ANGLE_MAX);
	venturiEstimateData.venturiAbsPitchAngleFiltered = lowPassFilterUpdate(&venturiPitchAngleLPF, pitch, dt);

	venturiEstimateData.ventiriAbsPitchRCFiltered = lowPassFilterUpdate(&venturiPitchRCLPF, constrainToRangeF(fabs(rcData.RC_EFFECTIVE_DATA[RC_PITCH_CHANNEL_INDEX]), 0, VENTURI_EST_PITCH_RC_MAX), dt);

	float effectiveAngle = venturiEstimateData.venturiAbsPitchAngleFiltered;
	if (venturiEstimateData.ventiriAbsPitchRCFiltered > 0.1f && venturiEstimateData.venturiAbsPitchAngleFiltered <= VENTURI_SMALL_ANGLE_TSH) {
		effectiveAngle = VENTURI_SMALL_ANGLE_DEFAULT;
	}

	float estSpeed = fastSqrtf(tanApprox(convertDegToRad(effectiveAngle))) * VENTURI_K_AERO_DRAG_SCALAR;
	venturiEstimateData.lateralVelocity = constrainToRangeF(estSpeed, 0, VENTURI_EST_SPEED_MAX);

	float bias = constrainToRangeF(venturiEstimateData.lateralVelocity * VENTURI_EST_BIAS_GAIN, 0, VENTURI_EST_BIAS_VALUE_MAX);
	venturiEstimateData.venturiBias = lowPassFilterUpdate(&venturiBiasLPF, bias, dt);

	return venturiEstimateData.venturiBias;
}

float updateVenturiBiasHeuristic(float dt) {
	if (fcStatusData.throttlePercent <= fcStatusData.liftOffThrottlePercent) {
		resetVenturiBiasEstimator();
		return 0;
	};
	float pitch = constrainToRangeF(fabs(applyDeadBandFloat(0, sensorAttitudeData.pitch, VENTURI_EST_PITCH_ANGLE_MIN)), 0, VENTURI_EST_PITCH_ANGLE_MAX);
	venturiEstimateData.venturiAbsPitchAngleFiltered = lowPassFilterUpdate(&venturiPitchAngleLPF, pitch, dt);

	venturiEstimateData.ventiriAbsPitchRCFiltered = lowPassFilterUpdate(&venturiPitchRCLPF, constrainToRangeF(fabs(rcData.RC_EFFECTIVE_DATA[RC_PITCH_CHANNEL_INDEX]), 0, VENTURI_EST_PITCH_RC_MAX), dt);
	venturiEstimateData.currentThrottle = fmaxf(fcStatusData.throttlePercent - fcStatusData.liftOffThrottlePercent, 0.0f);
	float effectiveAngle = venturiEstimateData.venturiAbsPitchAngleFiltered;

	if (venturiEstimateData.ventiriAbsPitchRCFiltered > 0.1f && venturiEstimateData.venturiAbsPitchAngleFiltered <= VENTURI_SMALL_ANGLE_TSH) {
		effectiveAngle = VENTURI_SMALL_ANGLE_DEFAULT;
	}
	float estSpeed = fastSqrtf(venturiEstimateData.currentThrottle) * sinApprox(convertDegToRad(effectiveAngle)) * venturiEstimateData.venturiAccelGain;

	venturiEstimateData.lateralVelocity = constrainToRangeF(estSpeed, 0, VENTURI_EST_SPEED_MAX);
	float bias = constrainToRangeF(venturiEstimateData.lateralVelocity * VENTURI_EST_BIAS_GAIN, 0, VENTURI_EST_BIAS_VALUE_MAX);
	venturiEstimateData.venturiBias = lowPassFilterUpdate(&venturiBiasLPF, bias, dt);
	return venturiEstimateData.venturiBias;
}

float updateVenturiBiasEstimate(float dt) {
	//return updateVenturiBiasHeuristic(dt);
	return updateVenturiBiasAerodynamic(dt);
}
void resetVenturiBiasEstimator(void) {
	venturiEstimateData.venturiBias = 0.0f;
	venturiEstimateData.venturiVelocity = 0.0f;
	venturiEstimateData.venturiBiasGain = 0.0f;
	venturiEstimateData.venturiThrustFactor = 0.0f;
	venturiEstimateData.venturiAbsPitchAngleFiltered = 0.0f;
	venturiEstimateData.ventiriAbsPitchRCFiltered = 0.0f;
	lowPassFilterReset(&venturiPitchAngleLPF);
}

#endif
