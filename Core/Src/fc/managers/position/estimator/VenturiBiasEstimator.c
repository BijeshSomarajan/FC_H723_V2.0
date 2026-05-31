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
LOWPASSFILTER  venturiBiasLPF;

uint8_t initVenturiBiasEstimator(void) {
	lowPassFilterInit(&venturiBiasLPF, VENTURI_EST_BIAS_LPF_FREQ);
	resetVenturiBiasEstimator();
	return 1;
}

__ATTR_ITCM_TEXT
float updateVenturiBiasEstimate(float dt) {

	if (!fcStatusData.canFly || fcStatusData.throttlePercent <= fcStatusData.liftOffThrottlePercent) {
		resetVenturiBiasEstimator();
		return 0.0f;
	}

	float imuPitch = constrainToRangeF(applyDeadBandFloat(0.0f, sensorAttitudeData.pitch, VENTURI_EST_PITCH_ANGLE_MIN), -VENTURI_EST_PITCH_ANGLE_MAX, VENTURI_EST_PITCH_ANGLE_MAX);
	float pitchRadians = convertDegToRadF(imuPitch);

	// a = g * tan(theta). thrustGain acts as a scaler/correction factor for the airframe.
	float lateralAccel = fabsf(tanApprox(pitchRadians)) * GRAVITY_MSS * VENTURI_EST_ACCEL_GAIN;

	// 4. Velocity Integration with Drag
	float drag = venturiEstimateData.lateralSpeed * VENTURI_EST_DRAG_GAIN;
	float acceleration = lateralAccel - drag;
	venturiEstimateData.lateralSpeed += (acceleration * dt);
	venturiEstimateData.lateralSpeed = constrainToRangeF(venturiEstimateData.lateralSpeed, -VENTURI_EST_SPEED_MAX, VENTURI_EST_SPEED_MAX);

	//Bernoulli Bias Calculation (Quadratic: Bias = k * v^2)
	float speed = venturiEstimateData.lateralSpeed;
	float bias = (speed * speed) * VENTURI_EST_BIAS_GAIN;
	bias = constrainToRangeF(bias, 0.0f, VENTURI_EST_BIAS_VALUE_MAX);

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
