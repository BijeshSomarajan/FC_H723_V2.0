#ifndef SRC_FC_MANAGERS_POSITION_ESTIMATOR_VENTURIBIASESTIMATOR_C_
#define SRC_FC_MANAGERS_POSITION_ESTIMATOR_VENTURIBIASESTIMATOR_C_

#include "VenturiBiasEstimator.h"

#include <stdio.h>

#include "../../../calibration/Calibration.h"
#include "../../../dsp/LowPassFilter.h"
#include "../../../logger/Logger.h"
#include "../../../memory/Memory.h"
#include "../../../status/FCStatus.h"
#include "../../../util/MathUtil.h"
#include "../helpers/PositionManagerHelper.h"
#include "PositionEstimatorHelper.h"

VENTURI_ESTIMATE_DATA venturiEstimateData;
float venturiBiasGain = VENTURI_EST_BIAS_GAIN;

uint8_t initVenturiBiasEstimator(void) {
	resetVenturiBiasEstimator();
	return 1;
}

__ATTR_ITCM_TEXT
float getVenturiBiasEstimate(float dt, float speed) {
	if (!fcStatusData.canFly || fcStatusData.throttlePercent <= fcStatusData.liftOffThrottlePercent || !isNavModeActive()) {
		resetVenturiBiasEstimator();
		return 0.0f;
	}
	float v = constrainToRangeF(speed, 0.0f, VENTURI_EST_SPEED_MAX);
	venturiEstimateData.lateralSpeedMag = v;

	float target = constrainToRangeF(v * v * venturiBiasGain, 0.0f, VENTURI_EST_BIAS_VALUE_MAX);
	float tau = (target > venturiEstimateData.venturiBias) ? VENTURI_EST_BIAS_TAU_RISE : VENTURI_EST_BIAS_TAU_FALL;
	venturiEstimateData.venturiBias += (dt / (tau + dt)) * (target - venturiEstimateData.venturiBias);
	return venturiEstimateData.venturiBias;
}

void resetVenturiBiasEstimator(void) {
	venturiEstimateData.venturiBias = 0.0f;
	venturiEstimateData.lateralSpeedMag = 0.0f;
}

#endif
