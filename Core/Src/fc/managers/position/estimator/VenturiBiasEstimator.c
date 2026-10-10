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
LOWPASSFILTER venturiBiasLPF;
float venturiBiasGain = VENTURI_EST_BIAS_GAIN_DEFAULT;

uint8_t initVenturiBiasEstimator(void) {
	lowPassFilterInit(&venturiBiasLPF, VENTURI_EST_BIAS_LPF_FREQ);
	venturiBiasGain = get1KXScaledCalibrationValue(CALIB_PROP_VENTURI_ALT_GAIN_ADDR);
	if (venturiBiasGain <= 0.0f) {
		venturiBiasGain = VENTURI_EST_BIAS_GAIN_DEFAULT;
	}
	char debugBuf[64];
	sprintf(debugBuf, "[Venturi Estimator] Gain=%.3f, Initialized\n", venturiBiasGain);
	logString(debugBuf);
	resetVenturiBiasEstimator();
	return 1;
}

__ATTR_ITCM_TEXT
float getVenturiBiasEstimate(float dt, float speed) {
	if (speed < VENTURI_EST_SPEED_MIN || !fcStatusData.canFly || fcStatusData.throttlePercent <= fcStatusData.liftOffThrottlePercent || !isNavModeActive()) {
		resetVenturiBiasEstimator();
		return 0.0f;
	}
	venturiEstimateData.lateralSpeedMag = constrainToRangeF(speed, VENTURI_EST_SPEED_MIN, VENTURI_EST_SPEED_MAX);
	float bias = venturiEstimateData.lateralSpeedMag * venturiEstimateData.lateralSpeedMag * venturiBiasGain;
	bias = constrainToRangeF(bias, 0.0f, VENTURI_EST_BIAS_VALUE_MAX);
	venturiEstimateData.venturiBias = lowPassFilterUpdate(&venturiBiasLPF, bias, dt);
	return venturiEstimateData.venturiBias;
}

void resetVenturiBiasEstimator(void) {
	venturiEstimateData.venturiBias = 0.0f;
	venturiEstimateData.lateralSpeedMag = 0.0f;
	lowPassFilterReset(&venturiBiasLPF);
}

#endif
