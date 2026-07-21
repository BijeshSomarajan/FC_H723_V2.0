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
LOWPASSFILTER venturiBiasLPF;

uint8_t initVenturiBiasEstimator(void) {
	lowPassFilterInit(&venturiBiasLPF, VENTURI_EST_BIAS_LPF_FREQ);
	resetVenturiBiasEstimator();
	return 1;
}
/**
 * @brief Updates the aerodynamic Venturi/Bernoulli pressure drop compensation bias.
 * @note  Smooths out post-stop bias decay to match the airframe's natural pneumatic
 * pressure equalization rate, eliminating the altitude drop/dip when halting.
 * @param dt Delta time since last execution loop in seconds.
 * @return float Cleaned, low-pass filtered throttle bias compensation value.
 */
__ATTR_ITCM_TEXT
float getVenturiBiasEstimate(float dt) {
	// 1. Safety Guard: Reset and bypass if the vehicle cannot fly or is below liftoff threshold
	if (!fcStatusData.canFly || fcStatusData.throttlePercent <= fcStatusData.liftOffThrottlePercent) {
		resetVenturiBiasEstimator();
		return 0.0f;
	}

	// 2. Extract and clamp signed pitch input
	float imuPitch = applyDeadBandFloat(0.0f, sensorAttitudeData.pitch, VENTURI_EST_PITCH_ANGLE_MIN);
	imuPitch = constrainToRangeF(imuPitch, -VENTURI_EST_PITCH_ANGLE_MAX, VENTURI_EST_PITCH_ANGLE_MAX);
	float pitchRadians = convertDegToRadF(imuPitch);
	// 3. Signed Acceleration Mapping
	float lateralAccel = tanApprox(pitchRadians) * GRAVITY_MSS * VENTURI_EST_ACCEL_GAIN;
	// 4. Vector Drag Calculation
	float drag = venturiEstimateData.lateralSpeed * VENTURI_EST_DRAG_GAIN;
	float acceleration = lateralAccel - drag;
	// Store previous speed state to evaluate zero-cross boundaries
	float prevSpeed = venturiEstimateData.lateralSpeed;
	// 5. Velocity State Integration
	venturiEstimateData.lateralSpeed += (acceleration * dt);

	// 6. Robust Zero-Cross Braking Protection
	// If acceleration is actively opposing the current direction of flight (braking phase)
	if ((prevSpeed > 0.0f && lateralAccel < 0.0f) || (prevSpeed < 0.0f && lateralAccel > 0.0f)) {
		// Clamp to exactly zero if the mathematical step overshoots the zero boundary
		if (((prevSpeed > 0.0f && venturiEstimateData.lateralSpeed <= 0.0f) || (prevSpeed < 0.0f && venturiEstimateData.lateralSpeed >= 0.0f)) && fabsf(prevSpeed) > VENTURI_EST_BRAKE_ARM_SPEED) {
			venturiEstimateData.lateralSpeed = 0.0f;
			venturiEstimateData.brakeDwell = VENTURI_EST_BRAKE_DWELL;   // e.g. 0.5f s
		}
	}

	// 6b. Brake dwell: a zero-cross during braking means the vehicle stopped.
	// Hold the speed model at zero for BRAKE_DWELL so continued brake pitch
	// isn't misread as flight in the opposite direction. A genuine direction
	// reversal outlasts the dwell and integrates normally afterwards.
	if (venturiEstimateData.brakeDwell > 0.0f) {
		venturiEstimateData.brakeDwell -= dt;
		venturiEstimateData.lateralSpeed = 0.0f;  // hold: don't build opposite-sign speed yet
	}


	// 7. Tuned Active Speed Damping Loop
	// When stick inputs enter the deadband, we drain the remaining speed memory
	// more gently using VENTURI_EST_DAMPING_GAIN to prevent a sharp drop-off step.
	if (imuPitch == 0.0f) {
		venturiEstimateData.lateralSpeed -= (venturiEstimateData.lateralSpeed * VENTURI_EST_DAMPING_GAIN * dt);
		if (fabsf(venturiEstimateData.lateralSpeed) < 0.001f) {
			venturiEstimateData.lateralSpeed = 0.0f;
		}
	}
	// Dynamic anti-windup clamp on speed state
	venturiEstimateData.lateralSpeed = constrainToRangeF(venturiEstimateData.lateralSpeed, -VENTURI_EST_SPEED_MAX, VENTURI_EST_SPEED_MAX);
	// 8. Quadratic Bernoulli Bias Translation
	float speed = venturiEstimateData.lateralSpeed;
	float bias = (speed * speed) * VENTURI_EST_BIAS_GAIN;
	bias = constrainToRangeF(bias, 0.0f, VENTURI_EST_BIAS_VALUE_MAX);
	// 9. Low-Pass Filter Smoothing
	// The lower LPF frequency handles the heavy lifting of matching pneumatic lag
	venturiEstimateData.venturiBias = lowPassFilterUpdate(&venturiBiasLPF, bias, dt);
	return venturiEstimateData.venturiBias;
}

void resetVenturiBiasEstimator(void) {
	venturiEstimateData.venturiBias = 0.0f;
	venturiEstimateData.lateralSpeed = 0.0f;
	venturiEstimateData.pitchAngleAbsFiltered = 0.0f;
	venturiEstimateData.brakeDwell = 0.0f;
	lowPassFilterReset(&venturiBiasLPF);
}

#endif
