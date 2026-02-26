#include "AltitudeManager.h"

#include "../../calibration/Calibration.h"
#include "../../control/altitude/AltitudeControl.h"
#include "../../control/ControlData.h"
#include "../../dsp/LowPassFilter.h"
#include "../../FCConfig.h"
#include "../../logger/Logger.h"
#include "../../memory/Memory.h"
#include "../../sensors/altitude/AltitudeSensor.h"
#include "../../sensors/altitude/devices/AltitudeDevice.h"
#include "../../sensors/attitude/AttitudeSensor.h"
#include "../../sensors/rc/RCSensor.h"
#include "../../status/FCStatus.h"
#include "../../timers/DeltaTimer.h"
#include "../../timers/GPTimer.h"
#include "../../timers/Scheduler.h"
#include "../../util/MathUtil.h"
#include "../../util/CommonUtil.h"
#include "../../imu/IMU.h"
#include "../../managers/position/PositionManager.h"

// Inner state variables
float altitudeUpdateDt = 0;

float altMgrAltHoldActivationDt = 0;
float altStabilizationDt = 0;

float altMgrMaxSLAlt = 0;
float altMgrMaxGndAlt = 0;

uint8_t altMgrWasInStabMode = 0;
float altMgrMaxLiftComponent = 0;

ALTITUDE_CONTROL_GAINS altControlGains;

LOWPASSFILTER altMgrThrottleControlLPF;
uint8_t altMgrWasThrottleCentered = 0;
float altMgrPreviousThrottleControl = 0;
float altMgrPreviousCurrentThrottle = 0;
float altMgrCurrentThrottleDelta = 0;
float altMgrCurrentThrottleRate = 0;
float altMgrCurrentThrottleRateGain = 1.0f;
float altMgrLateralMovementDt = 0;
uint8_t altMgrLateralFlightControlApplied = 0;

void startAltitudeSensorsRead(void);
void manageAltitudeTask(void);

uint8_t initAltitudeManager(void) {
	logString("[Altitude Manager] Init > Start\n");
	uint8_t status = initAltitudeSensors();
	if (status) {
		logString("[Altitude Manager] Sensor Init > Success\n");
		startAltitudeSensorsRead();
		schedulerAddTask(manageAltitudeTask, ALTITUDE_MANAGEMENT_TASK_FREQUENCY, ALT_MANAGEMENT_TASK_PRIORITY);
		logString("[Altitude Manager] All tasks   > Started\n");

		fcStatusData.liftOffThrottlePercent = (float) getCalibrationValue(CALIB_PROP_RC_LIFTOFF_THROTTLE_ADDR) / (float) ALT_MGR_MAX_PERMISSIBLE_THROTTLE_DELTA;

		altMgrMaxSLAlt = (float) getCalibrationValue(CALIB_PROP_ALT_HOLD_MAX_ASL_HEIGHT_ADDR);
		altMgrMaxGndAlt = (float) getCalibrationValue(CALIB_PROP_ALT_HOLD_MAX_TERRAIN_HEIGHT_ADDR);
		altMgrMaxLiftComponent = cosf(convertDegToRad(ALT_MGR_TILT_TH_MAX_ANGLE));

		lowPassFilterInit(&altMgrThrottleControlLPF, ALT_MGR_THROTTLE_AVERAGING_LPF_FREQUENCY);
		altControlGains.masterPGain = 1.0f;
		altControlGains.ratePGain = 1.0f;
		altControlGains.rateIGain = 1.0f;
		altControlGains.rateDGain = 1.0f;
		altControlGains.accPGain = 1.0f;
		altControlGains.accDGain = 1.0f;

		initAltitudeControl();
	} else {
		logString("[Altitude Manager] Init > Failed!\n");
	}
	return status;
}

__ATTR_ITCM_TEXT
void readBaroSensorTimerCallback() {
	readAltitudeSensors();
}

void startAltitudeSensorsRead() {
	initGPTimer3(BARO_SENSOR_READ_FREQUENCY, readBaroSensorTimerCallback, 4);
	startGPTimer3();
}

__ATTR_ITCM_TEXT
void manageAltControlSettings(float dt) {
	if (!rcData.throttleCentered) {
		float deflectionGain = 1.0f;
		if (altMgrPreviousCurrentThrottle != 0) {
			altMgrCurrentThrottleDelta = fabsf(fcStatusData.currentThrottle - altMgrPreviousCurrentThrottle);
			if (altMgrCurrentThrottleDelta > 0) {
				altMgrCurrentThrottleRate = (altMgrCurrentThrottleDelta / dt) * ALT_MGR_ALT_CONTROL_STICK_RATE_SCALER;
				altMgrCurrentThrottleRate = constrainToRangeF(altMgrCurrentThrottleRate, 0.0f, 1.0f);
				altMgrCurrentThrottleRateGain = 1.0f - (altMgrCurrentThrottleRate * ALT_MGR_THROTTLE_RATE_ATTENUATION_GAIN);
			}
		}
		float currentStickDeflection = fabsf(rcData.RC_EFFECTIVE_DATA[RC_TH_CHANNEL_INDEX]);
		float deflectionRatio = constrainToRangeF(currentStickDeflection / (float) ALT_MGR_MAX_PERMISSIBLE_THROTTLE_DELTA, 0.0f, 1.0f);
		deflectionGain = 1.0f - (deflectionRatio * ALT_MGR_ALT_CONTROL_STICK_ATTENUATION_GAIN);
		float totalAttenuation = altMgrCurrentThrottleRateGain * deflectionGain;
		altControlGains.masterPGain = ALT_MGR_ALT_CONTROL_SETTING_MASTER_P_GAIN * totalAttenuation;
		altControlGains.ratePGain = ALT_MGR_ALT_CONTROL_SETTING_RATE_P_GAIN * totalAttenuation;
		altControlGains.rateIGain = ALT_MGR_ALT_CONTROL_SETTING_RATE_I_GAIN * totalAttenuation;
		altControlGains.accPGain = ALT_MGR_ALT_CONTROL_SETTING_ACC_P_GAIN * totalAttenuation;
		resetAltitudeRIControl();
	} else {
		altMgrCurrentThrottleDelta = 0;
		altMgrCurrentThrottleRate = 0;
		if (altControlGains.masterPGain < 1.0f) {
			altControlGains.masterPGain += (dt / ALT_MGR_ALT_CONTROL_SETTING_MP_TAU) * (1.0f - altControlGains.masterPGain);
		}
		if (altControlGains.ratePGain < 1.0f) {
			altControlGains.ratePGain += (dt / ALT_MGR_ALT_CONTROL_SETTING_RP_TAU) * (1.0f - altControlGains.ratePGain);
		}
		if (altControlGains.rateIGain < 1.0f) {
			altControlGains.rateIGain += (dt / ALT_MGR_ALT_CONTROL_SETTING_RI_TAU) * (1.0f - altControlGains.rateIGain);
		}
		if (altControlGains.accPGain < 1.0f) {
			altControlGains.accPGain += (dt / ALT_MGR_ALT_CONTROL_SETTING_AP_TAU) * (1.0f - altControlGains.accPGain);
		}
	}
}

__ATTR_ITCM_TEXT
void handleThrottleChange(float dt) {
	float gain = rcData.RC_EFFECTIVE_DATA[RC_TH_CHANNEL_INDEX] * ALT_MGR_ALT_AGGREGATION_GAIN * dt;
	if (altMgrWasThrottleCentered != 0) {
		fcStatusData.currentThrottle = altMgrThrottleControlLPF.output;
	}
	fcStatusData.currentThrottle += gain;
	fcStatusData.currentThrottle = constrainToRangeF(fcStatusData.currentThrottle, 0, ALT_MGR_MAX_PERMISSIBLE_THROTTLE_DELTA);
	fcStatusData.throttlePercent = fcStatusData.currentThrottle / ALT_MGR_MAX_PERMISSIBLE_THROTTLE_DELTA;
	if (fcStatusData.throttlePercent >= fcStatusData.liftOffThrottlePercent) {
		fcStatusData.isFlying = 1;
	} else {
		fcStatusData.isFlying = 0;
	}
}

__ATTR_ITCM_TEXT
void calculateTiltCompThrottle(float dt) {
	static float currentTiltCompThDelta = 0.0f;
	float target = 0.0f;
	float pitch = sensorAttitudeData.pitch;
	float roll = sensorAttitudeData.roll;
	float totalTilt = fastSqrtf(pitch * pitch + roll * roll);
	if (totalTilt > ALT_MGR_TILT_TH_MIN_ANGLE) {
		// Geometric Lift Loss: 1.0 - (CosP * CosR)
		float liftComponent = cosApprox(convertDegToRad(pitch)) * cosApprox(convertDegToRad(roll));
		// Clamp to the 45-degree limit defined in header
		float minComponent = cosApprox(convertDegToRad(ALT_MGR_TILT_TH_MAX_ANGLE));
		liftComponent = fmaxf(liftComponent, minComponent);
		// Apply linear gain to the loss
		target = (1.0f - liftComponent) * ALT_MGR_TILT_COMP_TH_ADJUST_GAIN;
		target = fminf(target, ALT_MGR_TILT_TH_ADJUST_MAX_LIMIT);
	}
	// Smooth filtering
	float activeTau = (target > currentTiltCompThDelta) ? ALT_MGR_TILT_COMP_TH_ADJUST_TAU : (ALT_MGR_TILT_COMP_TH_ADJUST_TAU * ALT_MGR_TILT_COMP_EXIT_TAU_MULTIPLIER);
	float alpha = dt / (activeTau + dt);
	currentTiltCompThDelta += alpha * (target - currentTiltCompThDelta);
	controlData.tiltCompThDelta = currentTiltCompThDelta;
}

__ATTR_ITCM_TEXT
void updateAltitudeRefernces() {
	fcStatusData.altitudeSLHome = sensorAltitudeData.altitudeSLMaxFiltered;
	fcStatusData.altitudeSLRef = sensorAltitudeData.altitudeSLMaxFiltered;
	fcStatusData.altitudeSLMax = fcStatusData.altitudeSLHome + altMgrMaxSLAlt;
	fcStatusData.altitudeGndMax = altMgrMaxGndAlt;
}

__ATTR_ITCM_TEXT
float getClampedCurrentAltitude() {
	float altitudeDelta = positionData.zPosition - fcStatusData.altitudeSLRef;
	altitudeDelta = constrainToRangeF(altitudeDelta, -ALT_MGR_MAX_ALT_DELTA, ALT_MGR_MAX_ALT_DELTA);
	return fcStatusData.altitudeSLRef + altitudeDelta;
}

__ATTR_ITCM_TEXT
void calculateVenturiBiasOld(float dt) {
	static float velocityProxy = 0.0f;
	static float venturiBias = 0.0f;

	// 1. Get current states
	float pitchRad = convertDegToRad(sensorAttitudeData.pitch);
	float netThrottlePct = fmaxf(fcStatusData.throttlePercent - fcStatusData.liftOffThrottlePercent, 0.0f);

	// 2. Calculate Horizontal Acceleration
	float horizontalThrust = netThrottlePct * sinApprox(pitchRad);

	// 3. Integrate Force to Velocity with Time-Consistent Drag
	velocityProxy += (horizontalThrust * dt * ALT_MGR_VENTURI_ACCEL_SCALER);
	float dragAlpha = dt / (ALT_MGR_VENTURI_DRAG_TAU + dt);
	velocityProxy -= dragAlpha * velocityProxy;

	// 4. Calculate Venturi Bias
	float gain = (velocityProxy < 0.0f) ? ALT_MGR_VENTURI_GAIN_BWD : ALT_MGR_VENTURI_GAIN_FWD;
	float targetBias = -(velocityProxy * velocityProxy) * gain;

	// 5. Smooth the output (Rise Fast, Fade Slow Logic)
	// targetBias is always negative.
	// If targetBias < venturiBias, it means the vacuum is INCREASING (Rise Fast)
	float activeTau = (targetBias < venturiBias) ? ALT_MGR_VENTURI_TAU_RISE : ALT_MGR_VENTURI_TAU_FADE;
	float alpha = dt / (activeTau + dt);
	venturiBias += alpha * (targetBias - venturiBias);

	// 6. Final safety clamp
	sensorAltitudeData.altVenturiBias = constrainToRangeF(venturiBias, -ALT_MGR_VENTURI_BIAS_MAX, 0.0f);
}




__ATTR_ITCM_TEXT
void calculateVenturiBias(float dt) {
	static float velocityProxy = 0.0f;
	static float venturiBias = 0.0f;
// 1. Get current states
	float pitchRad = convertDegToRad(sensorAttitudeData.pitch);
	float netThrottlePct = fmaxf(fcStatusData.throttlePercent - fcStatusData.liftOffThrottlePercent, 0.0f);
// 2. Acceleration Calculation
	float horizontalThrust = netThrottlePct * sinApprox(pitchRad);
// 3. Update Velocity with "Terminal Velocity" Cap
// This prevents the math from overshooting the physical limits of the drone.
	velocityProxy += (horizontalThrust * dt * ALT_MGR_VENTURI_ACCEL_SCALER);
// HARD LIMIT: Constrain proxy to realistic max speed (e.g., +/- 15.0m/s)
	velocityProxy = constrainToRangeF(velocityProxy, -ALT_MGR_MAX_SPEED, ALT_MGR_MAX_SPEED);
// 4. Time-Consistent Drag
// Increasing DRAG_TAU here will stop the "gradual decrease" while moving.
	float dragAlpha = dt / (ALT_MGR_VENTURI_DRAG_TAU + dt);
	velocityProxy -= dragAlpha * velocityProxy;
// 5. Calculate Bias with Asymmetric Directional Gains
	float gain = (velocityProxy < 0.0f) ? ALT_MGR_VENTURI_GAIN_BWD : ALT_MGR_VENTURI_GAIN_FWD;
	float targetBias = -(velocityProxy * velocityProxy) * gain;
// 6. Rise Fast / Fade Slow Filter
	float activeTau = (targetBias < venturiBias) ? ALT_MGR_VENTURI_TAU_RISE : ALT_MGR_VENTURI_TAU_FADE;
	float alpha = dt / (activeTau + dt);
	venturiBias += alpha * (targetBias - venturiBias);
	sensorAltitudeData.altVenturiBias = constrainToRangeF(venturiBias, -ALT_MGR_VENTURI_BIAS_MAX, 0.0f);
}

__ATTR_ITCM_TEXT
void manageAltitude(float dt) {
	if (!rcData.throttleCentered) {
		handleThrottleChange(dt);
		altMgrWasThrottleCentered = 0;
	} else {
		if (altMgrWasThrottleCentered == 0) {
			fcStatusData.altitudeSLRef = positionData.zPosition;
			altMgrWasThrottleCentered = 1;
		} else {
			altMgrWasThrottleCentered = 2;
		}
		altMgrPreviousThrottleControl = altMgrThrottleControlLPF.output;
	}
	manageAltControlSettings(dt);
	if (fcStatusData.isFlying) {
		//calculateTiltCompThrottle(dt);
		float clampedCurrentAltitude = getClampedCurrentAltitude();
		calculateVenturiBias(dt);
		clampedCurrentAltitude += sensorAltitudeData.altVenturiBias;
		controlAltitudeWithGains(dt, fcStatusData.altitudeSLRef, clampedCurrentAltitude, altControlGains);
	} else {
		updateAltitudeRefernces();
		resetAltitudeControl(1);
		controlData.tiltCompThDelta = 0;
	}
	controlData.throttleControl = fcStatusData.currentThrottle + controlData.altitudeControl + controlData.tiltCompThDelta;
	controlData.throttleControl = constrainToRangeF(controlData.throttleControl, 0, ALT_MGR_MAX_PERMISSIBLE_THROTTLE_DELTA);
	altMgrPreviousCurrentThrottle = fcStatusData.currentThrottle;
	lowPassFilterUpdate(&altMgrThrottleControlLPF, controlData.throttleControl, dt);
}

void resetAltMgrStates() {
	fcStatusData.throttlePercent = 0;
	fcStatusData.currentThrottle = 0;
	controlData.throttleControl = 0;
	controlData.tiltCompThDelta = 0;
	fcStatusData.isFlying = 0;

	altControlGains.masterPGain = 1.0f;
	altControlGains.ratePGain = 1.0f;
	altControlGains.rateIGain = 1.0f;
	altControlGains.rateDGain = 1.0f;
	altControlGains.accPGain = 1.0f;
	altControlGains.accDGain = 1.0f;

	altMgrWasThrottleCentered = 0;
	altMgrPreviousThrottleControl = 0;
	altMgrPreviousCurrentThrottle = 0;
	altMgrCurrentThrottleRate = 0;
	altMgrCurrentThrottleDelta = 0;
	altMgrCurrentThrottleRateGain = 1.0f;

	altMgrLateralMovementDt = 0;
	altMgrLateralFlightControlApplied = 0;
	sensorAltitudeData.altVenturiBias = 0;

	lowPassFilterReset(&altMgrThrottleControlLPF);
}

__ATTR_ITCM_TEXT
void manageAltitudeTask(void) {
	float dt = getDeltaTime(ALT_MANAGER_TIMER_CHANNEL);
	sensorAltitudeData.altProcessDt = dt;
	if (fcStatusData.canFly) {
		manageAltitude(dt);
	} else {
		resetAltitudeControl(1);
		updateAltitudeRefernces();
		resetAltMgrStates();
	}
}

__ATTR_ITCM_TEXT
void doAltitudeManagement(void) {
	if (!fcStatusData.isConfigMode) {
		if (fcStatusData.canStabilize) {
			altMgrWasInStabMode = 1;
		} else if (altMgrWasInStabMode && fcStatusData.isStabilized) {
			altMgrWasInStabMode = 0;
		}
		if (loadAltitudeSensorsData()) {
			float dt = getDeltaTime(SENSOR_BARO_READ_TIMER_CHANNEL);
			sensorAltitudeData.altUpdateDt = dt;
			updateAltitudeSensorData(dt);
			updatePositionManagerZPosition(sensorAltitudeData.altitudeSLScaled, dt);
		}
	}
}

void resetAltitudeManager(void) {
	resetAltitudeControl(1);
	resetAltitudeSensors(0);
	resetAltMgrStates();
}
