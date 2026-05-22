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
float altMgrPreviousThrottle = 0.0f;

float altMgrAccDtAccumulation = 0.0f;
float altMgrVelDtAccumulation = 0.0f;
float altMgrAltDtAccumulation = 0.0f;
float altMgrLowThDtAccumulation = 0.0f;

float altMgrCurrentTiltCompThDelta = 0.0f;

uint8_t altMgrLandingPulseActive = 0;
float altMgrLandingPulseDt = 0;
float altMgrLandingCommand = 0;

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

		fcStatusData.liftOffThrottlePercent = (float) getCalibrationValue(CALIB_PROP_RC_LIFTOFF_THROTTLE_ADDR) / (float) MAX_PERMISSIBLE_THROTTLE_DELTA;

		altMgrMaxSLAlt = (float) getCalibrationValue(CALIB_PROP_ALT_HOLD_MAX_ASL_HEIGHT_ADDR);
		altMgrMaxGndAlt = (float) getCalibrationValue(CALIB_PROP_ALT_HOLD_MAX_TERRAIN_HEIGHT_ADDR);
		altMgrMaxLiftComponent = cosf(convertDegToRadF(ALT_MGR_TILT_TH_MAX_ANGLE));

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
	if (!rcData.throttleCentered || altMgrLandingPulseActive) {
		float deflectionGain = 1.0f;
		if (altMgrPreviousCurrentThrottle != 0) {
			altMgrCurrentThrottleDelta = fabsf(fcStatusData.currentThrottle - altMgrPreviousCurrentThrottle);
			if (altMgrCurrentThrottleDelta > 0) {
				altMgrCurrentThrottleRate = (altMgrCurrentThrottleDelta / dt) * ALT_MGR_ALT_CONTROL_STICK_RATE_SCALER;
				altMgrCurrentThrottleRate = constrainToRangeF(altMgrCurrentThrottleRate, 0.0f, 1.0f);
				altMgrCurrentThrottleRateGain = 1.0f - (altMgrCurrentThrottleRate * ALT_MGR_THROTTLE_RATE_ATTENUATION_GAIN);
			}
		}
		float currentStickDeflection = fabsf(altMgrLandingPulseActive ? -altMgrLandingCommand : rcData.RC_EFFECTIVE_DATA[RC_TH_CHANNEL_INDEX]);
		float deflectionRatio = constrainToRangeF(currentStickDeflection / (float) MAX_PERMISSIBLE_THROTTLE_DELTA, 0.0f, 1.0f);
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
	float currentStick = altMgrLandingPulseActive ? -altMgrLandingCommand : rcData.RC_EFFECTIVE_DATA[RC_TH_CHANNEL_INDEX];
	float gain = currentStick * ALT_MGR_ALT_AGGREGATION_GAIN * dt;
	if (altMgrWasThrottleCentered != 0) {
		float lpfValue = altMgrThrottleControlLPF.output;
		if (currentStick < 0.0f && lpfValue > altMgrPreviousThrottle) {
			fcStatusData.currentThrottle = altMgrPreviousThrottle;
		} else if (currentStick > 0.0f && lpfValue < altMgrPreviousThrottle) {
			fcStatusData.currentThrottle = altMgrPreviousThrottle;
		} else {
			fcStatusData.currentThrottle = lpfValue;
		}
	}
	float nextThrottle = fcStatusData.currentThrottle + gain;
	if (currentStick < 0.0f) { // Moving Down
		if (nextThrottle > altMgrPreviousThrottle) {
			nextThrottle = altMgrPreviousThrottle;
		}
	} else if (currentStick > 0.0f) { // Moving Up
		if (nextThrottle < altMgrPreviousThrottle) {
			nextThrottle = altMgrPreviousThrottle;
		}
	}

	fcStatusData.currentThrottle = nextThrottle;
	fcStatusData.currentThrottle = constrainToRangeF(fcStatusData.currentThrottle, 0, MAX_PERMISSIBLE_THROTTLE_DELTA);
	fcStatusData.throttlePercent = fcStatusData.currentThrottle / MAX_PERMISSIBLE_THROTTLE_DELTA;

	if (!fcStatusData.isFlying && fcStatusData.throttlePercent >= fcStatusData.liftOffThrottlePercent) {
		fcStatusData.isFlying = 1;
		altMgrLowThDtAccumulation = 0;
	} else if (fcStatusData.isFlying && fminf(fcStatusData.throttleControlPercent, fcStatusData.throttlePercent) < (fcStatusData.liftOffThrottlePercent * 0.25f)) {
		if (altMgrLowThDtAccumulation >= ALT_MGR_THROTTLE_THRESHOLD_PERIOD) {
			fcStatusData.isFlying = 0;
		} else {
			altMgrLowThDtAccumulation += dt;
		}
	} else {
		altMgrLowThDtAccumulation = 0;
	}

	altMgrPreviousThrottle = fcStatusData.currentThrottle;
}

__ATTR_ITCM_TEXT
void handleLanding(float dt) {
	if ((fcStatusData.isLandingModeActive || fcStatusData.isLandingModeActiveAfterRTH) && rcData.throttleCentered) {
		altMgrLandingPulseDt += dt;
		if (altMgrLandingPulseActive) {
			if (altMgrLandingPulseDt >= ALT_MGR_ALT_LANDING_PULSE_ACTIVE_PERIOD) {
				altMgrLandingCommand = 0;
				altMgrLandingPulseActive = 0;
				altMgrLandingPulseDt = 0;
			} else {
				altMgrLandingCommand = ALT_MGR_ALT_LANDING_STICK_COMMAND;
			}
		} else {
			if (altMgrLandingPulseDt >= ALT_MGR_ALT_LANDING_PULSE_INACTIVE_PERIOD) {
				altMgrLandingPulseActive = 1;
				altMgrLandingPulseDt = 0;
			} else {
				altMgrLandingCommand = 0;
			}
		}
	} else {
		altMgrLandingPulseActive = 0;
		altMgrLandingPulseDt = 0;
		altMgrLandingCommand = 0;
	}
}

__ATTR_ITCM_TEXT
void updateAltitudeReferences() {
	fcStatusData.altitudeSLRef = positionCordinateData.zPosition; //sensorAltitudeData.altitudeSLMaxFiltered;
	fcStatusData.altitudeSLHome = fcStatusData.altitudeSLRef;
	fcStatusData.altitudeSLMax = fcStatusData.altitudeSLHome + altMgrMaxSLAlt;
	fcStatusData.altitudeGndMax = altMgrMaxGndAlt;
}

__ATTR_ITCM_TEXT
float getClampedCurrentAltitude() {
	float altitudeDelta = positionCordinateData.zPosition - fcStatusData.altitudeSLRef;
	altitudeDelta = constrainToRangeF(altitudeDelta, -ALT_MGR_MAX_ALT_DELTA, ALT_MGR_MAX_ALT_DELTA);
	return fcStatusData.altitudeSLRef + altitudeDelta;
}

__ATTR_ITCM_TEXT
void calculateTiltCompThrottle(float dt) {
	float target = 0.0f;
	// --- Get attitude ---
	float pitch = sensorAttitudeData.pitch;   // degrees
	float roll = sensorAttitudeData.roll;    // degrees
	// --- Convert to radians ---
	float pitchRad = convertDegToRadF(pitch);
	float rollRad = convertDegToRadF(roll);
	// --- Compute lift component ---
	float cosP = cosApproxF(pitchRad);
	float cosR = cosApproxF(rollRad);
	float liftComponent = cosP * cosR;
	// --- Clamp tilt effect using max tilt angle ---
	float minComponent = cosApproxF(convertDegToRadF(ALT_MGR_TILT_TH_MAX_ANGLE));
	liftComponent = fmaxf(liftComponent, minComponent);
	// --- Apply only if meaningful tilt ---
	if (liftComponent < 0.999f) {
		// % thrust loss due to tilt
		float tiltCompFactor = (1.0f / liftComponent) - 1.0f;
		// Convert hover throttle (0–1) → throttle units
		float hoverThrottle = fcStatusData.liftOffThrottlePercent * MAX_PERMISSIBLE_THROTTLE_DELTA;
		// --- Final compensation ---
		target = hoverThrottle * tiltCompFactor * ALT_MGR_TILT_COMP_TH_GAIN;
		// --- Safety clamp (throttle units) ---
		target = fminf(target, ALT_MGR_TILT_TH_ADJUST_MAX_LIMIT);
	}
	// --- Smooth response (your original logic retained) ---
	float activeTau = (target >= altMgrCurrentTiltCompThDelta) ? ALT_MGR_TILT_COMP_TH_ADJUST_TAU_RISE : ALT_MGR_TILT_COMP_TH_ADJUST_TAU_FADE;
	float alpha = dt / (activeTau + dt);
	target = constrainToRangeF(target, 0, ALT_MGR_TILT_TH_ADJUST_MAX_LIMIT);
	altMgrCurrentTiltCompThDelta += alpha * (target - altMgrCurrentTiltCompThDelta);
	controlData.tiltCompThDelta = altMgrCurrentTiltCompThDelta;
}

__ATTR_ITCM_TEXT
void manageAltitude(float dt) {
	handleLanding(dt);
	if (!rcData.throttleCentered || altMgrLandingPulseActive) {
		handleThrottleChange(dt);
		altMgrWasThrottleCentered = 0;
	} else {
		if (altMgrWasThrottleCentered == 0) {
			fcStatusData.altitudeSLRef = positionCordinateData.zPosition;
			altMgrWasThrottleCentered = 1;
		} else {
			altMgrWasThrottleCentered = 2;
		}
		altMgrPreviousThrottleControl = altMgrThrottleControlLPF.output;
	}
	manageAltControlSettings(dt);
	if (fcStatusData.isFlying) {
		altMgrAccDtAccumulation += dt;
		altMgrVelDtAccumulation += dt;
		altMgrAltDtAccumulation += dt;
		while ( altMgrAltDtAccumulation >= ALTITUDE_MANAGEMENT_ALT_TASK_PERIOD || altMgrVelDtAccumulation >= ALTITUDE_MANAGEMENT_VEL_TASK_PERIOD || altMgrAccDtAccumulation >= ALTITUDE_MANAGEMENT_ACC_TASK_PERIOD ) {
			if (altMgrAccDtAccumulation >= ALTITUDE_MANAGEMENT_ACC_TASK_PERIOD) {
				controlAltitudeAccWithGains(ALTITUDE_MANAGEMENT_ACC_TASK_PERIOD, altControlGains);
				altMgrAccDtAccumulation -= ALTITUDE_MANAGEMENT_ACC_TASK_PERIOD;
			}
			if (altMgrVelDtAccumulation >= ALTITUDE_MANAGEMENT_VEL_TASK_PERIOD) {
				controlAltitudeVelWithGains(ALTITUDE_MANAGEMENT_VEL_TASK_PERIOD, altControlGains);
				altMgrVelDtAccumulation -= ALTITUDE_MANAGEMENT_VEL_TASK_PERIOD;
			}
			if (altMgrAltDtAccumulation >= ALTITUDE_MANAGEMENT_ALT_TASK_PERIOD) {
				controlAltitudeAltWithGains(ALTITUDE_MANAGEMENT_ALT_TASK_PERIOD, fcStatusData.altitudeSLRef, getClampedCurrentAltitude(), altControlGains);
				altMgrAltDtAccumulation -= ALTITUDE_MANAGEMENT_ALT_TASK_PERIOD;
			}
		}
#if ALT_MGR_TILT_COMP_ENABLED ==1
		calculateTiltCompThrottle(dt);
#else
		controlData.tiltCompThDelta = 0;
#endif
	} else {
		updateAltitudeReferences();
		resetAltitudeControl(1);
		controlData.tiltCompThDelta = 0;
		altMgrAccDtAccumulation = 0;
		altMgrVelDtAccumulation = 0;
		altMgrAltDtAccumulation = 0;
	}
	controlData.throttleControl = fcStatusData.currentThrottle + controlData.altitudeControl + controlData.tiltCompThDelta;
	controlData.throttleControl = constrainToRangeF(controlData.throttleControl, 0, MAX_PERMISSIBLE_THROTTLE_DELTA);
	fcStatusData.throttleControlPercent = controlData.throttleControl / MAX_PERMISSIBLE_THROTTLE_DELTA;
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
	fcStatusData.throttleControlPercent = 0;

	altMgrWasThrottleCentered = 0;
	altMgrPreviousThrottleControl = 0;
	altMgrPreviousCurrentThrottle = 0;
	altMgrCurrentThrottleRate = 0;
	altMgrCurrentThrottleDelta = 0;
	altMgrCurrentThrottleRateGain = 1.0f;
	altMgrPreviousThrottle = 0.0f;
	altMgrLowThDtAccumulation = 0;

	altMgrLandingPulseActive = 0;
	altMgrLandingPulseDt = 0;
	altMgrLandingCommand = 0;

	lowPassFilterReset(&altMgrThrottleControlLPF);
}

__ATTR_ITCM_TEXT
void manageAltitudeTask(void) {
	float dt = getDeltaTime(ALT_MANAGER_TIMER_CHANNEL);
	dt = constrainToRangeF(dt, ALTITUDE_MANAGEMENT_TASK_PERIOD * 0.001f, ALTITUDE_MANAGEMENT_TASK_PERIOD * 4.0f);
	sensorAltitudeData.altProcessDt = dt;
	if (fcStatusData.canFly) {
		manageAltitude(dt);
	} else if (fcStatusData.hasCrashed) {
		resetAltitudeManager();
	} else {
		resetAltitudeControl(1);
		updateAltitudeReferences();
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
			dt = constrainToRangeF(dt, BARO_SENSOR_READ_PERIOD * 0.001f, BARO_SENSOR_READ_PERIOD * 4.0f);
			sensorAltitudeData.altUpdateDt = dt;
			updateAltitudeSensorData(dt);
			updatePositionManagerZPosition(sensorAltitudeData.altitudeSLFiltered, dt);
		}
		if (fcStatusData.hasCrashed) {
			resetAltitudeManager();
		}
	}
}

void resetAltitudeManager(void) {
	resetAltitudeControl(1);
	resetAltitudeSensors(0);
	resetAltMgrStates();
}
