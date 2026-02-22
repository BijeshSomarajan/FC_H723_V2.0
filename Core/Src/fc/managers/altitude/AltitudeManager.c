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

float altMgrLiftOffThrottle = 0;
float altMgrLiftOffThrottlePercent = 0;
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

		altMgrLiftOffThrottle = (float) getCalibrationValue(CALIB_PROP_RC_LIFTOFF_THROTTLE_ADDR);
		altMgrLiftOffThrottlePercent = altMgrLiftOffThrottle / (float) ALT_MGR_MAX_PERMISSIBLE_THROTTLE_DELTA;
		fcStatusData.liftOffThrottlePercent = altMgrLiftOffThrottlePercent;

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
		resetAltitudeRateIControl();
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
	fcStatusData.throttlePercentage = fcStatusData.currentThrottle / ALT_MGR_MAX_PERMISSIBLE_THROTTLE_DELTA;
	if (fcStatusData.throttlePercentage >= altMgrLiftOffThrottlePercent) {
		fcStatusData.isFlying = 1;
	} else {
		fcStatusData.isFlying = 0;
	}
}

__ATTR_ITCM_TEXT
void updateTiltCompThDelta(float dt) {
	static float currentTiltCompThDelta = 0.0f;
	float target = 0.0f;
	// 1. Extract Attitude and determine dominant tilt
	float pitch = sensorAttitudeData.pitch;
	float roll = sensorAttitudeData.roll;
	//float maxTilt = (fabsf(pitch) > fabsf(roll)) ? fabsf(pitch) : fabsf(roll);
	float maxTilt = fastSqrtf(pitch * pitch + roll * roll);
	// 2. Only calculate if tilt exceeds threshold
	if (maxTilt > ALT_MGR_TILT_TH_MIN_ANGLE) {
		float pRad = convertDegToRad(pitch);
		float rRad = convertDegToRad(roll);
		// Calculate the vertical component (CosP * CosR)
		float baseLiftComponent = cosApprox(pRad) * cosApprox(rRad);
		if (baseLiftComponent < altMgrMaxLiftComponent) {
			baseLiftComponent = altMgrMaxLiftComponent;
		}
		// Calculate loss and normalize
		float baseLiftLoss = 1.0f - baseLiftComponent;
		float maxPossibleLoss = 1.0f - altMgrMaxLiftComponent;
		float normalizedLoss = baseLiftLoss / (maxPossibleLoss > 0.0f ? maxPossibleLoss : 1.0f);
		// Shape the response
		float shapedLoss = generateSCurve(normalizedLoss, ALT_MGR_TILT_COMP_S_CURVE_SHARPNESS);
		// Start with the base gain
		target = shapedLoss * ALT_MGR_TILT_COMP_TH_ADJUST_GAIN;
		/* --- DIRECTIONAL GAIN ASPECT --- */
		// if PITCH_DIR is -1 and pitch is negative (Nose Down), apply extra gain
		if (ALT_MGR_TILT_COMP_TH_PITCH_DIR != 0 && ((ALT_MGR_TILT_COMP_TH_PITCH_DIR < 0 && pitch < 0) || (ALT_MGR_TILT_COMP_TH_PITCH_DIR > 0 && pitch > 0))) {
			target *= ALT_MGR_TILT_COMP_TH_ADJUST_ASSYMETRIC_GAIN;
		}
		if (ALT_MGR_TILT_COMP_TH_ROLL_DIR != 0 && ((ALT_MGR_TILT_COMP_TH_ROLL_DIR < 0 && roll < 0) || (ALT_MGR_TILT_COMP_TH_ROLL_DIR > 0 && roll > 0))) {
			target *= ALT_MGR_TILT_COMP_TH_ADJUST_ASSYMETRIC_GAIN;
		}
		// Hard limit for safety
		target = constrainToRangeF(target, 0, ALT_MGR_TILT_TH_ADJUST_LIMIT);
	}
	/* --- ASYMMETRIC FILTERING (ATTACK/DECAY) --- */
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
		//updateTiltCompThDelta(dt);
		controlData.tiltCompThDelta = 0;
		float clampedCurrentAltitude = getClampedCurrentAltitude();
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
	fcStatusData.throttlePercentage = 0;
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
