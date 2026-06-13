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
#include "../../managers/position/estimator/PositionEstimatorHelper.h"

// Inner state variables
float altMgrAltHoldActivationDt = 0;
float altStabilizationDt = 0;
float altMgrMaxHeight = 0;
uint8_t altMgrWasInStabMode = 0;

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

float altMgrSLAltUpdateDt = 0;
float altMgrTerrainAltUpdateDt = 0;

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

		altMgrMaxHeight = (float) get100XScaledCalibrationValue(CALIB_PROP_ALT_HOLD_MAX_HEIGHT_ADDR);

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
void readAltitudeSensorTimerCallback() {
	readAltitudeSensors(ALTITUDE_SENSOR_READ_PERIOD);
}

void startAltitudeSensorsRead() {
	initGPTimer3(ALTITUDE_SENSOR_READ_FREQUENCY, readAltitudeSensorTimerCallback, 4);
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

	// Clean, unobstructed tracking of stick inputs
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
	if ((fcStatusData.isLandingModeActive) && rcData.throttleCentered) {
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
	fcStatusData.altitudeSLMax = fcStatusData.altitudeSLHome + altMgrMaxHeight;
}

__ATTR_ITCM_TEXT
float getClampedCurrentAltitude() {
	float altitudeDelta = positionCordinateData.zPosition - fcStatusData.altitudeSLRef;
	altitudeDelta = constrainToRangeF(altitudeDelta, -ALT_MGR_MAX_ALT_DELTA, ALT_MGR_MAX_ALT_DELTA);
	return fcStatusData.altitudeSLRef + altitudeDelta;
}

// Add this to your global/struct state definitions alongside your other tracker:
float altMgrTiltCompIntermediate = 0.0f;
__ATTR_ITCM_TEXT
void calculateTiltCompThrottle(float dt) {
	float target = 0.0f;

	// 1. Get attitude and convert to radians
	float pitchRad = convertDegToRadF(sensorAttitudeData.pitch);
	float rollRad = convertDegToRadF(sensorAttitudeData.roll);

	// 2. Compute the composite vertical lift scaling vector
	float cosP = cosApproxF(pitchRad);
	float cosR = cosApproxF(rollRad);
	float liftComponent = cosP * cosR;

	// 3. Pre-calculate physical macro boundaries into cosine float space
	float deadbandComponent = cosApproxF(convertDegToRadF(ALT_MGR_TILT_COMP_MIN_ANGLE));
	float maxAngleComponent = cosApproxF(convertDegToRadF(ALT_MGR_TILT_COMP_MAX_ANGLE));

	// 4. Check if the vehicle has tilted past the minimum deadband threshold
	if (liftComponent < deadbandComponent) {
		float clampedLift = fmaxf(liftComponent, maxAngleComponent);
		float tiltCompFactor = (1.0f / clampedLift) - 1.0f;
		float hoverThrottle = fcStatusData.liftOffThrottlePercent * MAX_PERMISSIBLE_THROTTLE_DELTA;

		target = hoverThrottle * tiltCompFactor * ALT_MGR_TILT_COMP_GAIN;
		target = fminf(target, ALT_MGR_TILT_COMP_MAX_LIMIT);
	}

	// 5. OPTIMIZED: Asymmetric Cascaded Second-Order S-Curve Filter Step
	// Determines tau based on whether the overall profile is expanding or contracting
	float activeTau = (target >= altMgrCurrentTiltCompThDelta) ? ALT_MGR_TILT_COMP_TAU_RISE : ALT_MGR_TILT_COMP_TAU_FADE;

	// Adjust alpha for a cascaded system. To maintain a similar overall transient window
	// as your original first-order filter, reduce your base TAU values by roughly 30-40%.
	float alpha = dt / (activeTau + dt);
	target = constrainToRangeF(target, 0.0f, ALT_MGR_TILT_COMP_MAX_LIMIT);

	// Stage 1: Primary smoothing (Generates the baseline transition profile)
	altMgrTiltCompIntermediate += alpha * (target - altMgrTiltCompIntermediate);

	// Stage 2: Secondary smoothing (Rounds off the acceleration corners -> Completes the S-Curve)
	altMgrCurrentTiltCompThDelta += alpha * (altMgrTiltCompIntermediate - altMgrCurrentTiltCompThDelta);

	// 6. Pipe the filtered delta directly into the actuator mixer matrix
	controlData.tiltCompThDelta = altMgrCurrentTiltCompThDelta;
}

__ATTR_ITCM_TEXT
void manageAltitude(float dt) {
	handleLanding(dt);
	if (!rcData.throttleCentered || altMgrLandingPulseActive) {
		// EDGE TRIGGER:
		if (altMgrWasThrottleCentered != 0) {
			// 1. Snapshot the actual physical throttle output to baseline memory
			fcStatusData.currentThrottle = altMgrThrottleControlLPF.output;
			altMgrPreviousThrottle = fcStatusData.currentThrottle;
			// 2. Clear the mixing output instantly to handle multi-rate execution lag
			controlData.altitudeControl = 0.0f;
		}
		handleThrottleChange(dt);
		fcStatusData.altitudeSLRef = positionCordinateData.zPosition;
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

	// Wipes internal integrators and scales gains downwards for stick tracking
	manageAltControlSettings(dt);

	if (fcStatusData.isFlying) {
		altMgrAccDtAccumulation += dt;
		altMgrVelDtAccumulation += dt;
		altMgrAltDtAccumulation += dt;
		while (altMgrAltDtAccumulation >= ALTITUDE_MANAGEMENT_ALT_TASK_PERIOD || altMgrVelDtAccumulation >= ALTITUDE_MANAGEMENT_VEL_TASK_PERIOD || altMgrAccDtAccumulation >= ALTITUDE_MANAGEMENT_ACC_TASK_PERIOD) {
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

	// High-rate mixer equation now perfectly protected against stale values
	controlData.throttleControl = fcStatusData.currentThrottle + controlData.altitudeControl + controlData.tiltCompThDelta + controlData.posBrakeCompThDelta;
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
	if (fcStatusData.isConfigMode) {
		return;
	}
	if (fcStatusData.hasCrashed) {
		resetAltitudeManager();
		return; // Stop execution immediately if crashed
	}
	if (fcStatusData.canStabilize) {
		altMgrWasInStabMode = 1;
	} else if (altMgrWasInStabMode && fcStatusData.isStabilized) {
		altMgrWasInStabMode = 0;
	}

	uint8_t dataAvailableMask = loadAltitudeSensorsData();
	float dt = getDeltaTime(SENSOR_ALT_READ_TIMER_CHANNEL);

	altMgrSLAltUpdateDt += dt;
	altMgrSLAltUpdateDt = constrainToRangeF(altMgrSLAltUpdateDt, 0.0001, ALTITUDE_SENSOR_READ_PERIOD * 10.0f);

#if SENSOR_ALT_LIDAR_AVAILABLE == 1
	altMgrTerrainAltUpdateDt += dt;
	altMgrTerrainAltUpdateDt = constrainToRangeF(altMgrTerrainAltUpdateDt, 0.0001, ALTITUDE_SENSOR_READ_PERIOD * 10.0f);
#endif

	if (dataAvailableMask != SENSOR_DATA_NONE) {
		if (dataAvailableMask & SENSOR_DATA_BARO) {
			updateZPositionSL(sensorAltitudeData.altitudeSLFiltered, altMgrSLAltUpdateDt);
			altMgrSLAltUpdateDt = 0;
		}

#if SENSOR_ALT_LIDAR_AVAILABLE == 1
		if (dataAvailableMask & SENSOR_DATA_LIDAR) {
			updateZPositionTerrain(sensorAltitudeData.altitudeTerrain, sensorAltitudeData.altitudeTerrainQlty, altMgrTerrainAltUpdateDt);
			altMgrTerrainAltUpdateDt = 0;
		}
#endif
	}
}

void resetAltitudeManager(void) {
	resetAltitudeControl(1);
	resetAltitudeSensors(0);
	resetAltMgrStates();
}
