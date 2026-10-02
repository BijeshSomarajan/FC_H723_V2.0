#include "AltitudeManager.h"

#include <math.h>

#include "../../calibration/Calibration.h"
#include "../../control/altitude/AltitudeControl.h"
#include "../../control/ControlData.h"
#include "../../dsp/LowPassFilter.h"
#include "../../FCConfig.h"
#include "../../logger/Logger.h"
#include "../../memory/Memory.h"
#include "../../sensors/altitude/AltitudeSensor.h"
#include "../../sensors/attitude/AttitudeSensor.h"
#include "../../sensors/rc/RCSensor.h"
#include "../../status/FCStatus.h"
#include "../../timers/DeltaTimer.h"
#include "../../timers/GPTimer.h"
#include "../../timers/Scheduler.h"
#include "../../util/MathUtil.h"
#include "../position/common/PositionCommon.h"
#include "../position/estimator/PositionEstimatorHelper.h"
#include "../position/helpers/PositionManagerHelper.h"

// Inner state variables
float altMgrAltHoldActivationDt = 0;
float altStabilizationDt = 0;
uint8_t altMgrWasInStabMode = 0;

ALTITUDE_CONTROL_GAINS altControlGains;

LOWPASSFILTER altMgrThrottleControlLPF;
LOWPASSFILTER altMgrAltHoldBrakingLPF;

float altMgrPreviousThrottleControl = 0;
float altMgrPreviousCurrentThrottle = 0;
float altMgrCurrentThrottleDelta = 0;
float altMgrCurrentThrottleRate = 0;
float altMgrCurrentThrottleRateGain = 1.0f;
float altMgrPreviousThrottle = 0.0f;

float altMgrVelDtAccumulation = 0.0f;
float altMgrAltDtAccumulation = 0.0f;
float altMgrBrakingDt = 0.0f;
float altMgrLowThDtAccumulation = 0.0f;

float altMgrCurrentTiltCompThDelta = 0.0f;

uint8_t altMgrLandingPulseActive = 0;
float altMgrLandingPulseDt = 0;
float altMgrLandingCommand = 0;

float altMgrSLAltUpdateDt = 0;
float altMgrTerrainAltUpdateDt = 0;
uint8_t altMgrWasTerrainModeActive = 0;
float altMgrAltSpeedGain = ALT_MGR_ALT_SPEED_GAIN_DEFAULT; //Meter Per Sec

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
		lowPassFilterInit(&altMgrThrottleControlLPF, ALT_MGR_THROTTLE_CONTROL_LPF_FREQUENCY);
		lowPassFilterInit(&altMgrAltHoldBrakingLPF, ALTITUDE_MGR_ALT_HOLD_BRAKE_REF_LPF_FREQ);

		altControlGains.masterPGain = 1.0f;
		altControlGains.ratePGain = 1.0f;
		altControlGains.rateIGain = 1.0f;
		altControlGains.rateDGain = 1.0f;
		altControlGains.dobGain = 1.0f;

		fcStatusData.altitudeHoldState = ALT_HOLD_STATE_IDLE;

		altMgrAltSpeedGain = get1KXScaledCalibrationValue(CALIB_PROP_ALT_HOLD_SPEED_ADDR);
		if (altMgrAltSpeedGain <= 0.0f || altMgrAltSpeedGain >= 1.0f) {
			altMgrAltSpeedGain = ALT_MGR_ALT_SPEED_GAIN_DEFAULT;
		}

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
void handleThrottleChange(float dt) {
	float currentStick = altMgrLandingPulseActive ? -altMgrLandingCommand : rcData.RC_EFFECTIVE_DATA[RC_TH_CHANNEL_INDEX];
	float gain = currentStick * altMgrAltSpeedGain * dt;
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
		//Final Reset
		fcStatusData.altitudeRef = positionCordinateData.zPosition;
		controlData.tiltCompThDelta = 0;
		altMgrCurrentTiltCompThDelta = 0.0f;
		resetAltitudeControl(1.0f);
		fcStatusData.altitudeHoldState = ALT_HOLD_STATE_IDLE;
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
	fcStatusData.altitudeRef = positionCordinateData.zPosition;
	fcStatusData.altitudeSLHome = fcStatusData.altitudeRef;
}

__ATTR_ITCM_TEXT
float getClampedCurrentAltitude() {
	float altitudeDelta = positionCordinateData.zPosition - fcStatusData.altitudeRef;
	altitudeDelta = constrainToRangeF(altitudeDelta, -ALT_MGR_MAX_ALT_DELTA, ALT_MGR_MAX_ALT_DELTA);
	return fcStatusData.altitudeRef + altitudeDelta;
}

/**
 * Learn the true hover throttle.
 * Seeded from the liftoff calibration (measured in ground effect, does not
 * track battery sag), then slowly trimmed from the actual mixed throttle
 * whenever the vehicle is genuinely hovering: flying, not climbing, not
 * heavily tilted. Bounded to a sanity band around the seed.
 */

__ATTR_ITCM_TEXT
void updateHoverThrottleEstimate(float dt) {
	float seed = fcStatusData.liftOffThrottlePercent * MAX_PERMISSIBLE_THROTTLE_DELTA;
	if (fcStatusData.hoverThrottle <= 0.0f) {
		fcStatusData.hoverThrottle = seed; /* first use: take the seed */
		return;
	}
#if ALT_CONTROL_HOVER_LEARN_ENABLED == 1
	if (!fcStatusData.isFlying) {
		return;
	}
	/* Climbing/descending throttle is not hover throttle */
	if (fabsf(positionCordinateData.zVelocity) > ALT_CONTROL_HOVER_LEARN_VEL_MAX) {
		return;
	}
	/* Tilted flight needs extra throttle that is not hover thrust */
	float liftComponent = cosApproxF(convertDegToRadF(sensorAttitudeData.pitch)) * cosApproxF(convertDegToRadF(sensorAttitudeData.roll));
	if (liftComponent < ALT_CONTROL_HOVER_LEARN_LIFT_MIN) {
		return;
	}
	float alpha = dt / (ALT_CONTROL_HOVER_LEARN_TAU + dt);
	fcStatusData.hoverThrottle += alpha * ((controlData.throttleControlBase + controlData.altitudeDOBControl) - fcStatusData.hoverThrottle);
	fcStatusData.hoverThrottle = constrainToRangeF(fcStatusData.hoverThrottle, seed * ALT_CONTROL_HOVER_LEARN_MIN_RATIO, seed * ALT_CONTROL_HOVER_LEARN_MAX_RATIO);
#endif
}

__ATTR_ITCM_TEXT
void calculateTiltCompThrottle(float dt) {
	float target = 0.0f;
	/* 1. Attitude -> lift scaling: fraction of thrust still pointing up */
	float pitchRad = convertDegToRadF(sensorAttitudeData.pitch);
	float rollRad = convertDegToRadF(sensorAttitudeData.roll);
	float liftComponent = cosApproxF(pitchRad) * cosApproxF(rollRad);
	/* 2. Physical boundaries in cosine space */
	float deadbandComponent = cosApproxF(convertDegToRadF(ALT_MGR_TILT_COMP_MIN_ANGLE));
	float maxAngleComponent = cosApproxF(convertDegToRadF(ALT_MGR_TILT_COMP_MAX_ANGLE));
	/* 3. Past the deadband -> compute the extra throttle the tilt costs.
	 *    (1/lift - 1) is a DELTA on top of the existing hover throttle,
	 *    which is why the -1 is correct: the base is already applied. */
	if (liftComponent < deadbandComponent) {
		float clampedLift = fmaxf(liftComponent, maxAngleComponent);
		float tiltCompFactor = (1.0f / clampedLift) - 1.0f;
		/* GAIN must be 1.0 for full compensation. Anything below 1.0 is
		 * deliberate under-compensation and will read as steady-state sag. */
		target = fcStatusData.hoverThrottle * tiltCompFactor * ALT_MGR_TILT_COMP_GAIN;
	}
	target = constrainToRangeF(target, 0.0f, ALT_MGR_TILT_COMP_MAX_LIMIT);
	/* 4. Single-pole asymmetric filter. One state, one direction, so the
	 *    rise/fade split is meaningful. Faster in than out: compensate tilt
	 *    promptly, relax gently. TAU values are now honest - no cascade
	 *    de-rating needed, so if you carried the "reduce by 30-40%" note,
	 *    undo it and set TAU to the real transient window you want. */
	float activeTau = (target >= altMgrCurrentTiltCompThDelta) ? ALT_MGR_TILT_COMP_TAU_RISE : ALT_MGR_TILT_COMP_TAU_FADE;
	float alpha = dt / (activeTau + dt);
	altMgrCurrentTiltCompThDelta += alpha * (target - altMgrCurrentTiltCompThDelta);

	altMgrCurrentTiltCompThDelta = constrainToRangeF(altMgrCurrentTiltCompThDelta, 0.0f, ALT_MGR_TILT_COMP_MAX_LIMIT);

	/* 5. Into the mixer */
	controlData.tiltCompThDelta = altMgrCurrentTiltCompThDelta;
}


__ATTR_ITCM_TEXT
void applyAltitudeControls(float dt) {
	altMgrVelDtAccumulation += dt;
	altMgrAltDtAccumulation += dt;
	while (altMgrAltDtAccumulation >= ALTITUDE_MANAGEMENT_ALT_TASK_PERIOD || altMgrVelDtAccumulation >= ALTITUDE_MANAGEMENT_VEL_TASK_PERIOD) {
		//Alt velocity
		if (altMgrVelDtAccumulation >= ALTITUDE_MANAGEMENT_VEL_TASK_PERIOD) {
			if (fcStatusData.altitudeHoldState != ALT_HOLD_STATE_IDLE) {
				controlAltitudeVelWithGains(ALTITUDE_MANAGEMENT_VEL_TASK_PERIOD, altControlGains);
			}
			altMgrVelDtAccumulation -= ALTITUDE_MANAGEMENT_VEL_TASK_PERIOD;
		}
		//Alt position control
		if (altMgrAltDtAccumulation >= ALTITUDE_MANAGEMENT_ALT_TASK_PERIOD) {
			switch (fcStatusData.altitudeHoldState) {
			case ALT_HOLD_STATE_IDLE:
				resetAltitudeControl(1);
				lowPassFilterResetToValue(&altMgrAltHoldBrakingLPF, positionCordinateData.zVelocity);
				altMgrBrakingDt = 0;
				fcStatusData.altitudeHoldState = ALT_HOLD_STATE_BRAKING;
				break;
			case ALT_HOLD_STATE_BRAKING:
				setExpectedAltitudeVelocity(ALTITUDE_MANAGEMENT_ALT_TASK_PERIOD, 0.0f);
				lowPassFilterUpdate(&altMgrAltHoldBrakingLPF, positionCordinateData.zVelocity, ALTITUDE_MANAGEMENT_ALT_TASK_PERIOD);
				fcStatusData.altitudeRef = positionCordinateData.zPosition;
				altMgrBrakingDt += ALTITUDE_MANAGEMENT_ALT_TASK_PERIOD;
				float verticalSpeed = fabsf(altMgrAltHoldBrakingLPF.output);
				if (verticalSpeed <= ALTITUDE_MGR_ALT_HOLD_BRAKE_MIN_SPEED) {
				    // Normal completion: vertical speed is sufficiently low.
				    fcStatusData.altitudeRef = positionCordinateData.zPosition;
				    fcStatusData.altitudeHoldState = ALT_HOLD_STATE_LOCKED;
				} else if (altMgrBrakingDt >= ALTITUDE_MGR_ALT_HOLD_BRAKE_MAX_PERIOD) {
				    // Timeout: braking did not reach the normal speed threshold.
				    // Record this condition for diagnostics.
				    // Do not interpret timeout as proof that vertical motion has stopped.
				    fcStatusData.altitudeRef = positionCordinateData.zPosition;
				    fcStatusData.altitudeHoldState = ALT_HOLD_STATE_LOCKED;
				}
				break;
			case ALT_HOLD_STATE_LOCKED:
				lowPassFilterUpdate(&altMgrAltHoldBrakingLPF, positionCordinateData.zVelocity, ALTITUDE_MANAGEMENT_ALT_TASK_PERIOD);
				controlAltitudeAltWithGains(ALTITUDE_MANAGEMENT_ALT_TASK_PERIOD, fcStatusData.altitudeRef, getClampedCurrentAltitude(), altControlGains);
				break;
			}
			altMgrAltDtAccumulation -= ALTITUDE_MANAGEMENT_ALT_TASK_PERIOD;
		}
	}
}

__ATTR_ITCM_TEXT
void manageAltitude(float dt) {
	handleLanding(dt);
	updateHoverThrottleEstimate(dt);

	if (!rcData.throttleCentered || altMgrLandingPulseActive) {
		// 1. Snapshot the actual physical throttle output to baseline memory
		fcStatusData.currentThrottle = altMgrThrottleControlLPF.output;
		altMgrPreviousThrottle = fcStatusData.currentThrottle;
		// 2. Clear the mixing output instantly to handle multi-rate execution lag
		controlData.altitudeControl = 0.0f;
		resetAltitudeDOBControl();
		handleThrottleChange(dt);
		resetAltitudeControl(1);

		fcStatusData.altitudeRef = positionCordinateData.zPosition;
		fcStatusData.altitudeHoldState = ALT_HOLD_STATE_IDLE;
		controlData.tiltCompThDelta = 0;
		altMgrCurrentTiltCompThDelta = 0.0f;

		altMgrVelDtAccumulation = 0;
		altMgrAltDtAccumulation = 0;

	} else {
		applyAltitudeControls(dt);
		calculateTiltCompThrottle(dt);
	}

	if (!fcStatusData.isFlying) {
		updateAltitudeReferences();
		resetAltitudeControl(1);
		controlData.tiltCompThDelta = 0;
		altMgrCurrentTiltCompThDelta = 0.0f;
		altMgrVelDtAccumulation = 0;
		altMgrAltDtAccumulation = 0;
	}

	// High-rate mixer equation now perfectly protected against stale values
	controlData.throttleControlBase = fcStatusData.currentThrottle + controlData.altitudeControl;
	controlData.throttleControl = controlData.throttleControlBase + controlData.altitudeDOBControl + controlData.tiltCompThDelta;
	controlData.throttleControl = constrainToRangeF(controlData.throttleControl, 0, MAX_PERMISSIBLE_THROTTLE_DELTA);
	fcStatusData.throttleControlPercent = controlData.throttleControl / MAX_PERMISSIBLE_THROTTLE_DELTA;
	altMgrPreviousCurrentThrottle = fcStatusData.currentThrottle;

	lowPassFilterUpdate(&altMgrThrottleControlLPF, controlData.throttleControl, dt);
}

void resetAltMgrStates() {
	fcStatusData.throttlePercent = 0;
	fcStatusData.currentThrottle = 0;
	controlData.throttleControl = 0;
	controlData.throttleControlBase = 0;
	controlData.tiltCompThDelta = 0;
	altMgrCurrentTiltCompThDelta = 0.0f;
	fcStatusData.isFlying = 0;

	altControlGains.masterPGain = 1.0f;
	altControlGains.ratePGain = 1.0f;
	altControlGains.rateIGain = 1.0f;
	altControlGains.rateDGain = 1.0f;
	altControlGains.dobGain = 1.0f;

	fcStatusData.throttleControlPercent = 0;
	fcStatusData.hoverThrottle = 0;

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

	sensorAltitudeData.altitudeTerrainZOffset = 0;
	sensorAltitudeData.altitudeSLZOffset = 0;
	altMgrWasTerrainModeActive = 0;

	fcStatusData.altitudeHoldState = ALT_HOLD_STATE_IDLE;
	lowPassFilterReset(&altMgrThrottleControlLPF);
	lowPassFilterReset(&altMgrAltHoldBrakingLPF);
}

__ATTR_ITCM_TEXT
void manageAltitudeTask(void) {
	float dt = getDeltaTime(ALT_MANAGER_TIMER_CHANNEL);
	dt = constrainToRangeF(dt, ALTITUDE_MANAGEMENT_TASK_PERIOD * 0.001f, ALTITUDE_MANAGEMENT_TASK_PERIOD * 4.0f);
	sensorAltitudeData.altProcessDt = dt;
	if (fcStatusData.hasCrashed) {
		resetAltitudeManager();
	} else if (fcStatusData.canFly) {
		manageAltitude(dt);
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
		return;
	}
	if (fcStatusData.canStabilize) {
		altMgrWasInStabMode = 1;
	} else if (altMgrWasInStabMode && fcStatusData.isStabilized) {
		altMgrWasInStabMode = 0;
	}

	uint8_t dataAvailableMask = loadAltitudeSensorsData();
	float dt = getDeltaTime(SENSOR_ALT_READ_TIMER_CHANNEL);

	altMgrSLAltUpdateDt += dt;
	altMgrSLAltUpdateDt = constrainToRangeF(altMgrSLAltUpdateDt, 0.0001f, ALTITUDE_SENSOR_READ_PERIOD * 10.0f);

	if (fcStatusData.isTerrainSensorExist == 1) {
		altMgrTerrainAltUpdateDt += dt;
		altMgrTerrainAltUpdateDt = constrainToRangeF(altMgrTerrainAltUpdateDt, 0.0001f, ALTITUDE_SENSOR_READ_PERIOD * 10.0f);

		if (!altMgrWasTerrainModeActive && fcStatusData.isTerrainAltModeActive && fcStatusData.isTerrainAltDataReliable) {
			sensorAltitudeData.altitudeTerrainZOffset = positionCordinateData.zPosition - sensorAltitudeData.altitudeTerrainFiltered;
		} else if (altMgrWasTerrainModeActive && !fcStatusData.isTerrainAltModeActive) {
			sensorAltitudeData.altitudeSLZOffset = positionCordinateData.zPosition - sensorAltitudeData.altitudeSLFiltered;
		}
		altMgrWasTerrainModeActive = fcStatusData.isTerrainAltModeActive;
	}

	if (dataAvailableMask != SENSOR_DATA_NONE) {
		if (dataAvailableMask & SENSOR_DATA_BARO) {
			updateZPositionSL(sensorAltitudeData.altitudeSLZOffset, sensorAltitudeData.altitudeSLScaled, altMgrSLAltUpdateDt);
			altMgrSLAltUpdateDt = 0.0f;
		}
		if (fcStatusData.isTerrainSensorExist == 1) {
			if (dataAvailableMask & SENSOR_DATA_LIDAR) {
				updateTerrainAltDataReliability(altMgrTerrainAltUpdateDt);
				if (fcStatusData.isTerrainAltDataReliable && fcStatusData.isTerrainAltModeActive) {
					updateZPositionTerrain(sensorAltitudeData.altitudeTerrainZOffset, sensorAltitudeData.altitudeTerrain, sensorAltitudeData.altitudeTerrainQual, POSITION_TERRAIN_ALT_DIST_MIN, POSITION_TERRAIN_ALT_DIST_MAX, altMgrTerrainAltUpdateDt);
				}
				altMgrTerrainAltUpdateDt = 0.0f;
			}
		}

	}

}

void resetAltitudeManager(void) {
	resetAltitudeControl(1);
	resetAltitudeSensors(0);
	resetAltMgrStates();
}
