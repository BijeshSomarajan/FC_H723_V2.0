#include "MotorManager.h"

#include "../../calibration/Calibration.h"
#include "../../control/ControlData.h"
#include "../../logger/Logger.h"
#include "../../memory/Memory.h"
#include "../../sensors/rc/RCSensor.h"
#include "../../status/FCStatus.h"
#include "../../timers/GPTimer.h"
#include "../../util/MathUtil.h"

void setPWMChannelValue(uint8_t channel, int value);
void idlePWMs(void);
void motorControlTask(void);

PWM_DATA __ATTR_DTCM_BSS pwmData;
uint8_t motorControlInitStatus = 0;
float motorControlThrustExpoGain = 0.0f;

uint8_t initMotorManager(void) {
	uint8_t status = 1;
	status = initPWM(PWM_MODE_ONESHOT);
	if (!status) {
		logString("[Motor Manager] >> IO >> PWM > Failed\n");
	} else {
		logString("[Motor Manager] >> IO >> PWM > Success\n");
	}
	if (status) {
		initGPTimer6(MOTOR_CONTROL_FREQUENCY, motorControlTask, 5);
		startGPTimer6();
		motorControlThrustExpoGain = get1KXScaledCalibrationValue(CALIB_PROP_THRUST_EXPO_ADDR);
		if (motorControlThrustExpoGain < 0.0f || motorControlThrustExpoGain > 1.0f) {
			motorControlThrustExpoGain = 0; //No expo , In valid inputs.
		}
		motorControlInitStatus = status;
		logString("[Motor Manager] >> Init > Success\n");
	} else {
		logString("[Motor Manager] >> Init > Failed\n");
	}
	stopOutputs();
	return status;
}

__ATTR_ITCM_TEXT
void updatePWMValuesOld() {
	for (uint8_t indx = 0; indx < PWM_CHANNEL_COUNT; indx++) {
		pwmData.PWM_VALUES[indx] = constrainToRange(pwmData.PWM_VALUES[indx], 0, RC_CHANNEL_DELTA_VALUE);
		setPWMChannelValue(indx, pwmData.PWM_VALUES[indx] + MOTOR_PWM_PRESET);
	}
}

/* Map a single motor's linear thrust demand through the inverse thrust curve.
 * Input and output are both in PWM counts [0, RC_CHANNEL_DELTA_VALUE].
 * demand is the post-mix, post-desaturation motor value. */
__ATTR_ITCM_TEXT
float applyThrustLinearization(float demand) {
	float range = (float) RC_CHANNEL_DELTA_VALUE;
	if (range < 1.0f) {
		return demand; /* guard */
	}
	/* normalize to [0,1] */
	float n = demand / range;
	n = constrainToRangeF(n, 0.0f, 1.0f);
	/* blend linear and sqrt by the expo factor */
	float linearized = (1.0f - motorControlThrustExpoGain) * n + motorControlThrustExpoGain * fastSqrtf(n);
	return linearized * range;
}

__ATTR_ITCM_TEXT
void updateMotorPWMValues(void) {
	float maxMotor = pwmData.PWM_VALUES[0];
	float minMotor = pwmData.PWM_VALUES[0];
	// Find bounds
	for (uint8_t i = 1; i < PWM_CHANNEL_COUNT; i++) {
		float value = pwmData.PWM_VALUES[i];
		if (value > maxMotor)
			maxMotor = value;
		if (value < minMotor)
			minMotor = value;
	}
	float satCorrection = 0.0f;
	float span = maxMotor - minMotor;
	/* Case 1: Mixer span fits inside available range */
	if (span <= (float) RC_CHANNEL_DELTA_VALUE) {
		if (minMotor < 0.0f) {
			satCorrection = -minMotor;
		}
		if ((maxMotor + satCorrection) > (float) RC_CHANNEL_DELTA_VALUE) {
			satCorrection -= (maxMotor + satCorrection) - (float) RC_CHANNEL_DELTA_VALUE;
		}
	}
	/* Case 2: Span exceeds range, center it symmetrically */
	else {
		satCorrection = ((float) RC_CHANNEL_DELTA_VALUE * 0.5f) - ((maxMotor + minMotor) * 0.5f);
	}

	// Output stage - Inline evaluation directly into hardware write
	/*
	 for (uint8_t i = 0; i < PWM_CHANNEL_COUNT; i++) {
	 setPWMChannelValue(i, constrainToRangeF(pwmData.PWM_VALUES[i] + satCorrection, 0.0f, (float) RC_CHANNEL_DELTA_VALUE) + MOTOR_PWM_PRESET);
	 }
	 */
	for (uint8_t i = 0; i < PWM_CHANNEL_COUNT; i++) {
		float motorVal = constrainToRangeF(pwmData.PWM_VALUES[i] + satCorrection, 0.0f, (float) RC_CHANNEL_DELTA_VALUE);
		if (motorControlThrustExpoGain > 0.0f) {
			motorVal = applyThrustLinearization(motorVal);      // <-- linearize the final value
		}
		setPWMChannelValue(i, motorVal + MOTOR_PWM_PRESET);
	}
}

__ATTR_ITCM_TEXT
void updatePWMValues() {
	for (uint8_t indx = 0; indx < PWM_CHANNEL_COUNT; indx++) {
		pwmData.PWM_VALUES[indx] = constrainToRange(pwmData.PWM_VALUES[indx], 0, RC_CHANNEL_DELTA_VALUE);
		setPWMChannelValue(indx, pwmData.PWM_VALUES[indx] + MOTOR_PWM_PRESET);
	}
}

__ATTR_ITCM_TEXT
void motorControlTask() {
	if (fcStatusData.canFly) {
		float throttleControl = controlData.throttleControl;
		float pitchControl = controlData.pitchControl;
		float rollControl = controlData.rollControl;
		float yawControl = controlData.yawControl;
		pwmData.PWM_VALUES[0] = (throttleControl - pitchControl - rollControl + yawControl) * controlData.batteryDepletionGain;
		pwmData.PWM_VALUES[1] = (throttleControl - pitchControl + rollControl - yawControl) * controlData.batteryDepletionGain;
		pwmData.PWM_VALUES[2] = (throttleControl + pitchControl - rollControl - yawControl) * controlData.batteryDepletionGain;
		pwmData.PWM_VALUES[3] = (throttleControl + pitchControl + rollControl + yawControl) * controlData.batteryDepletionGain;
		updateMotorPWMValues();
	} else {
		stopOutputs();
	}
}

__ATTR_ITCM_TEXT
void idlePWMs() {
	if (motorControlInitStatus) {
		for (uint8_t indx = 0; indx < PWM_CHANNEL_COUNT; indx++) {
			pwmData.PWM_VALUES[indx] = 0;
		}
		updateMotorPWMValues();
	}
}

__ATTR_ITCM_TEXT
void setPWMChannelValue(uint8_t channel, int value) {
	updatePWM(channel, value < 0 ? 0 : value);
}

__ATTR_ITCM_TEXT
void stopOutputs(void) {
	idlePWMs();
}

