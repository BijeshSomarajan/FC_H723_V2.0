#include "AttitudeControl.h"

#include <sys/_stdint.h>

#include "../../calibration/Calibration.h"
#include "../../memory/Memory.h"
#include "../../sensors/attitude/AttitudeSensor.h"
#include "../../status/FCStatus.h"
#include "../ControlData.h"
#include "../Pid.h"

void configureAttitudeControl(float rollPitchRatio);
//Attitude PID references
PID attitudePitchPID, attitudeRollPID, attitudeYawPID, attitudePitchRatePID, attitudeRollRatePID, attitudeYawRatePID;

/**
 * Initializes the attitude control
 */
uint8_t initAttitudeControl() {

	/* Attitude Master PID initiation*/
	pidInit(&attitudePitchPID, getScaledCalibrationValue(CALIB_PROP_PID_KP_PITCH_ADDR), 0, 0, 0);
	pidInit(&attitudeRollPID, getScaledCalibrationValue(CALIB_PROP_PID_KP_ROLL_ADDR), 0, 0, 0);
	pidInit(&attitudeYawPID, getScaledCalibrationValue(CALIB_PROP_PID_KP_YAW_ADDR), 0, 0, 0);
	//Set overall limit
	pidSetPIDOutputLimits(&attitudePitchPID, -getCalibrationValue(CALIB_PROP_PID_PITCH_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_PID_PITCH_LIMIT_ADDR));
	pidSetPIDOutputLimits(&attitudeRollPID, -getCalibrationValue(CALIB_PROP_PID_ROLL_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_PID_ROLL_LIMIT_ADDR));
	pidSetPIDOutputLimits(&attitudeYawPID, -getCalibrationValue(CALIB_PROP_PID_YAW_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_PID_YAW_LIMIT_ADDR));
	//Set P limit
	pidSetPOutputLimits(&attitudePitchPID, -getCalibrationValue(CALIB_PROP_PID_PITCH_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_PID_PITCH_LIMIT_ADDR));
	pidSetPOutputLimits(&attitudeRollPID, -getCalibrationValue(CALIB_PROP_PID_ROLL_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_PID_ROLL_LIMIT_ADDR));
	pidSetPOutputLimits(&attitudeYawPID, -getCalibrationValue(CALIB_PROP_PID_YAW_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_PID_YAW_LIMIT_ADDR));

	/* Attitude Rate PID Settings*/
	pidInit(&attitudePitchRatePID, getScaledCalibrationValue(CALIB_PROP_RATE_PID_KP_PITCH_ADDR), getScaledCalibrationValue(CALIB_PROP_RATE_PID_KI_PITCH_ADDR), getScaledCalibrationValue(CALIB_PROP_RATE_PID_KD_PITCH_ADDR), ATT_CONTROL_D_RATE_LPF_FREQ);
	pidInit(&attitudeRollRatePID, getScaledCalibrationValue(CALIB_PROP_RATE_PID_KP_ROLL_ADDR), getScaledCalibrationValue(CALIB_PROP_RATE_PID_KI_ROLL_ADDR), getScaledCalibrationValue(CALIB_PROP_RATE_PID_KD_ROLL_ADDR), ATT_CONTROL_D_RATE_LPF_FREQ);
	pidInit(&attitudeYawRatePID, getScaledCalibrationValue(CALIB_PROP_RATE_PID_KP_YAW_ADDR), getScaledCalibrationValue(CALIB_PROP_RATE_PID_KI_YAW_ADDR), getScaledCalibrationValue(CALIB_PROP_RATE_PID_KD_YAW_ADDR), ATT_CONTROL_D_RATE_LPF_FREQ);
	//Set overall limit
	pidSetPIDOutputLimits(&attitudePitchRatePID, -getCalibrationValue(CALIB_PROP_RATE_PID_PITCH_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_RATE_PID_PITCH_LIMIT_ADDR));
	pidSetPIDOutputLimits(&attitudeRollRatePID, -getCalibrationValue(CALIB_PROP_RATE_PID_ROLL_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_RATE_PID_ROLL_LIMIT_ADDR));
	pidSetPIDOutputLimits(&attitudeYawRatePID, -getCalibrationValue(CALIB_PROP_RATE_PID_YAW_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_RATE_PID_YAW_LIMIT_ADDR));
	//Set P limit
	pidSetPOutputLimits(&attitudePitchRatePID, -getCalibrationValue(CALIB_PROP_RATE_PID_PITCH_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_RATE_PID_PITCH_LIMIT_ADDR));
	pidSetPOutputLimits(&attitudeRollRatePID, -getCalibrationValue(CALIB_PROP_RATE_PID_ROLL_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_RATE_PID_ROLL_LIMIT_ADDR));
	pidSetPOutputLimits(&attitudeYawRatePID, -getCalibrationValue(CALIB_PROP_RATE_PID_YAW_LIMIT_ADDR), getCalibrationValue(CALIB_PROP_RATE_PID_YAW_LIMIT_ADDR));
	//Set I limit
	pidSetIOutputLimits(&attitudePitchRatePID, -getCalibrationValue(CALIB_PROP_PID_PITCH_LIMIT_ADDR) * ATT_CONTROL_RATE_PID_I_LIMIT_RATIO, getCalibrationValue(CALIB_PROP_PID_PITCH_LIMIT_ADDR) * ATT_CONTROL_RATE_PID_I_LIMIT_RATIO);
	pidSetIOutputLimits(&attitudeRollRatePID, -getCalibrationValue(CALIB_PROP_PID_ROLL_LIMIT_ADDR) * ATT_CONTROL_RATE_PID_I_LIMIT_RATIO, getCalibrationValue(CALIB_PROP_PID_ROLL_LIMIT_ADDR) * ATT_CONTROL_RATE_PID_I_LIMIT_RATIO);
	pidSetIOutputLimits(&attitudeYawRatePID, -getCalibrationValue(CALIB_PROP_PID_YAW_LIMIT_ADDR) * ATT_CONTROL_RATE_PID_I_LIMIT_RATIO, getCalibrationValue(CALIB_PROP_PID_YAW_LIMIT_ADDR) * ATT_CONTROL_RATE_PID_I_LIMIT_RATIO);
	//Set D limit
	pidSetDOutputLimits(&attitudePitchRatePID, -getCalibrationValue(CALIB_PROP_PID_PITCH_LIMIT_ADDR) * ATT_CONTROL_RATE_PID_D_LIMIT_RATIO, getCalibrationValue(CALIB_PROP_PID_PITCH_LIMIT_ADDR) * ATT_CONTROL_RATE_PID_D_LIMIT_RATIO);
	pidSetDOutputLimits(&attitudeRollRatePID, -getCalibrationValue(CALIB_PROP_PID_ROLL_LIMIT_ADDR) * ATT_CONTROL_RATE_PID_D_LIMIT_RATIO, getCalibrationValue(CALIB_PROP_PID_ROLL_LIMIT_ADDR) * ATT_CONTROL_RATE_PID_D_LIMIT_RATIO);
	pidSetDOutputLimits(&attitudeYawRatePID, -getCalibrationValue(CALIB_PROP_PID_YAW_LIMIT_ADDR) * ATT_CONTROL_RATE_PID_D_LIMIT_RATIO, getCalibrationValue(CALIB_PROP_PID_YAW_LIMIT_ADDR) * ATT_CONTROL_RATE_PID_D_LIMIT_RATIO);
	return 1;
}

/**
 * Resets Attitude control
 */
void resetAttitudeControl(uint8_t hard) {
	if (hard) {
		//Reset the master PIDs
		pidReset(&attitudePitchPID);
		pidReset(&attitudeRollPID);
		pidReset(&attitudeYawPID);
		//Reset the rate PIDs
		pidReset(&attitudePitchRatePID);
		pidReset(&attitudeRollRatePID);
		pidReset(&attitudeYawRatePID);
		//Reset process control values
		controlData.pitchControl = 0.0f;
		controlData.rollControl = 0.0f;
		controlData.yawControl = 0.0f;
	}
	//Reset the master PIDs
	pidResetI(&attitudePitchPID);
	pidResetI(&attitudeRollPID);
	pidResetI(&attitudeYawPID);
	//Reset the master PIDs
	pidResetI(&attitudePitchRatePID);
	pidResetI(&attitudeRollRatePID);
	pidResetI(&attitudeYawRatePID);
}

/************************************************************************/

__ATTR_ITCM_TEXT
float updateHeadingDelta() {
	float headingDelta = sensorAttitudeData.heading - fcStatusData.headingRef;
	// Use fmodf to bring the value into the range (-360, 360)
	headingDelta = fmodf(headingDelta, 360.0f);
	// Adjust the result to the range [-180, 180]
	if (headingDelta > 180.0f) {
		headingDelta -= 360.0f;
	} else if (headingDelta < -180.0f) {
		headingDelta += 360.0f;
	}
	return headingDelta;
}

__ATTR_ITCM_TEXT
void controlAttitudeWithGains(float dt, float expectedPitch, float expectedRoll, float expectedYaw, float rateIGain, float rateDGain) {
	float headingDelta = updateHeadingDelta();
	fcStatusData.headingDelta = headingDelta;
	float pitch = sensorAttitudeData.pitch;
	float roll = sensorAttitudeData.roll;
	float pitchRate = sensorAttitudeData.pitchRate;
	float rollRate = sensorAttitudeData.rollRate;
	float yawRate = sensorAttitudeData.yawRate;
	/*
	 float headingDelta = 0;
	 float pitch = 0;
	 float roll = 0;
	 float pitchRate = 0;
	 float rollRate = 0;
	 float yawRate = 0;
	 */
	pidUpdate(&attitudePitchPID, pitch, expectedPitch, dt);
	pidUpdate(&attitudeRollPID, roll, expectedRoll, dt);
	pidUpdate(&attitudeYawPID, headingDelta, expectedYaw, dt);

	pidUpdateWithGains(&attitudePitchRatePID, pitchRate, attitudePitchPID.pid, dt, 1.0f, rateIGain, rateDGain);
	pidUpdateWithGains(&attitudeRollRatePID, rollRate, attitudeRollPID.pid, dt, 1.0f, rateIGain, rateDGain);
	pidUpdateWithGains(&attitudeYawRatePID, yawRate, attitudeYawPID.pid, dt, 1.0f, rateIGain, rateDGain);

	controlData.pitchControl = attitudePitchRatePID.pid;
	controlData.rollControl = attitudeRollRatePID.pid;
	controlData.yawControl = attitudeYawRatePID.pid;


	// controlData.pitchControl = 0;
	// controlData.rollControl = 0;
	// controlData.yawControl = 0;

	controlData.attitudeControlDt = dt;
}

__ATTR_ITCM_TEXT
void controlAttitude(float dt, float expectedPitch, float expectedRoll, float expectedYaw) {
	float headingDelta = updateHeadingDelta();
	fcStatusData.headingDelta = headingDelta;
	float pitch = sensorAttitudeData.pitch;
	float roll = sensorAttitudeData.roll;
	float pitchRate = sensorAttitudeData.pitchRate;
	float rollRate = sensorAttitudeData.rollRate;
	float yawRate = sensorAttitudeData.yawRate;
	/*
	 float headingDelta = 0;
	 float pitch = 0;
	 float roll = 0;
	 float pitchRate = 0;
	 float rollRate = 0;
	 float yawRate = 0;
	 */
	pidUpdate(&attitudePitchPID, pitch, expectedPitch, dt);
	pidUpdate(&attitudeRollPID, roll, expectedRoll, dt);
	pidUpdate(&attitudeYawPID, headingDelta, expectedYaw, dt);

	pidUpdate(&attitudePitchRatePID, pitchRate, attitudePitchPID.pid, dt);
	pidUpdate(&attitudeRollRatePID, rollRate, attitudeRollPID.pid, dt);
	pidUpdate(&attitudeYawRatePID, yawRate, attitudeYawPID.pid, dt);

	controlData.pitchControl = attitudePitchRatePID.pid;
	controlData.rollControl = attitudeRollRatePID.pid;
	controlData.yawControl = attitudeYawRatePID.pid;

	/*
	 controlData.pitchControl = 0;
	 controlData.rollControl = 0;
	 controlData.yawControl = 0;
	 */
	controlData.attitudeControlDt = dt;
}

