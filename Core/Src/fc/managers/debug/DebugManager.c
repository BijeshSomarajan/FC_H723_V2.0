#include "DebugManager.h"

#include <sys/_stdint.h>

#include "../../control/ControlData.h"
#include "../../control/altitude/AltitudeControl.h"
#include "../../imu/IMU.h"
#include "../../sensors/altitude/devices/AltitudeDevice.h"
#include "../../sensors/altitude/AltitudeSensor.h"
#include "../../sensors/attitude/AttitudeSensor.h"
#include "../../sensors/attitude/noisefilter/AttitudeNoiseFilter.h"
#include "../../sensors/rc/RCSensor.h"
#include "../../status/FCStatus.h"
#include "../../timers/DeltaTimer.h"
#include "../../timers/Scheduler.h"
#include "../config/ConfigHelper.h"
#include "../../FCConfig.h"
#include "../output/OutputManager.h"
#include "../../dsp/BiQuadFilter.h"
#include "../../dsp/FFT.h"
#include "../../sensors/attitude/noisefilter/AdaptiveNotchFilter.h"
#include "../../managers/position/PositionManager.h"
#include "../../control/Pid.h"

int32_t DEBUG_DATA_BUFFER[8];
extern LOWPASSFILTER thControlRefLPF;
extern uint8_t altControlAccEnabled;

void debugTask(void);

uint8_t initDebugManager(void) {
	schedulerAddTask(debugTask, DEBUG_FREQUENCY, DEBUG_TASK_PRIORITY);
	return 1;
}

void debugTime(float dt) {
	DEBUG_DATA_BUFFER[0] = 1.0f / (imuData.arhsDt == 0 ? 1 : imuData.arhsDt);
	DEBUG_DATA_BUFFER[1] = 1.0f / (sensorAttitudeData.agtDataUpdateDt == 0 ? 1 : sensorAttitudeData.agtDataUpdateDt);
	DEBUG_DATA_BUFFER[2] = 1.0f / (sensorAttitudeData.magDataUpdateDt == 0 ? 1 : sensorAttitudeData.magDataUpdateDt);
	DEBUG_DATA_BUFFER[3] = 1.0f / (sensorAttitudeData.noiseFilterProcessXDt == 0 ? 1 : sensorAttitudeData.noiseFilterProcessXDt);
	DEBUG_DATA_BUFFER[4] = 1.0f / (sensorAltitudeData.altUpdateDt == 0 ? 1 : sensorAltitudeData.altUpdateDt);
	DEBUG_DATA_BUFFER[5] = 1.0f / (rcData.updateDt == 0 ? 1 : rcData.updateDt);
	DEBUG_DATA_BUFFER[6] = 1.0f / (pwmData.updateDt == 0 ? 1 : pwmData.updateDt);
	sendConfigData(DEBUG_DATA_BUFFER, 8, CMD_FC_DATA);
}

void debugImu(float dt) {
	DEBUG_DATA_BUFFER[0] = sensorAttitudeData.pitchRate * 10;
	DEBUG_DATA_BUFFER[1] = sensorAttitudeData.pitch * 10;
	DEBUG_DATA_BUFFER[2] = sensorAttitudeData.rollRate * 10;
	DEBUG_DATA_BUFFER[3] = sensorAttitudeData.roll * 10;
	DEBUG_DATA_BUFFER[4] = sensorAttitudeData.yawRate * 10;
	DEBUG_DATA_BUFFER[5] = sensorAttitudeData.heading;
	DEBUG_DATA_BUFFER[6] = positionData.zVelocity * 10;
	DEBUG_DATA_BUFFER[7] = 1.0f / (imuData.arhsDt == 0 ? 1 : imuData.arhsDt);
	sendConfigData(DEBUG_DATA_BUFFER, 8, CMD_FC_DATA);
}

extern float altMgrCurrentThrottleDelta;
extern float altMgrCurrentThrottleRate;
extern float altMgrPreviousCurrentThrottle;
extern float alrMgrThrottleAggregationDt;
extern float altMgrCurrentThrottleRateGain;
extern float altControlMasterPGain;
extern LOWPASSFILTER altMgrThrottleControlLPF;
extern ALTITUDE_CONTROL_GAINS altControlGains;
extern PID altRatePID;
float curAlt = 0;

void debugPosition(float dt) {
	if (curAlt == 0) {
		curAlt = sensorAltitudeData.altitudeSLFiltered;
	}
	DEBUG_DATA_BUFFER[0] = positionData.zAcceleration;
	DEBUG_DATA_BUFFER[1] = positionData.zVelocity;
	DEBUG_DATA_BUFFER[2] = (sensorAltitudeData.altitudeSLFiltered - curAlt) * 10;
	DEBUG_DATA_BUFFER[3] = (positionData.zPosition - curAlt) * 10;
	DEBUG_DATA_BUFFER[4] = 1.0f / (positionData.positionProcessDt == 0 ? 1 : positionData.positionProcessDt);
	sendConfigData(DEBUG_DATA_BUFFER, 5, CMD_FC_DATA);
}

void currentDebug() {
	/*
	 DEBUG_DATA_BUFFER[0] = 1.0f / (imuData.arhsDt == 0 ? 1 : imuData.arhsDt);
	 DEBUG_DATA_BUFFER[1] = 1.0f / (sensorAttitudeData.agtDataUpdateDt == 0 ? 1 : sensorAttitudeData.agtDataUpdateDt);
	 DEBUG_DATA_BUFFER[2] = 1.0f / (sensorAttitudeData.noiseFilterProcessXDt == 0 ? 1 : sensorAttitudeData.noiseFilterProcessXDt);
	 DEBUG_DATA_BUFFER[3] = 1.0f / (sensorAltitudeData.altUpdateDt == 0 ? 1 : sensorAltitudeData.altUpdateDt);
	 DEBUG_DATA_BUFFER[4] = 1.0f / (sensorAltitudeData.altProcessDt == 0 ? 1 : sensorAltitudeData.altProcessDt);
	 DEBUG_DATA_BUFFER[5] = 1.0f / (positionData.positionProcessDt == 0 ? 1 : positionData.positionProcessDt);
	 DEBUG_DATA_BUFFER[6] = 1.0f / (positionData.positionZUpdateDt == 0 ? 1 : positionData.positionZUpdateDt);
	 DEBUG_DATA_BUFFER[7] = 1.0f / (pwmData.updateDt == 0 ? 1 : pwmData.updateDt);
	 */

	sendConfigData(DEBUG_DATA_BUFFER, 8, CMD_FC_DATA);
}

void debugTask() {
	if (!fcStatusData.isDebugEnabled) {
		return;
	}
	float dt = 0.001f;	//getDeltaTime(DEBUG_TIMER_CHANNEL);
	debugPosition(dt);
	//debugTime(dt);
	//currentDebug();
}
