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
#include "../../managers/position/common/PositionCommon.h"
#include "../../managers/position/estimator/VenturiBiasEstimator.h"
#include "../../control/Pid.h"
#include "../../io/uart/UART.h"
#include "../../sensors/position/GNSS.h"

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
	DEBUG_DATA_BUFFER[6] = positionCordinateData.zVelocity * 10;
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
extern VENTURI_ESTIMATE_DATA venturiEstimateData;
float curAlt = 0;

void debugPosition(float dt) {
	if (curAlt == 0) {
		curAlt = 1; //positionData.zPosition;
	}
	DEBUG_DATA_BUFFER[0] = (positionCordinateData.zPosition - curAlt);
	DEBUG_DATA_BUFFER[1] = (sensorAltitudeData.altitudeSLMaxFiltered - curAlt);
	DEBUG_DATA_BUFFER[2] = (sensorAltitudeData.altitudeSLScaled - curAlt);
	DEBUG_DATA_BUFFER[3] = (fcStatusData.altitudeSLRef - curAlt);
	DEBUG_DATA_BUFFER[4] = (positionCordinateData.zVelocity);
	DEBUG_DATA_BUFFER[5] = (controlData.altitudeControl);

	//DEBUG_DATA_BUFFER[4] = venturiEstimateData.venturiBias;
	//DEBUG_DATA_BUFFER[5] = venturiEstimateData.pitchAngleAbsFiltered;
	//DEBUG_DATA_BUFFER[6] = sensorAttitudeData.pitch;
	sendConfigData(DEBUG_DATA_BUFFER, 6, CMD_FC_DATA);
}

void debugBrake(float dt) {

	DEBUG_DATA_BUFFER[0] = rcData.RC_EFFECTIVE_DATA[RC_PITCH_CHANNEL_INDEX];
	DEBUG_DATA_BUFFER[1] = (positionCommandData.pitchCommand);

	DEBUG_DATA_BUFFER[2] = rcData.RC_EFFECTIVE_DATA[RC_ROLL_CHANNEL_INDEX];
	DEBUG_DATA_BUFFER[3] = (positionCommandData.rollCommand);
	sendConfigData(DEBUG_DATA_BUFFER, 4, CMD_FC_DATA);
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

void debugAltThrottle(float dt) {
	DEBUG_DATA_BUFFER[0] = controlData.throttleControl;
	DEBUG_DATA_BUFFER[1] = altMgrThrottleControlLPF.output;
	DEBUG_DATA_BUFFER[2] = controlData.altitudeControl;
	DEBUG_DATA_BUFFER[3] = fcStatusData.currentThrottle;
	DEBUG_DATA_BUFFER[4] = (positionCordinateData.zPosition);
	sendConfigData(DEBUG_DATA_BUFFER, 5, CMD_FC_DATA);
}

void debugGPS() {
	/*
	 char temp[200];
	 sprintf(temp, "Hz:%.2f,MC:%d,NS:%d,FS:%d,Lt:%f,Ln:%f\n", 1.0f / gpsData.updateDt, gpsData.msgCount, gpsData.satCount, gpsData.fixStatus, gpsData.latDeg, gpsData.longDeg);
	 uart5WriteDMA((uint8_t*) temp, strlen(temp));
	 */
	DEBUG_DATA_BUFFER[0] = gnssData.updateDt == 0 ? 1 : 1.0f / gnssData.updateDt;
	DEBUG_DATA_BUFFER[1] = gnssData.satCount;
	DEBUG_DATA_BUFFER[2] = gnssData.fixStatus;
	sendConfigData(DEBUG_DATA_BUFFER, 3, CMD_FC_DATA);
}

void debugPositionXy(float dt) {

	DEBUG_DATA_BUFFER[0] = positionCordinateData.xPositionRaw;
	DEBUG_DATA_BUFFER[1] = positionCordinateData.xPosition;
	DEBUG_DATA_BUFFER[2] = positionCordinateData.xVelocity;

	DEBUG_DATA_BUFFER[3] = positionCordinateData.yPositionRaw;
	DEBUG_DATA_BUFFER[4] = positionCordinateData.yPosition;
	DEBUG_DATA_BUFFER[5] = positionCordinateData.yVelocity;

	DEBUG_DATA_BUFFER[6] = fcStatusData.isGNSSDataReliable * 20;
	DEBUG_DATA_BUFFER[7] = sensorAttitudeData.heading;

	sendConfigData(DEBUG_DATA_BUFFER, 8, CMD_FC_DATA);
}

void debugPositionAlign(float dt) {
	DEBUG_DATA_BUFFER[0] = sensorAttitudeData.heading;
	DEBUG_DATA_BUFFER[1] = positionCordinateData.xVelocity * 100;
	DEBUG_DATA_BUFFER[2] = positionCordinateData.xAcceleration * 10;

	DEBUG_DATA_BUFFER[3] = positionCordinateData.yVelocity * 100;
	DEBUG_DATA_BUFFER[4] = positionCordinateData.yAcceleration * 10;

	sendConfigData(DEBUG_DATA_BUFFER, 5, CMD_FC_DATA);
}

void debugTask() {
	if (!fcStatusData.isDebugEnabled) {
		return;
	}
	float dt = 0.001f;	//getDeltaTime(DEBUG_TIMER_CHANNEL);
	//debugPosition(dt);
	//debugAltThrottle(dt);
	//debugGPS();
	debugPositionXy(dt);
	//debugPositionAlign(dt);
	// debugBrake(dt);
	//debugTime(dt);
	//currentDebug();
}
