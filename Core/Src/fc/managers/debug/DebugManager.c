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
#include "../../util/MathUtil.h"
#include "../../util/CommonUtil.h"

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
		curAlt = positionCordinateData.zPosition;
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
	/*
	 DEBUG_DATA_BUFFER[0] = controlData.throttleControl;
	 DEBUG_DATA_BUFFER[1] = altMgrThrottleControlLPF.output;
	 DEBUG_DATA_BUFFER[2] = controlData.altitudeControl;
	 DEBUG_DATA_BUFFER[3] = fcStatusData.currentThrottle;
	 */
	DEBUG_DATA_BUFFER[0] = deviceAltitudeData.altitude * 100;
	DEBUG_DATA_BUFFER[1] = (positionCordinateData.zVelocity);
	DEBUG_DATA_BUFFER[2] = (positionCordinateData.zPosition);
	DEBUG_DATA_BUFFER[3] = sensorAltitudeData.altitudeSLScaled;

	sendConfigData(DEBUG_DATA_BUFFER, 6, CMD_FC_DATA);
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
//DEBUG_DATA_BUFFER[0] = sensorAttitudeData.heading;
	/*
	 DEBUG_DATA_BUFFER[0] = imuData.axEarthLinear * 1000;
	 DEBUG_DATA_BUFFER[1] = imuData.axEarthLinear1 * 1000;
	 DEBUG_DATA_BUFFER[2] = imuData.axEarthLinear2 * 1000;
	 DEBUG_DATA_BUFFER[3] = imuData.ayEarthLinear * 1000;
	 DEBUG_DATA_BUFFER[4] = imuData.ayEarthLinear1 * 1000;
	 DEBUG_DATA_BUFFER[5] = imuData.ayEarthLinear2 * 1000;
	 DEBUG_DATA_BUFFER[6] = imuData.azEarthLinear * 1000;
	 DEBUG_DATA_BUFFER[7] = sensorAttitudeData.heading * 10;
	 */
	/*
	 DEBUG_DATA_BUFFER[0] = sensorAttitudeData.heading * 10;
	 DEBUG_DATA_BUFFER[1] = controlData.positionXControl * 100;
	 DEBUG_DATA_BUFFER[2] = controlData.pitchControl * 100;
	 DEBUG_DATA_BUFFER[3] = controlData.positionYControl * 100;
	 DEBUG_DATA_BUFFER[4] = controlData.rollControl * 100;
	 */

	DEBUG_DATA_BUFFER[0] = sensorAttitudeData.heading * 10;
	DEBUG_DATA_BUFFER[1] = sensorAttitudeData.mxFiltered * 100;
	DEBUG_DATA_BUFFER[2] = sensorAttitudeData.myFiltered * 100;
	DEBUG_DATA_BUFFER[3] = sensorAttitudeData.mzFiltered * 100;

	/*
	 DEBUG_DATA_BUFFER[6] = gnssData.latitude * 10000000;
	 DEBUG_DATA_BUFFER[7] = gnssData.longitude * 10000000;
	 */

	sendConfigData(DEBUG_DATA_BUFFER, 4, CMD_FC_DATA);
}
extern float attitudeAngleControlDt;
void debugPositionAlign(float dt) {
	DEBUG_DATA_BUFFER[0] = sensorAttitudeData.pitch * 10;
	DEBUG_DATA_BUFFER[1] = sensorAttitudeData.roll * 10;
	DEBUG_DATA_BUFFER[2] = sensorAttitudeData.heading * 10;
	DEBUG_DATA_BUFFER[3] = 1.0f / attitudeAngleControlDt;
	/*
	 //DEBUG_DATA_BUFFER[0] = positionCordinateData.xPosition  * 10;
	 DEBUG_DATA_BUFFER[1] = positionCordinateData.xVelocity * 10;
	 DEBUG_DATA_BUFFER[2] = controlData.positionXControl * 10;
	 DEBUG_DATA_BUFFER[3] = positionCommandData.pitchCommand * 10;

	 DEBUG_DATA_BUFFER[4] = positionCordinateData.yPosition * 10;
	 DEBUG_DATA_BUFFER[5] = positionCordinateData.yVelocity * 10;
	 DEBUG_DATA_BUFFER[6] = controlData.positionYControl * 10;
	 DEBUG_DATA_BUFFER[7] = positionCommandData.rollCommand * 10;
	 */
	sendConfigData(DEBUG_DATA_BUFFER, 4, CMD_FC_DATA);
}

void debugTilt() {
	DEBUG_DATA_BUFFER[0] = sensorAttitudeData.pitch;
	DEBUG_DATA_BUFFER[1] = sensorAttitudeData.roll;
	DEBUG_DATA_BUFFER[2] = controlData.tiltCompThDelta ;
	DEBUG_DATA_BUFFER[3] = controlData.throttleControl * 10;
	DEBUG_DATA_BUFFER[4] = controlData.pitchControl;
	DEBUG_DATA_BUFFER[5] = controlData.rollControl;
	sendConfigData(DEBUG_DATA_BUFFER, 7, CMD_FC_DATA);
}

void debugTask() {
	if (!fcStatusData.isDebugEnabled) {
		return;
	}
	float dt = 0.001f;	//getDeltaTime(DEBUG_TIMER_CHANNEL);
	debugTilt();
	//debugPositionAlign(dt);
	//debugPosition(dt);
	//debugAltThrottle(dt);
	//debugGPS();
	//debugPosition(dt);
	//debugPositionXy(dt);
	//debugPositionAlign(dt);
	// debugBrake(dt);
	//debugTime(dt);
	//currentDebug();
}
