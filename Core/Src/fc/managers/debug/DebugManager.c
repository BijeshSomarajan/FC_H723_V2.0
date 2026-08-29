#include "DebugManager.h"

#include <stdio.h>
#include <sys/_stdint.h>

#include "../../control/ControlData.h"
#include "../../control/Pid.h"
#include "../../dsp/BiQuadFilter.h"
#include "../../dsp/FFT.h"
#include "../../dsp/LowPassFilter.h"
#include "../../FCConfig.h"
#include "../../imu/IMU.h"
#include "../../logger/Logger.h"
#include "../../sensors/altitude/AltitudeSensor.h"
#include "../../sensors/attitude/AttitudeSensor.h"
#include "../../sensors/attitude/devices/AttitudeDevice.h"
#include "../../sensors/battery/BatterySensor.h"
#include "../../sensors/position/GNSS.h"
#include "../../sensors/rc/RCSensor.h"
#include "../../status/FCStatus.h"
#include "../../timers/Scheduler.h"
#include "../config/ConfigHelper.h"
#include "../position/common/PositionCommon.h"
#include "../position/estimator/PositionEstimator.h"

int32_t DEBUG_DATA_BUFFER[16];
extern LOWPASSFILTER thControlRefLPF;
extern uint8_t altControlAccEnabled;
extern POSITION_EKF positionEkf;
extern PID positionXPID, positionYPID, positionXRatePID, positionYRatePID;
void debugTask(void);

uint8_t initDebugManager(void) {
	schedulerAddTask(debugTask, DEBUG_TASK_FREQUENCY, DEBUG_TASK_PRIORITY);
	return 1;
}

char buf[256];
extern float testGNSSRV, testGNSSRP;
void debugString() {
	sprintf(buf, "%.2f,%.2f,%.2f\n", sensorAttitudeData.heading, batteryData.voltage, sensorAltitudeData.altitudeTerrain);
	logString(buf);
}

void debugGPS() {
	DEBUG_DATA_BUFFER[0] = sensorAttitudeData.heading;
	DEBUG_DATA_BUFFER[1] = fcStatusData.isNavDataReliable * 1000;
	DEBUG_DATA_BUFFER[2] = fcStatusData.isPositionHomeSet * 1000;
	DEBUG_DATA_BUFFER[3] = positionCordinateData.xPosition * 1000;
	DEBUG_DATA_BUFFER[4] = positionCordinateData.yPosition * 1000;
	sendConfigData(DEBUG_DATA_BUFFER, 5, CMD_FC_DATA);
}

uint8_t sendX = 0;
void debugRC() {
	DEBUG_DATA_BUFFER[0] = fcStatusData.canStart;
	DEBUG_DATA_BUFFER[1] = rcData.RC_DELTA_DATA[RC_TH_CHANNEL_INDEX];
	DEBUG_DATA_BUFFER[2] = rcData.RC_DELTA_DATA[RC_YAW_CHANNEL_INDEX];
	DEBUG_DATA_BUFFER[3] = rcData.RC_DELTA_DATA[RC_PITCH_CHANNEL_INDEX];
	DEBUG_DATA_BUFFER[4] = rcData.RC_DELTA_DATA[RC_ROLL_CHANNEL_INDEX];
	DEBUG_DATA_BUFFER[5] = rcData.RC_DELTA_DATA[RC_NAV_CHANNEL_INDEX];
	DEBUG_DATA_BUFFER[6] = fcStatusData.isLandingModeActive;
	DEBUG_DATA_BUFFER[7] = fcStatusData.isTerrainAltModeActive;
	DEBUG_DATA_BUFFER[8] = fcStatusData.isNavModeActive;
	DEBUG_DATA_BUFFER[9] = fcStatusData.isNavRTHModeActive;

	sendConfigData(DEBUG_DATA_BUFFER, 10, CMD_FC_DATA);
}

void debugPID() {
	DEBUG_DATA_BUFFER[0] = sensorAttitudeData.pitchRate * 10;
	DEBUG_DATA_BUFFER[1] = sensorAttitudeData.pitchCtrlRate * 10;
	DEBUG_DATA_BUFFER[2] = controlData.pitchControl * 10;

	DEBUG_DATA_BUFFER[3] = sensorAttitudeData.rollRate * 10;
	DEBUG_DATA_BUFFER[4] = sensorAttitudeData.rollCtrlRate * 10;
	DEBUG_DATA_BUFFER[5] = controlData.rollControl * 10;

	DEBUG_DATA_BUFFER[6] = sensorAttitudeData.yawRate * 10;
	DEBUG_DATA_BUFFER[7] = sensorAttitudeData.yawCtrlRate * 10;
	DEBUG_DATA_BUFFER[8] = controlData.yawControl * 10;

	sendConfigData(DEBUG_DATA_BUFFER, 9, CMD_FC_DATA);
}

void debugNoise() {

	DEBUG_DATA_BUFFER[0] = sensorAttitudeData.gxDS * 10;
	DEBUG_DATA_BUFFER[1] = sensorAttitudeData.gxDSFiltered * 10;
	DEBUG_DATA_BUFFER[2] = sensorAttitudeData.gyDS * 10;
	DEBUG_DATA_BUFFER[3] = sensorAttitudeData.gyDSFiltered * 10;
	DEBUG_DATA_BUFFER[4] = sensorAttitudeData.gzDS * 10;
	DEBUG_DATA_BUFFER[5] = sensorAttitudeData.gzDSFiltered * 10;
	DEBUG_DATA_BUFFER[6] = positionCordinateData.zVelocity * 1000;
	DEBUG_DATA_BUFFER[7] = fcStatusData.currentThrottle;
	sendConfigData(DEBUG_DATA_BUFFER, 8, CMD_FC_DATA);
}

void debugIMU() {
	DEBUG_DATA_BUFFER[0] = sensorAttitudeData.pitch * 10;
	DEBUG_DATA_BUFFER[1] = sensorAttitudeData.pitchRate * 10;
	DEBUG_DATA_BUFFER[2] = sensorAttitudeData.roll * 10;
	DEBUG_DATA_BUFFER[3] = sensorAttitudeData.rollRate * 10;
	DEBUG_DATA_BUFFER[4] = sensorAttitudeData.heading * 10;
	DEBUG_DATA_BUFFER[5] = sensorAttitudeData.yawRate * 10;
	sendConfigData(DEBUG_DATA_BUFFER, 6, CMD_FC_DATA);
}

void debugIMUStr() {
	sprintf(buf, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", sensorAttitudeData.pitch, sensorAttitudeData.pitchRate, controlData.pitchControl, sensorAttitudeData.roll, sensorAttitudeData.rollRate, controlData.rollControl, sensorAttitudeData.heading, sensorAttitudeData.yawRate,
			controlData.yawControl);
	logString(buf);
}

void debugBattery() {
	DEBUG_DATA_BUFFER[0] = fcStatusData.batteryNomVolt * 10;
	DEBUG_DATA_BUFFER[1] = fcStatusData.batteryType * 10;
	DEBUG_DATA_BUFFER[2] = fcStatusData.batteryAlertState * 10;
	DEBUG_DATA_BUFFER[3] = batteryData.voltage * 10;
	DEBUG_DATA_BUFFER[4] = controlData.batteryDepletionGain * 10;
	sendConfigData(DEBUG_DATA_BUFFER, 5, CMD_FC_DATA);
}

extern IMU_DATA imuData;
void debugALt() {
	DEBUG_DATA_BUFFER[0] = sensorAttitudeData.azG * 1000;
	DEBUG_DATA_BUFFER[1] = deviceAttitudeData.azG * 1000;
	DEBUG_DATA_BUFFER[2] = sensorAttitudeData.azGFilteredImu * 1000;
	DEBUG_DATA_BUFFER[3] = imuData.azEarthLinear * 1000;
	DEBUG_DATA_BUFFER[4] = imuData.azBodyLinear * 1000;
	DEBUG_DATA_BUFFER[5] = positionCordinateData.zAcceleration * 1000;
	DEBUG_DATA_BUFFER[6] = positionCordinateData.zVelocity * 1000;
	sendConfigData(DEBUG_DATA_BUFFER, 7, CMD_FC_DATA);
}

void debugGnssData() {
	sprintf(buf, "Fix:%d,hAcc:%.2f,vAcc:%.2f,SN:%d,Rel:%d,Hme:%d\n", gnssData.fixType, gnssData.hAcc, gnssData.vAcc, gnssData.satCount, fcStatusData.isNavDataReliable, fcStatusData.isPositionHomeSet);
	logString(buf);
}

extern FFTContext fftContextGyroX;
extern FFTContext fftContextGyroY;
extern BIQUADFILTER noiseFilterFftNtfGyroX[2];
extern BIQUADFILTER noiseFilterFftNtfGyroY[2];

void debufFFT(){
	sprintf(buf,"[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]\r\n",
	       fftContextGyroX.topFreqBin[0],
	       noiseFilterFftNtfGyroX[0].center_freq,
	       fftContextGyroX.topFreqBin[1],
	       noiseFilterFftNtfGyroX[1].center_freq,
	       sensorAttitudeData.gxDS,
	       sensorAttitudeData.gxDSFiltered,
	       fftContextGyroY.topFreqBin[0],
	       noiseFilterFftNtfGyroY[0].center_freq,
	       fftContextGyroY.topFreqBin[1],
	       noiseFilterFftNtfGyroY[1].center_freq,
	       sensorAttitudeData.gyDS,
	       sensorAttitudeData.gyDSFiltered);
	logString(buf);

}

float nowMs = 0;
void debugTask() {
	if (!fcStatusData.isDebugEnabled) {
		return;
	}
	float dt = 1.0f / DEBUG_TASK_FREQUENCY;
	(void) dt;
//nowMs += dt;
//debugBattery();
//debugRC();
debugIMU();
//debugALt();
//debugGnssData();
//	debugIMUStr();
//	debufFFT();
//	debugNoise();
}
