#include "DebugManager.h"

#include <sys/_stdint.h>

#include "../../control/ControlData.h"
#include "../../control/altitude/AltitudeControl.h"
#include "../../imu/IMU.h"
#include "../../sensors/altitude/devices/AltitudeDevice.h"
#include "../../sensors/altitude/AltitudeSensor.h"
#include "../../sensors/attitude/AttitudeSensor.h"
#include "../../sensors/attitude/noisefilter/AttitudeNoiseFilter.h"
#include "../../sensors/battery/BatterySensor.h"
#include "../../sensors/rc/RCSensor.h"
#include "../../status/FCStatus.h"
#include "../../timers/DeltaTimer.h"
#include "../../timers/Scheduler.h"
#include "../config/ConfigHelper.h"
#include "../../FCConfig.h"
#include "../../dsp/BiQuadFilter.h"
#include "../../dsp/FFT.h"
#include "../../sensors/altitude/devices/tfmini/TFMini.h"
#include "../../sensors/attitude/noisefilter/AdaptiveNotchFilter.h"
#include "../../managers/position/common/PositionCommon.h"
#include "../../managers/position/estimator/VenturiBiasEstimator.h"
#include "../../managers/position/estimator/PositionEstimator.h"
#include "../../control/Pid.h"
#include "../../io/uart/UART.h"
#include "../../sensors/position/GNSS.h"
#include "../../util/MathUtil.h"
#include "../../util/CommonUtil.h"
#include "../motor/MotorManager.h"
#include "../../logger/Logger.h"
#include "../../sensors/rc/RCTelemetry.h"
#include "../../control/position/PositionControl.h"

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

void debugBattery() {
	DEBUG_DATA_BUFFER[0] = positionCordinateData.zPosition * 100;
	DEBUG_DATA_BUFFER[1] = positionCordinateData.zVelocity * 100;
	DEBUG_DATA_BUFFER[2] = fcStatusData.altitudeRef * 100;
	DEBUG_DATA_BUFFER[3] = fcStatusData.headingRef * 10;

	sendConfigData(DEBUG_DATA_BUFFER, 4, CMD_FC_DATA);
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
void debugCRSF() {
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
	;
	DEBUG_DATA_BUFFER[3] = sensorAttitudeData.gyDSFiltered * 10;
	DEBUG_DATA_BUFFER[4] = sensorAttitudeData.gzDS * 10;
	;
	DEBUG_DATA_BUFFER[5] = sensorAttitudeData.gzDSFiltered * 10;
	;
	sendConfigData(DEBUG_DATA_BUFFER, 6, CMD_FC_DATA);
}

void debugIMU() {

	DEBUG_DATA_BUFFER[0] = sensorAttitudeData.pitch * 10;
	DEBUG_DATA_BUFFER[1] = sensorAttitudeData.roll * 10;
	DEBUG_DATA_BUFFER[2] = sensorAttitudeData.heading * 10;

	DEBUG_DATA_BUFFER[3] = sensorAttitudeData.pitchRate * 10;
	DEBUG_DATA_BUFFER[4] = sensorAttitudeData.rollRate * 10;
	DEBUG_DATA_BUFFER[5] = sensorAttitudeData.yawRate * 10;

	sendConfigData(DEBUG_DATA_BUFFER, 6, CMD_FC_DATA);
}

void debugAlt() {
	//sendConfigData(DEBUG_DATA_BUFFER, 4, CMD_FC_DATA);
	sprintf(buf, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n", positionCordinateData.zPosition, positionCordinateData.zVelocity, sensorAltitudeData.altitudeSLFiltered, sensorAttitudeData.pitch, venturiEstimateData.venturiBias, venturiEstimateData.lateralSpeed,
			controlData.tiltCompThDelta, controlData.throttleControl, fcStatusData.currentThrottle, positionEkf.x[POS_EKF_Z_AXIS * POS_EKF_AXIS_DIM + POS_EKF_STATE_BP], venturiEstimateData.brakeDwell);
	logString(buf);
}

float dtAcc;
void debugPosHold(float dt) {
	dtAcc+=dt;
	//sprintf(buf, "%.1f,%lf,%lf,%lf,%lf,%.4f,%.4f,%.4f,%.4f,%d,%d,%d,%.2f,%.2f\r\n",dtAcc,gnssData.latitude,gnssData.longitude,fcStatusData.positionLatHome,fcStatusData.positionLongHome,fcStatusData.positionXRef,positionCordinateData.xPosition,fcStatusData.positionYRef,positionCordinateData.yPosition,fcStatusData.isNavModeActive,fcStatusData.postionHoldState,fcStatusData.isPositionHomeSet,positionXPID.pid,positionYPID.pid);

	sprintf(buf, "%.1f,%lf,%lf,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d\r\n",dtAcc,gnssData.latitude,gnssData.longitude,fcStatusData.positionXRef,positionCordinateData.xPosition,positionCordinateData.xVelocity,fcStatusData.positionYRef,positionCordinateData.yPosition,positionCordinateData.yVelocity,positionXPID.pid,positionYPID.pid,positionXRatePID.pid,positionYRatePID.pid,fcStatusData.isNavModeActive,fcStatusData.postionHoldState);

	logString(buf);
}

void debugTask() {
	if (!fcStatusData.isDebugEnabled) {
		return;
	}
	float dt = 1.0f/DEBUG_TASK_FREQUENCY;
	(void) dt;
	//debugString();
	//debugGraph();
	//debugRC();
	//debugBattery();
	//debugCRSF();
	//debugNoise();
	//debugPID();
	//debugIMU();
	//debugAlt();
	debugPosHold(dt);
}
