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
#include "../motor/MotorManager.h"

int32_t DEBUG_DATA_BUFFER[16];
extern LOWPASSFILTER thControlRefLPF;
extern uint8_t altControlAccEnabled;

void debugTask(void);

uint8_t initDebugManager(void) {
	schedulerAddTask(debugTask, DEBUG_TASK_FREQUENCY, DEBUG_TASK_PRIORITY);
	return 1;
}

void debugNoise() {
	DEBUG_DATA_BUFFER[0] = sensorAttitudeData.gxDS * 10;
	DEBUG_DATA_BUFFER[1] = sensorAttitudeData.gxDSFiltered * 10;
	DEBUG_DATA_BUFFER[2] = sensorAttitudeData.gyDS * 10;
	DEBUG_DATA_BUFFER[3] = sensorAttitudeData.gyDSFiltered * 10;
	DEBUG_DATA_BUFFER[4] = sensorAttitudeData.gzDS * 10;
	DEBUG_DATA_BUFFER[5] = sensorAttitudeData.gzDSFiltered * 10;
	DEBUG_DATA_BUFFER[6] = sensorAttitudeData.heading * 10;
	sendConfigData(DEBUG_DATA_BUFFER, 7, CMD_FC_DATA);
}

/*
 void debugLeverArm() {
 DEBUG_DATA_BUFFER[0] = fcStatusData.batteryVolt * 10;
 DEBUG_DATA_BUFFER[1] = deviceAttitudeData.ayG * 10000;
 DEBUG_DATA_BUFFER[2] = sensorAttitudeData.ayG * 10000;
 DEBUG_DATA_BUFFER[3] = sensorAttitudeData.gxDSFiltered * 10;
 sendConfigData(DEBUG_DATA_BUFFER, 4, CMD_FC_DATA);
 }
 */
/*
 void debugAttitude() {
 DEBUG_DATA_BUFFER[0] = sensorAttitudeData.pitch * 10;
 DEBUG_DATA_BUFFER[1] = sensorAttitudeData.pitchRate * 10;
 DEBUG_DATA_BUFFER[2] = sensorAttitudeData.roll * 10;
 DEBUG_DATA_BUFFER[3] = sensorAttitudeData.rollRate * 10;
 DEBUG_DATA_BUFFER[4] = sensorAttitudeData.heading * 10;
 DEBUG_DATA_BUFFER[5] = sensorAttitudeData.yawRate * 10;
 DEBUG_DATA_BUFFER[6] = sensorAltitudeData.altitudeSLFiltered * 10;
 DEBUG_DATA_BUFFER[7] = positionCordinateData.zPosition * 10;
 DEBUG_DATA_BUFFER[8] = positionCordinateData.zVelocity * 10;
 sendConfigData(DEBUG_DATA_BUFFER, 9, CMD_FC_DATA);
 }
 */
/*
 void debugVenturi() {
 DEBUG_DATA_BUFFER[0] = sensorAttitudeData.pitch;
 DEBUG_DATA_BUFFER[1] = rcData.RC_EFFECTIVE_DATA[RC_PITCH_CHANNEL_INDEX];
 DEBUG_DATA_BUFFER[2] = venturiEstimateData.venturiBias;
 DEBUG_DATA_BUFFER[3] = venturiEstimateData.lateralSpeed ;
 DEBUG_DATA_BUFFER[4] = positionCordinateData.zVelocity ;
 DEBUG_DATA_BUFFER[5] = controlData.tiltCompThDelta ;
 DEBUG_DATA_BUFFER[6] = controlData.altitudeControl;
 DEBUG_DATA_BUFFER[7] = positionCordinateData.zPosition;
 DEBUG_DATA_BUFFER[8] = sensorAltitudeData.altitudeSLScaled;

 sendConfigData(DEBUG_DATA_BUFFER, 9, CMD_FC_DATA);
 }
 */

/*
 float heightTemp = 0;
 float heightASL = 0;

 void debugGNSS() {

 if (heightTemp == 0 || gnssData.fixType == 0) {
 heightTemp = gnssData.height;
 }
 if (heightASL == 0 || gnssData.fixType == 0) {
 heightASL = gnssData.heightMSL;
 }
 DEBUG_DATA_BUFFER[0] = gnssData.velN * 100;
 DEBUG_DATA_BUFFER[1] = gnssData.velE * 100;
 DEBUG_DATA_BUFFER[2] = gnssData.velD * 1000;
 DEBUG_DATA_BUFFER[3] = positionCordinateData.xVelocity * 100;
 DEBUG_DATA_BUFFER[4] = positionCordinateData.yVelocity * 100;
 DEBUG_DATA_BUFFER[5] = positionCordinateData.zVelocity * 100;

 DEBUG_DATA_BUFFER[6] = (gnssData.height - heightTemp) * 100;
 DEBUG_DATA_BUFFER[7] = (gnssData.heightMSL - heightASL) * 100;
 DEBUG_DATA_BUFFER[8] = positionCordinateData.zPosition * 10;

 DEBUG_DATA_BUFFER[9] = gnssData.fixType;
 DEBUG_DATA_BUFFER[10] = gnssData.satCount;

 DEBUG_DATA_BUFFER[11] = gnssData.vAcc < 10 && gnssData.fixType > 0 ? gnssData.vAcc * 10 : 0;
 DEBUG_DATA_BUFFER[12] = gnssData.hAcc < 10 && gnssData.fixType > 0 ? gnssData.hAcc * 10 : 0;
 DEBUG_DATA_BUFFER[13] = gnssData.sAcc < 10 && gnssData.fixType > 0 ? gnssData.sAcc * 10 : 0;

 sendConfigData(DEBUG_DATA_BUFFER, 14, CMD_FC_DATA);

 }
 */
void debugPosition() {

	DEBUG_DATA_BUFFER[0] = fcStatusData.positionXRef * 10;
	DEBUG_DATA_BUFFER[1] = positionCordinateData.xPosition * 10;
	DEBUG_DATA_BUFFER[2] = positionCordinateData.xVelocity * 10;
	DEBUG_DATA_BUFFER[3] = controlData.positionXControl * 10;

	DEBUG_DATA_BUFFER[4] = fcStatusData.positionYRef * 10;
	DEBUG_DATA_BUFFER[5] = positionCordinateData.yPosition * 10;
	DEBUG_DATA_BUFFER[6] = positionCordinateData.yVelocity * 10;
	DEBUG_DATA_BUFFER[7] = controlData.positionYControl * 10;

	DEBUG_DATA_BUFFER[8] = positionCommandData.pitchCommand * 10;
	DEBUG_DATA_BUFFER[9] = positionCommandData.rollCommand * 10;
	DEBUG_DATA_BUFFER[10] = sensorAttitudeData.heading * 10;

	/*
	 DEBUG_DATA_BUFFER[3] = imuData.axEarthLinear * 1000;
	 DEBUG_DATA_BUFFER[4] = positionCordinateData.xAcceleration * 1000;
	 DEBUG_DATA_BUFFER[5] = positionCordinateData.xAccelerationBias * 1000;

	 DEBUG_DATA_BUFFER[6] = fcStatusData.positionYRef * 10;
	 DEBUG_DATA_BUFFER[7] = positionCordinateData.yPosition * 10;
	 DEBUG_DATA_BUFFER[8] = positionCordinateData.yVelocity * 10;
	 DEBUG_DATA_BUFFER[9] = imuData.ayEarthLinear * 1000;
	 DEBUG_DATA_BUFFER[10] = positionCordinateData.yAcceleration * 1000;
	 DEBUG_DATA_BUFFER[11] = positionCordinateData.yAccelerationBias * 1000;

	 DEBUG_DATA_BUFFER[12] = positionCordinateData.zVelocity * 10;
	 DEBUG_DATA_BUFFER[13] = imuData.azEarthLinear * 1000;
	 DEBUG_DATA_BUFFER[14] = positionCordinateData.zAccelerationBias * 1000;
	 DEBUG_DATA_BUFFER[15] = positionCordinateData.zAcceleration * 1000;
	 //	DEBUG_DATA_BUFFER[16] = positionCordinateData.zAccelerationBias * 1000;
	 */
	/*.
	 DEBUG_DATA_BUFFER[9] = sensorAttitudeData.heading * 10;
	 DEBUG_DATA_BUFFER[4] = controlData.positionXControl;
	 DEBUG_DATA_BUFFER[8] = controlData.positionYControl;
	 DEBUG_DATA_BUFFER[10] = positionCommandData.pitchCommand * 10;
	 DEBUG_DATA_BUFFER[11] = positionCommandData.rollCommand * 10;
	 */
	sendConfigData(DEBUG_DATA_BUFFER, 11, CMD_FC_DATA);
}

extern PID altPID;
extern PID altRatePID;
extern PID altAccPID;
extern float clampedAlt;
extern LOWPASSFILTER altMgrThrottleControlLPF;

void debugAltitude() {
	DEBUG_DATA_BUFFER[0] = sensorAltitudeData.altitudeSLFiltered * 1000;
	DEBUG_DATA_BUFFER[1] = positionCordinateData.zPositionRaw * 1000;
	DEBUG_DATA_BUFFER[2] = positionCordinateData.zPosition * 1000;
	DEBUG_DATA_BUFFER[3] = fcStatusData.altitudeSLRef * 1000;
	DEBUG_DATA_BUFFER[4] = (positionCordinateData.zPosition - fcStatusData.altitudeSLRef) * 1000;
	DEBUG_DATA_BUFFER[5] = positionCordinateData.zVelocity * 1000;

	DEBUG_DATA_BUFFER[6] = altPID.pid;
	DEBUG_DATA_BUFFER[7] = altRatePID.pid;
	DEBUG_DATA_BUFFER[8] = altAccPID.pid;
	DEBUG_DATA_BUFFER[9] = fcStatusData.currentThrottle;
	DEBUG_DATA_BUFFER[10] = controlData.throttleControl;
	DEBUG_DATA_BUFFER[11] = altMgrThrottleControlLPF.output;
	sendConfigData(DEBUG_DATA_BUFFER, 12, CMD_FC_DATA);
}

void debugAltitudeDevice() {
	DEBUG_DATA_BUFFER[0] = deviceAltitudeData.altitudeSLGround * 100;
	DEBUG_DATA_BUFFER[1] = deviceAltitudeData.altitudeSL * 100;
	DEBUG_DATA_BUFFER[2] = sensorAltitudeData.altitudeTerrain * 100;
	DEBUG_DATA_BUFFER[3] = sensorAltitudeData.altitudeTerrainQlty * 100;
	DEBUG_DATA_BUFFER[4] = (10.0f/sensorAltitudeData.altUpdateDt);
	sendConfigData(DEBUG_DATA_BUFFER, 5, CMD_FC_DATA);
}

void debugTask() {
	if (!fcStatusData.isDebugEnabled) {
		return;
	}
	float dt = 0.001f;
	(void) dt;
//	debugVenturi();
//	debugAttitude();
//	debugLeverArm();
//	debugNoise();
//  debugGNSS();
//  debugPosition();
//	debugAltitude();
	debugAltitudeDevice();
}
