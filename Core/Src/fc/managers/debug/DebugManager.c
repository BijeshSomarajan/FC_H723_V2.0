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

void debugOSD() {
	DEBUG_DATA_BUFFER[0] = fcStatusData.canStart | fcStatusData.canStabilize << 8 | fcStatusData.canFly << 16 | fcStatusData.hasCrashed << 24;
	DEBUG_DATA_BUFFER[1] = fcStatusData.isTxOn | fcStatusData.isPositionDataReliable << 8 | fcStatusData.isPositionHoldModeActive << 16 | fcStatusData.isRTHModeActive << 24;
	DEBUG_DATA_BUFFER[2] = fcStatusData.throttleControlPercent * 100;

	DEBUG_DATA_BUFFER[3] = sensorAttitudeData.pitch * 10;
	DEBUG_DATA_BUFFER[4] = sensorAttitudeData.roll * 10;
	DEBUG_DATA_BUFFER[5] = sensorAttitudeData.heading * 10;

	DEBUG_DATA_BUFFER[6] = positionCordinateData.xPosition * 10;
	DEBUG_DATA_BUFFER[7] = positionCordinateData.xVelocity * 10;

	DEBUG_DATA_BUFFER[8] = positionCordinateData.yPosition * 10;
	DEBUG_DATA_BUFFER[9] = positionCordinateData.yVelocity * 10;

	DEBUG_DATA_BUFFER[10] = positionCordinateData.zPosition;
	DEBUG_DATA_BUFFER[11] = positionCordinateData.zVelocity;

	DEBUG_DATA_BUFFER[12] = gnssData.satCount;
	DEBUG_DATA_BUFFER[13] = gnssData.fixStatus;

	DEBUG_DATA_BUFFER[14] = gnssData.latitude * 1000000;
	DEBUG_DATA_BUFFER[15] = gnssData.longitude * 1000000;

	sendConfigData(DEBUG_DATA_BUFFER, 16, CMD_FC_DATA);
}

void debugNoise() {
	DEBUG_DATA_BUFFER[0] = sensorAttitudeData.gxDS * 10;
	DEBUG_DATA_BUFFER[1] = sensorAttitudeData.gxDSFiltered * 10;
	DEBUG_DATA_BUFFER[2] = sensorAttitudeData.gyDS * 10;
	DEBUG_DATA_BUFFER[3] = sensorAttitudeData.gyDSFiltered * 10;
	DEBUG_DATA_BUFFER[4] = sensorAttitudeData.gzDS * 10;
	DEBUG_DATA_BUFFER[5] = sensorAttitudeData.gzDSFiltered * 10;
	sendConfigData(DEBUG_DATA_BUFFER, 6, CMD_FC_DATA);
}

void debugPosition() {
	DEBUG_DATA_BUFFER[0] = fcStatusData.positionXRef * 10;
	DEBUG_DATA_BUFFER[1] = positionCordinateData.xPosition * 10;
	DEBUG_DATA_BUFFER[2] = positionCordinateData.xVelocity * 10;
	DEBUG_DATA_BUFFER[3] = imuData.axEarthLinear * 1000;
	DEBUG_DATA_BUFFER[4] = positionCordinateData.xAcceleration * 1000;
	DEBUG_DATA_BUFFER[5] = positionCordinateData.xAccelerationBias * 1000;

	DEBUG_DATA_BUFFER[6] = fcStatusData.positionYRef * 10;
	DEBUG_DATA_BUFFER[7] = positionCordinateData.yPosition * 10;
	DEBUG_DATA_BUFFER[8] = positionCordinateData.yVelocity * 10;
	DEBUG_DATA_BUFFER[9] = imuData.ayEarthLinear * 1000;
	DEBUG_DATA_BUFFER[10] = positionCordinateData.yAcceleration * 1000;
	DEBUG_DATA_BUFFER[11] = positionCordinateData.yAccelerationBias * 1000;

	DEBUG_DATA_BUFFER[12] = positionCordinateData.zPosition * 10;
	DEBUG_DATA_BUFFER[13] = positionCordinateData.zVelocity * 10;
	DEBUG_DATA_BUFFER[14] = imuData.azEarthLinear * 1000;
	DEBUG_DATA_BUFFER[15] = positionCordinateData.zAcceleration * 1000;
//	DEBUG_DATA_BUFFER[16] = positionCordinateData.zAccelerationBias * 1000;

	/*.
	 DEBUG_DATA_BUFFER[9] = sensorAttitudeData.heading * 10;
	 DEBUG_DATA_BUFFER[4] = controlData.positionXControl;
	 DEBUG_DATA_BUFFER[8] = controlData.positionYControl;
	 DEBUG_DATA_BUFFER[10] = positionCommandData.pitchCommand * 10;
	 DEBUG_DATA_BUFFER[11] = positionCommandData.rollCommand * 10;
	 */
	sendConfigData(DEBUG_DATA_BUFFER, 16, CMD_FC_DATA);
}

void debugTask() {
	if (!fcStatusData.isDebugEnabled) {
		return;
	}
	float dt = 0.001f;
	(void) dt;
	//debugNoise();
	debugPosition();
}
