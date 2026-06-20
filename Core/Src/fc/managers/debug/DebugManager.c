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

int32_t DEBUG_DATA_BUFFER[16];
extern LOWPASSFILTER thControlRefLPF;
extern uint8_t altControlAccEnabled;
extern POSITION_EKF positionEkf;
void debugTask(void);

uint8_t initDebugManager(void) {
	schedulerAddTask(debugTask, DEBUG_TASK_FREQUENCY, DEBUG_TASK_PRIORITY);
	return 1;
}

char buf[256];
extern float testGNSSRV, testGNSSRP;
void debugString() {
	sprintf(buf, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n", positionCordinateData.zVelocity, -gnssData.velD, positionCordinateData.zPosition, positionCordinateData.zPositionRawSL, (gnssData.heightMSL - fcStatusData.positionZHome), gnssData.hAcc, gnssData.vAcc, gnssData.sAcc,
			testGNSSRV, testGNSSRP, fcStatusData.isNavigationDataReliable);
	logString(buf);
}

extern float testMotionScale;
extern float testBaroR;
extern float testVenturiR;
extern float testVenturiBias;
extern float testGnssRp;
extern float testTerrainR;

void debugGraph() {
	DEBUG_DATA_BUFFER[0] = positionCordinateData.zPositionRawTerrain * 100;
	DEBUG_DATA_BUFFER[1] = sensorAltitudeData.altitudeTerrainFiltered * 100;
	DEBUG_DATA_BUFFER[2] = positionCordinateData.zPositionRawSL * 100;
	DEBUG_DATA_BUFFER[3] = positionCordinateData.zPosition * 100;
	DEBUG_DATA_BUFFER[4] = positionCordinateData.zVelocity * 100;
	DEBUG_DATA_BUFFER[5] = fcStatusData.isTerrainAltModeActive * 100;
	DEBUG_DATA_BUFFER[6] = testVenturiBias * 100;

	sendConfigData(DEBUG_DATA_BUFFER, 9, CMD_FC_DATA);
}

void debugTask() {
	if (!fcStatusData.isDebugEnabled) {
		return;
	}
	float dt = 0.001f;
	(void) dt;
	//debugString();
	debugGraph();
}
