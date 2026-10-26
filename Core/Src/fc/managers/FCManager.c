#include "FCManager.h"

#include "../FCConfig.h"
#include "../logger/Logger.h"
#include "../memory/Memory.h"
#include "../status/FCStatus.h"
#include "../timers/DelayTimer.h"
#include "../timers/DeltaTimer.h"
#include "../timers/Scheduler.h"
#include "altitude/AltitudeManager.h"
#include "attitude/AttitudeManager.h"
#include "battery/BatteryManager.h"
#include "config/ConfigManager.h"
#include "debug/DebugManager.h"
#include "indicator/IndicatorManager.h"
#include "mission/MissionManager.h"
#include "motor/MotorManager.h"
#include "position/PositionManager.h"
#include "rc/RCManager.h"
#include "telemetry/TelemetryManager.h"

void dispatchManagers() {
	if (fcStatusData.hasInitialized) {
		doAttitudeManagement();
		doAltitudeManagement();
		doPositionManagement();
		doConfigManagement();
		doRCManagement();
	}
}

void runFC() {
	resetAllDeltaTimes();
	while (1) {
		dispatchManagers();
		dispatchScheduler();
	}
}

uint8_t initTimers() {
	initDelayTimer();
	initDeltaTimer();
	return 1;
}

uint8_t initManagers() {
	logString("[FC Manager] initManagers >> Start\n");
	uint8_t status = 1; // Start assuming success

	if (status) {
		status = initIndicatorManager();
		if (status) {
			logString("[FC Manager] >> initManagers >> initIndicatorManager > Success\n");
		} else {
			logString("[FC Manager] >> initManagers >> initIndicatorManager > Failed\n");
		}
	}

	if (status) {
		status = initConfigManager();
		if (status) {
			logString("[FC Manager] >> initManagers >> initConfigManager  > Success\n");
		} else {
			logString("[FC Manager] >> initManagers >> initConfigManager  > Failed\n");
		}
	}

	if (status) {
		status = initRCManager();
		if (status) {
			logString("[FC Manager] >> initManagers >> initRCManager > Success\n");
		} else {
			logString("[FC Manager] >> initManagers >> initRCManager > Failed\n");
		}
	}

	if (status) {
		status = initPositionManager();
		if (status) {
			logString("[FC Manager] >> initManagers >> initPositionManager > Success\n");
		} else {
			logString("[FC Manager] >> initManagers >> initPositionManager > Failed\n");
		}
	}

	if (status) {
		status = initMissionManager();
		if (status) {
			logString("[FC Manager] >> initManagers >> initMissionManager > Success\n");
		} else {
			logString("[FC Manager] >> initManagers >> initMissionManager > Failed\n");
		}
	}

	if (status) {
		status = initAttitudeManager();
		if (status) {
			logString("[FC Manager] >> initManagers >> initAttitudeManager > Success\n");
		} else {
			logString("[FC Manager] >> initManagers >> initAttitudeManager > Failed\n");
		}
	}

	if (status) {
		status = initAltitudeManager();
		if (status) {
			logString("[FC Manager] >> initManagers >> initAltitudeManager > Success\n");
		} else {
			logString("[FC Manager] >> initManagers >> initAltitudeManager > Failed\n");
		}
	}

	if (status) {
		status = initTelemetryManager();
		if (status) {
			logString("[FC Manager] >> initManagers >> initTelemetryManager > Success\n");
		} else {
			logString("[FC Manager] >> initManagers >> initTelemetryManager > Failed\n");
		}
	}

# if DEBUG_ENABLED == 1
	if (status) {
		status = initDebugManager();
		if (status) {
			logString("[FC Manager] >> initManagers >> initDebugManager > Success\n");
		} else {
			logString("[FC Manager] >> initManagers >> initDebugManager > Failed\n");
		}
	}
#endif

	if (status) {
		status = initMotorManager();
		if (status) {
			logString("[FC Manager] >> initManagers >> initOutputManager > Success\n");
		} else {
			logString("[FC Manager] >> initManagers >> initOutputManager > Failed\n");
		}
	}

	if (status) {
		status = initBatteryManager();
		if (status) {
			logString("[FC Manager] >> initManagers >> initBatteryManager > Success\n");
		} else {
			logString("[FC Manager] >> initManagers >> initBatteryManager > Failed\n");
		}
	}

	if (status) {
		logString("[FC Manager] >> initManagers > Success\n");
	} else {
		logString("[FC Manager] >> initManagers > Failed\n");
	}

	return status;
}

uint8_t initPeripherals() {
	uint8_t status = 1;

	status = initLogger();
	if (!status) {
		return 0;
	} else {
		logString("[FC Manager] >> initPeripherals >> Start\n");
		logString("[FC Manager] >> initPeripherals >> initLogger >> Success\n");
	}

	if (status) {
		status = initTimers();
		if (status) {
			logString("[FC Manager] >> initPeripherals >> initTimers > Success\n");
		} else {
			logString("[FC Manager] >> initPeripherals >> initManagers > Failed\n");
		}
	}

	return status;
}

void presetGPIOs() {
	delayMs(10);
}

uint8_t initFC() {
	uint8_t status = 1;
	presetGPIOs();
	initMemory();

	if (status) {
		status = initPeripherals();
		if (!status) {
			logString("[FC Manager] >> initFC >> initPeripherals >  Failed\n");
		} else {
			logString("[FC Manager] >> initFC >> initPeripherals >  Success\n");
		}
	}

	if (status) {
		status = initManagers();
		if (!status) {
			logString("[FC Manager] >> initFC >> initManagers > Failed\n");
			status = 0;
		} else {
			logString("[FC Manager] >> initFC >> initManagers > Success\n");
		}
	}

	if (status) {
		status = schedulerInit();
		if (!status) {
			logString("[FC Manager] >> initFC >> schedulerInit > Failed\n");
		} else {
			logString("[FC Manager] >> initFC >> schedulerInit > Success\n");
		}
	}

	if (!status) {
		initIndicatorManager(); // In case if this was not initialized
		logString("[FC Manager] >> initFC >> StartUp > Failed\n");
		while (1) {
			delayMs(150);
			indicateStartUpFailed();
		}
	} else {
		fcStatusData.hasInitialized = 1;
		indicateStartUpSuccess();
		logString("[FC Manager] >> initFC >> StartUp > Success , Running\n");
		runFC();
	}

	return 1;
}

