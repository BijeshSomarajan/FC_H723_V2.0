#include "BatteryManager.h"
#include "../../sensors/battery/BatterySensor.h"
#include "../../logger/Logger.h"
#include "../../FCConfig.h"
#include "../../timers/DeltaTimer.h"
#include "../../timers/Scheduler.h"

void batteryManegementTask(){
	readBatterySensor();
}

uint8_t initBatteryManager(void) {
	uint8_t status = 0;
	status = initBatterySensor();
	if (!status) {
		logString("[BatteryManager] Init Failed\n");
		return 0;
	} else {
		logString("[BatteryManager] Init Success\n");
	}
	schedulerAddTask(batteryManegementTask, BATTERY_MANAGER_TASK_FREQUENCY, BATTERY_MANAGER_TASK_PRIORITY);
	return 1;
}

void resetBatteryManager(void) {
}

