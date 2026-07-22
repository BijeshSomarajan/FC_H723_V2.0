#include "BatteryManager.h"
#include "../../sensors/battery/BatterySensor.h"
#include "../../logger/Logger.h"
#include "../../FCConfig.h"
#include "../../timers/DeltaTimer.h"
#include "../../timers/Scheduler.h"
#include "../../status/FCStatus.h"
#include "../../util/MathUtil.h"
#include "../../control/ControlData.h"
float rawBatteryVoltages[3];
uint8_t batteryVoltageSeeded = 0;
float getAverageBatteryVoltage(float vRaw) {
	if (!batteryVoltageSeeded) {
		rawBatteryVoltages[0] = rawBatteryVoltages[1] = rawBatteryVoltages[2] = vRaw;
		batteryVoltageSeeded = 1;
	}
	rawBatteryVoltages[2] = rawBatteryVoltages[1];
	rawBatteryVoltages[1] = rawBatteryVoltages[0];
	rawBatteryVoltages[0] = vRaw;
	float a = rawBatteryVoltages[0], b = rawBatteryVoltages[1], c = rawBatteryVoltages[2], t;
	if (a > b) {
		t = a;
		a = b;
		b = t;
	}
	if (b > c) {
		t = b;
		b = c;
		c = t;
	}
	if (a > b) {
		t = a;
		a = b;
		b = t;
	}
	return b; /* middle value */
}

void updateBatteryUsage(float dt) {
	float v = getAverageBatteryVoltage(batteryData.voltage);
	float gain = BATTERY_SAG_MIN_GAIN;
	/* below half of full charge -> sense line unplugged / glitch, don't comp */
	if (v > fcStatusData.maxBatteryVolt * 0.5f && fcStatusData.canFly) {
		gain = 1.0f + BATTERY_SAG_STRENGTH * (fcStatusData.maxBatteryVolt - v) / fcStatusData.maxBatteryVolt;
		if (gain < 1.0f) {
			gain = 1.0f;
		}
		gain = constrainToRangeF(gain, BATTERY_SAG_MIN_GAIN, BATTERY_SAG_MAX_GAIN);
	}
	controlData.batteryDepletionGain = gain;
}

void batteryManegementTask() {
	if (readBatterySensor()) {
		updateBatteryUsage(BATTERY_MANAGER_TASK_PERIOD);
	}
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
	resetBatteryManager();
	schedulerAddTask(batteryManegementTask, BATTERY_MANAGER_TASK_FREQUENCY, BATTERY_MANAGER_TASK_PRIORITY);
	return 1;
}

void resetBatteryManager(void) {
	controlData.batteryDepletionGain = 1.0f;
}
