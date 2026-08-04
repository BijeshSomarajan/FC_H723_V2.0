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

static float vBattFiltered = 0.0f;
static uint8_t vBattFilterSeeded = 0;
static uint8_t batteryCriticalLatched = 0;

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
	float vMed = getAverageBatteryVoltage(batteryData.voltage);

	/* Seed the low-pass only from a plausible reading. At power-up the sensor
	 * may report ~0 before the ADC settles; seeding to that and letting the
	 * filter ramp up would drag vBattFiltered through the alert bands and
	 * latch critical. Wait for a valid reading, then jump straight to it. */
	if (!vBattFilterSeeded) {
		if (vMed > fcStatusData.batteryNomVolt * 0.5f) {
			vBattFiltered = vMed;
			vBattFilterSeeded = 1;
		} else {
			controlData.batteryDepletionGain = 1.0f;
			return;
		}
	}

	float alpha = dt / (BATTERY_VCOMP_TAU + dt);
	vBattFiltered += alpha * (vMed - vBattFiltered);

	float gain = 1.0f;
	if (vBattFiltered > fcStatusData.batteryNomVolt * 0.5f) {
		gain = constrainToRangeF(fcStatusData.batteryNomVolt / vBattFiltered,
		                         BATTERY_VCOMP_MIN_GAIN, BATTERY_VCOMP_MAX_GAIN);

		float lowFactor, critFactor;
		if (fcStatusData.batteryType == BATTERY_TYPE_LIION) {
			lowFactor = BATTERY_LOW_FACTOR_LIION;
			critFactor = BATTERY_CRIT_FACTOR_LIION;
		} else {
			lowFactor = BATTERY_LOW_FACTOR_LIPO;
			critFactor = BATTERY_CRIT_FACTOR_LIPO;
		}
		float vLow = fcStatusData.batteryNomVolt * lowFactor;
		float vCrit = fcStatusData.batteryNomVolt * critFactor;

		if (vBattFiltered <= vCrit) {
			batteryCriticalLatched = 1;
		}

		if (batteryCriticalLatched) {
			fcStatusData.batteryAlertState = BATTERY_ALERT_CRITICAL;
		} else if (vBattFiltered <= vLow) {
			fcStatusData.batteryAlertState = BATTERY_ALERT_LOW;
		} else {
			fcStatusData.batteryAlertState = BATTERY_ALERT_NONE;
		}
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
	fcStatusData.batteryAlertState = BATTERY_ALERT_NONE;
	batteryVoltageSeeded = 0;
	vBattFilterSeeded = 0;
	batteryCriticalLatched = 0;
}
