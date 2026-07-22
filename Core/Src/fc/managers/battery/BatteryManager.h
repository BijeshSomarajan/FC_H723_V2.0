#ifndef SRC_FC_MANAGERS_BATTERY_BATTERYMANAGER_H_
#define SRC_FC_MANAGERS_BATTERY_BATTERYMANAGER_H_
#include <stdint.h>

#define BATTERY_MANAGER_TASK_FREQUENCY 10.0f
#define BATTERY_MANAGER_TASK_PERIOD    (1.0f/BATTERY_MANAGER_TASK_FREQUENCY)

/* ------------------------------------------------------------------ */
/* Battery sag compensation (voltage only)                            */
/* ------------------------------------------------------------------ */

/* --- Battery sag compensation (voltage only) --- */
#define BATTERY_SAG_STRENGTH  1.0f    /* 0..~1: 1.0 ~ fully restore lost thrust  */
#define BATTERY_SAG_MIN_GAIN  1.0f    /* clamp so a low pack can't over-boost     */
#define BATTERY_SAG_MAX_GAIN  1.2f    /* clamp so a low pack can't over-boost     */

void resetBatteryManager(void);
uint8_t initBatteryManager(void);

#endif /* SRC_FC_MANAGERS_BATTERY_BATTERYMANAGER_H_ */
