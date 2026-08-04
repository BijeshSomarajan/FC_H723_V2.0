#ifndef SRC_FC_MANAGERS_BATTERY_BATTERYMANAGER_H_
#define SRC_FC_MANAGERS_BATTERY_BATTERYMANAGER_H_
#include <stdint.h>

#define BATTERY_MANAGER_TASK_FREQUENCY 10.0f
#define BATTERY_MANAGER_TASK_PERIOD    (1.0f/BATTERY_MANAGER_TASK_FREQUENCY)

/* ------------------------------------------------------------------ */
/* Battery sag compensation (voltage only)                            */
/* ------------------------------------------------------------------ */

#define BATTERY_VCOMP_MIN_GAIN  0.7f    /* clamp: cap how far comp can CUT a fresh / over-volt pack */
#define BATTERY_VCOMP_MAX_GAIN  1.3f    /* clamp: cap how far comp can BOOST a deeply sagged pack   */
#define BATTERY_VCOMP_TAU       1.0f    /* seconds: low-pass time constant for the comp voltage     */

#define BATTERY_TYPE_LIPO           0
#define BATTERY_TYPE_LIION          1

/* Alert thresholds as a factor of nominal — per chemistry, since LiPo
 * carries less usable range below nominal than Li-ion. Tune to taste. */
#define BATTERY_LOW_FACTOR_LIPO     0.95f   /* ~3.51 V/cell */
#define BATTERY_CRIT_FACTOR_LIPO    0.92f   /* ~3.40 V/cell */
#define BATTERY_LOW_FACTOR_LIION    0.90f   /* ~3.24 V/cell */
#define BATTERY_CRIT_FACTOR_LIION   0.85f   /* ~3.06 V/cell */

#define BATTERY_ALERT_NONE          0
#define BATTERY_ALERT_LOW           1
#define BATTERY_ALERT_CRITICAL      2

void resetBatteryManager(void);
uint8_t initBatteryManager(void);

#endif /* SRC_FC_MANAGERS_BATTERY_BATTERYMANAGER_H_ */
