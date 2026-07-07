#ifndef SRC_FC_SENSORS_BATTERY_BATTERYSENSOR_H_
#define SRC_FC_SENSORS_BATTERY_BATTERYSENSOR_H_
#include <stdint.h>

typedef struct {
    float voltage;   // Volts
    float current;       // Amperes
} BatteryData;

extern BatteryData batteryData;

uint8_t initBatterySensor(void);
uint8_t readBatterySensor(void);

#endif /* SRC_FC_SENSORS_BATTERY_BATTERYSENSOR_H_ */
