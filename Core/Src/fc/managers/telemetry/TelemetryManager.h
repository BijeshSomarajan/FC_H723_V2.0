#ifndef SRC_FC_MANAGERS_TELEMETRY_TELEMETRYMANAGER_H_
#define SRC_FC_MANAGERS_TELEMETRY_TELEMETRYMANAGER_H_
#include <stdint.h>

// Explicit enumeration for round-robin states
typedef enum {
    TELEMETRY_STEP_ALTITUDE = 0,
    TELEMETRY_STEP_ATTITUDE,
    TELEMETRY_STEP_BATTERY,
    TELEMETRY_STEP_GNSS,
    TELEMETRY_STEP_FC_STATUS,
    TELEMETRY_STEP_COUNT // Keeps track of total states automatically (5)
} TelemetryStep;

#define TELEMETRY_TASK_FREQUENCY 60.0f
uint8_t initTelemetryManager(void);

#endif /* SRC_FC_MANAGERS_TELEMETRY_TELEMETRYMANAGER_H_ */
