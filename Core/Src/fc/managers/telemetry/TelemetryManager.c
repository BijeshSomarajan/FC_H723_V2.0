#include "TelemetryManager.h"

#include <sys/_stdint.h>
#include "../../FCConfig.h"
#include "../../imu/IMU.h"
#include "../../logger/Logger.h"
#include "../../status/FCStatus.h"
#include "../../timers/DelayTimer.h"
#include "../../timers/Scheduler.h"
#include "../config/ConfigHelper.h"
#include "../../sensors/rc/RCTelemetry.h"
#include "../../sensors/altitude/AltitudeSensor.h"
#include "../../sensors/attitude/AttitudeSensor.h"
#include "../../managers/position/PositionManager.h" // Cleaned up duplicate include
#include "../../sensors/position/GNSS.h"
#include "../../sensors/battery/BatterySensor.h"

char FC_STATUS_BUF[8];
TelemetryStep currentTelemetryStep = TELEMETRY_STEP_ALTITUDE;

void sendFCStatus() {
    // Off, Start, Stab, Fly
    if (!fcStatusData.canStart) {
        FC_STATUS_BUF[0] = 'O';
    } else if (fcStatusData.canStart) {
        FC_STATUS_BUF[0] = 'I';
    } else if (fcStatusData.canStabilize) {
        FC_STATUS_BUF[0] = 'S';
    } else if (fcStatusData.canFly) {
        FC_STATUS_BUF[0] = 'F';
    }
    // Loiter, Pos Hold, RTH
    FC_STATUS_BUF[1] = '|';
    if (fcStatusData.isNavModeActive) {
        FC_STATUS_BUF[2] = 'N';
    } else if (fcStatusData.isNavRTHModeActive) {
        FC_STATUS_BUF[2] = 'R';
    } else {
        FC_STATUS_BUF[2] = 'L';
    }
    // Terrain/Baro
    FC_STATUS_BUF[3] = '|';
    if (fcStatusData.isTerrainAltModeActive) {
        FC_STATUS_BUF[4] = 'T';
    } else {
        FC_STATUS_BUF[4] = 'B';
    }
    // Landing/Flying
    FC_STATUS_BUF[5] = '|';
    if (fcStatusData.isLandingModeActive) {
        FC_STATUS_BUF[6] = 'L';
    } else {
        FC_STATUS_BUF[6] = 'F';
    }
    FC_STATUS_BUF[7] = '\0';
    sendFlightModeTelemetry(FC_STATUS_BUF);
}

/**
 * @brief Main task executed by scheduler.
 * Sends exactly ONE type of telemetry per execution tick.
 */
void telemetryUpdateTask() {
    switch (currentTelemetryStep) {
        case TELEMETRY_STEP_ALTITUDE:
            sendAltitudeTelemetry(sensorAltitudeData.altitudeSL, positionCordinateData.zVelocity);
            break;

        case TELEMETRY_STEP_ATTITUDE:
            sendAttitudeTelemetry(sensorAttitudeData.pitch, sensorAttitudeData.roll, sensorAttitudeData.heading);
            break;

        case TELEMETRY_STEP_BATTERY:
            // Peer note: You are passing batteryData.voltage twice here.
            // If intentional (e.g., placeholder for current/capacity), leave it.
            sendBatteryTelemetry(batteryData.voltage, batteryData.voltage, 0, 0);
            break;

        case TELEMETRY_STEP_GNSS:
            sendGNSSTelemetry(gnssData.latitude, gnssData.longitude, gnssData.velN,
                              sensorAttitudeData.heading, positionCordinateData.zPosition,
                              gnssData.satCount);
            break;

        case TELEMETRY_STEP_FC_STATUS:
            sendFCStatus();
            break;

        default:
            currentTelemetryStep = TELEMETRY_STEP_ALTITUDE;
            return;
    }
    // Advance to next frame step, wrapping around smoothly
    currentTelemetryStep++;
    if (currentTelemetryStep >= TELEMETRY_STEP_COUNT) {
        currentTelemetryStep = TELEMETRY_STEP_ALTITUDE;
    }
}

uint8_t initTelemetryManager() {
    schedulerAddTask(telemetryUpdateTask, TELEMETRY_TASK_FREQUENCY, TELEMETRY_TASK_PRIORITY);
    fcStatusData.isOSDEnabled = 1;
    logString("[Telemetry Manager] >> Init >> Success\n");
    return 1;
}
