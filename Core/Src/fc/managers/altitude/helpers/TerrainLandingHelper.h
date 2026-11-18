#ifndef TERRAIN_LANDING_HELPER_H
#define TERRAIN_LANDING_HELPER_H

#include <stdint.h>
#include "../AltitudeCommon.h"

// ---------------------------------------------------------------------------
// Terrain-sensor assisted landing (OPTIONAL; lidar, sonar, etc.)
// Heights are AGL at the landing gear: altitudeTerrainFiltered - MOUNT_HEIGHT.
//
// 1. Baro stepped descent until terrain AGL <= ENGAGE_HEIGHT
// 2. altitudeRef ramps down, sink rate proportional to AGL
// 3. Touchdown -> existing throttle ramp-down
// No usable sensor = inert, landing stays purely baro based.
// ---------------------------------------------------------------------------
#define ALT_COMMAND_TERRAIN_MOUNT_HEIGHT             0.05f  // m, sensor-to-ground distance when sitting on the ground (measure it)
#define ALT_COMMAND_TERRAIN_ENGAGE_HEIGHT            3.0f   // m, hand over from baro steps to terrain descent

#define ALT_COMMAND_TERRAIN_SINK_RATE_MAX            0.5f   // m/s
#define ALT_COMMAND_TERRAIN_SINK_RATE_MIN            0.1f  // m/s
#define ALT_COMMAND_TERRAIN_SINK_RATE_GAIN           0.25f   // 1/s, sink rate = gain * AGL, clamped
#define ALT_COMMAND_TERRAIN_REF_MAX_LEAD             0.3f   // m, altitudeRef never runs further below actual z

#define ALT_COMMAND_TERRAIN_LOW_ZONE_HEIGHT          0.30f  // m, signal lost below this = ground is in the blind zone
#define ALT_COMMAND_TERRAIN_TOUCHDOWN_HEIGHT         0.10f  // m, valid reading at/below this = on the ground
#define ALT_COMMAND_TERRAIN_TOUCHDOWN_CONFIRM_TIME   0.10f  // s, touchdown condition must hold this long

#define ALT_COMMAND_TERRAIN_INVALID_TOLERANCE        0.3f   // s, dropout above the low zone before falling back to baro
#define ALT_COMMAND_TERRAIN_FINAL_TIMEOUT            15.0f  // s, max time in terrain phase before falling back to baro

void resetTerrainLandingStates(void);
uint8_t isTerrainLandingTouchdown(void);
uint8_t handleTerrainAssistedLanding(float dt);

#endif /* TERRAIN_LANDING_HELPER_H */
