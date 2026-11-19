#ifndef TERRAIN_ASISSTANCE_HELPER_H
#define TERRAIN_ASISSTANCE_HELPER_H

#include <stdint.h>
#include "../AltitudeCommon.h"

// ---------------------------------------------------------------------------
// Terrain-sensor assistance for takeoff and landing (OPTIONAL; lidar, sonar, etc.)
// Heights are AGL at the landing gear: altitudeTerrainFiltered - MOUNT_HEIGHT.
// No usable sensor = inert, takeoff and landing stay purely baro based.
//
// Lift-off height (the configured takeoff height) is the boundary between baro
// and terrain control:
//   Takeoff: climb under terrain control until AGL reaches the lift-off height.
//   Landing: 1. baro stepped descent until terrain AGL <= lift-off height
//            2. altitudeRef ramps down, sink rate proportional to AGL
//            3. touchdown -> existing throttle ramp-down
//
// Usage pattern (both flows): call the handle...() function once per tick,
// then read the state through the is...()/get...() queries (pure reads).
// ---------------------------------------------------------------------------
#define ALT_COMMAND_TERRAIN_MOUNT_HEIGHT             0.05f  // m, sensor-to-ground distance when sitting on the ground (measure it)

#define ALT_COMMAND_TERRAIN_SINK_RATE_MAX            0.5f   // m/s
#define ALT_COMMAND_TERRAIN_SINK_RATE_MIN            0.2f   // m/s
#define ALT_COMMAND_TERRAIN_SINK_RATE_GAIN           0.25f  // 1/s, sink rate = gain * AGL, clamped
#define ALT_COMMAND_TERRAIN_REF_MAX_LEAD             0.3f   // m, altitudeRef never runs further below actual z

#define ALT_COMMAND_TERRAIN_LOW_ZONE_HEIGHT          0.30f  // m, signal lost below this = ground is in the blind zone
#define ALT_COMMAND_TERRAIN_TOUCHDOWN_HEIGHT         0.10f  // m, valid reading at/below this = on the ground
#define ALT_COMMAND_TERRAIN_TOUCHDOWN_CONFIRM_TIME   0.10f  // s, touchdown condition must hold this long

#define ALT_COMMAND_TERRAIN_INVALID_TOLERANCE        0.5f   // s, dropout above the low zone before falling back to baro
#define ALT_COMMAND_TERRAIN_FINAL_TIMEOUT            30.0f  // s, max time in terrain phase before falling back to baro

#define ALT_COMMAND_TERRAIN_TAKEOFF_TOLERANCE        0.1f   // m, takeoff completes within this of the lift-off height

/** Call once at init with the configured takeoff (lift-off) height in metres. */
void initTerrainAssistedHelper(float liftOffHeight);
void resetTerrainLandingStates(void);

/** Sensor exists and its data is reliable. */
uint8_t isTerrainAssistanceEnabled(void);

// --- Landing ---------------------------------------------------------------
/**
 * Call every tick while a LANDING alt command runs. Advances the terrain state
 * machine; while active it owns altitudeRef. Inert without a usable sensor.
 */
void handleTerrainAssistedLanding(float dt);
uint8_t isTerrainLandingActive(void);      // terrain descent currently owns altitudeRef
uint8_t isTerrainLandingComplete(void);    // touchdown latched: caller ramps throttle down

// --- Takeoff ---------------------------------------------------------------
/** Call once per tick while a TAKEOFF alt command runs. */
void handleTerrainAssistedTakeoff(void);
uint8_t isTerrainTakeoffActive(void);      // terrain has a usable reading and controls this tick
uint8_t isTerrainAltitudeReached(void);    // AGL within tolerance of the lift-off height
float getTerrainTakeoffRef(void);          // altitudeRef for the climb: z + remaining

#endif /* TERRAIN_ASISSTANCE_HELPER_H */
