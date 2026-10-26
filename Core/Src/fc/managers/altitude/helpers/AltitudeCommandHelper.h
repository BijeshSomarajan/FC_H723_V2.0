#ifndef ALT_COMMAND_HELPER_H
#define ALT_COMMAND_HELPER_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// Takeoff completion
// ---------------------------------------------------------------------------
#define ALT_COMMAND_TARGET_ALTITUDE_TOLERANCE              0.1f   // m, takeoff completes within this of the target

// ---------------------------------------------------------------------------
// Landing: PID-stepped descent (altitudeRef moves down in steps, PID holds each step)
// ---------------------------------------------------------------------------
#define ALT_COMMAND_DISTANCE_STEP_MAX    2.0f
#define ALT_COMMAND_DISTANCE_STEP_MIN    0.25f
#define ALT_COMMAND_DISTANCE_STEP_GAIN   0.3f
#define ALT_COMMAND_STEP_TIMEOUT         10.0f
// ---------------------------------------------------------------------------
// Base throttle reduction above lift-off throttle (landing)
// ---------------------------------------------------------------------------
#define ALT_COMMAND_POST_LIFTOFF_BASE_THROTTLE_RATE        30.0f  // throttle units/s, max rate (fixed for landing)
#define ALT_COMMAND_MIN_BASE_THROTTLE_RATE                 (ALT_COMMAND_POST_LIFTOFF_BASE_THROTTLE_RATE * 0.25f) // throttle units/s, fine correction near target
#define ALT_COMMAND_RATE_FULL_ERROR                        0.75f  // m of error at/above which the max rate is used

// ---------------------------------------------------------------------------
// Linear throttle ramps below lift-off throttle
// ---------------------------------------------------------------------------
#define ALT_COMMAND_PRE_LIFTOFF_THROTTLE_INCREASE_RAMP_TIME  0.5f   // s, throttle 0 -> lift-off throttle
#define ALT_COMMAND_PRE_LIFTOFF_THROTTLE_DECREASE_RAMP_TIME  0.3f   // s, lift-off throttle -> 0
#define ALT_COMMAND_THROTTLE_ZERO_TOLERANCE                  0.01f  // throttle units, landing completes at/below this

typedef enum {
	ALT_COMMAND_TYPE_NONE = 0, ALT_COMMAND_TYPE_NORMAL, ALT_COMMAND_TYPE_LANDING, ALT_COMMAND_TYPE_TAKEOFF
} AltCommandType;

void resetAltCommandStates(void);
void startAltCommand(float currentAltitude, float targetAltitude, AltCommandType type);
void abortAltCommand(void);
void manageAltCommand(float dt);

uint8_t isAltCommandMode(void);
uint8_t isAltCommandComplete(void);
uint8_t isAltCommandThrottleUpdateRequired(void);

void setTargetAltForAltChangeCommand(float alt);
float getTargetAltForAltChangeCommand(void);

#endif /* ALT_COMMAND_HELPER_H */
