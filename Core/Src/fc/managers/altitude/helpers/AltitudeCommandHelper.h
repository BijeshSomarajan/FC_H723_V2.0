#ifndef ALT_COMMAND_HELPER_H
#define ALT_COMMAND_HELPER_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// Velocity limits (exceeding them pauses the command in SETTLING until the drone slows down)
// ---------------------------------------------------------------------------
// Low limits: apply when the remaining altitude error is within ALT_COMMAND_FAR_DISTANCE
#define ALT_COMMAND_MAX_CLIMB_VELOCITY             0.5f   // m/s
#define ALT_COMMAND_MAX_DESCENT_VELOCITY           0.5f   // m/s

// Far limits: beyond FAR_DISTANCE the limit grows with the remaining error, up to the FAR caps
#define ALT_COMMAND_FAR_DISTANCE                   3.0f   // m, error at/below which the low limits apply
#define ALT_COMMAND_FAR_VELOCITY_GAIN              0.2f   // 1/s: extra limit (m/s) per metre of error beyond FAR_DISTANCE
#define ALT_COMMAND_FAR_MAX_CLIMB_VELOCITY         0.75f   // m/s cap (reached at 4.0 m of error)
#define ALT_COMMAND_FAR_MAX_DESCENT_VELOCITY       0.75f   // m/s cap (reached at 5.0 m of error)

// Velocity below which a paused (SETTLING) command resumes adjusting
#define ALT_COMMAND_RESUME_VELOCITY                0.1f  // m/s

// ---------------------------------------------------------------------------
// Target tolerance and base throttle rate
// ---------------------------------------------------------------------------
#define ALT_COMMAND_ALTITUDE_TOLERANCE              0.10f  // m, command completes within this of the target
#define ALT_COMMAND_POST_LIFTOFF_BASE_THROTTLE_RATE 60.0f  // throttle units/s, max rate (used at/above RATE_FULL_ERROR; fixed for landing)
#define ALT_COMMAND_MIN_BASE_THROTTLE_RATE          (ALT_COMMAND_POST_LIFTOFF_BASE_THROTTLE_RATE * 0.25f) // throttle units/s, fine correction near target
#define ALT_COMMAND_RATE_FULL_ERROR                 0.75f  // m of error at/above which the max rate is used

// ---------------------------------------------------------------------------
// Phase throttle limits: max throttle change in one ADJUSTING phase before pausing in SETTLING
// ---------------------------------------------------------------------------
#define ALT_COMMAND_MAX_PHASE_THROTTLE_DELTA            100.0f  // throttle units, normal altitude commands
#define ALT_COMMAND_TAKEOFF_MAX_PHASE_THROTTLE_DELTA    200.0f  // throttle units, takeoff
#define ALT_COMMAND_THROTTLE_ZERO_TOLERANCE             0.01f   // throttle units, landing completes at/below this
// ---------------------------------------------------------------------------
// Takeoff boost: extra throttle rate right after lift-off, fading out with altitude gained
// ---------------------------------------------------------------------------
#define ALT_COMMAND_TAKEOFF_BOOST_FACTOR           1.5f  // peak extra rate at the ground, as a multiple of BASE_THROTTLE_RATE
#define ALT_COMMAND_TAKEOFF_BOOST_DECAY_FRACTION   0.35f  // fade-out distance as a fraction of the takeoff altitude delta
#define ALT_COMMAND_TAKEOFF_BOOST_MIN_DECAY_DIST   0.15f  // m, lower clamp on the fade-out distance (short takeoffs)
#define ALT_COMMAND_TAKEOFF_BOOST_MAX_DECAY_DIST   0.80f  // m, upper clamp on the fade-out distance (long takeoffs)

// ---------------------------------------------------------------------------
// Linear throttle ramps below lift-off throttle
// ---------------------------------------------------------------------------
#define ALT_COMMAND_TAKEOFF_RAMP_TIME              0.7f   // s, throttle 0 -> lift-off throttle
#define ALT_COMMAND_LANDING_RAMP_TIME              1.0f   // s, lift-off throttle -> 0 (separate so it can be tuned independently)

#define ALT_COMMAND_MAX_PHASE_DISTANCE   2.0f   // m, max travel in one ADJUSTING phase before pausing for the PID
#define ALT_COMMAND_SETTLE_TIMEOUT       10.0f   // s, max time in SETTLING before resuming regardless of velocity

typedef enum {
	ALT_COMMAND_STATE_IDLE = 0, ALT_COMMAND_STATE_ADJUSTING, ALT_COMMAND_STATE_SETTLING
} AltCommandState;

typedef enum {
	ALT_COMMAND_TYPE_NORMAL = 0, ALT_COMMAND_TYPE_LANDING, ALT_COMMAND_TYPE_TAKEOFF
} AltCommandType;

extern AltCommandState altCommandState;

void resetAltCommandStates(void);
void startAltCommand(float currentAltitude, float targetAltitude, AltCommandType type);
void abortAltCommand(void);
void manageAltCommand(float dt);
uint8_t isAltCommandActive(void);
uint8_t isAltCommandMode(void);
uint8_t isAltCommandComplete(void);
#endif /* ALT_COMMAND_HELPER_H */

