#ifndef ALT_COMMAND_HELPER_H
#define ALT_COMMAND_HELPER_H

#include <stdint.h>

#define ALT_COMMAND_MAX_CLIMB_VELOCITY             0.5f
#define ALT_COMMAND_MAX_DESCENT_VELOCITY           0.5f

#define ALT_COMMAND_RESUME_VELOCITY                0.05f

#define ALT_COMMAND_ALTITUDE_TOLERANCE             0.10f
#define ALT_COMMAND_BASE_THROTTLE_RATE             30.0f
#define ALT_COMMAND_MIN_BASE_THROTTLE_RATE         (ALT_COMMAND_BASE_THROTTLE_RATE * 0.1f) // fine correction near target
#define ALT_COMMAND_RATE_FULL_ERROR                1.2f  // m of error at/above which the max rate is used

#define ALT_COMMAND_MAX_PHASE_THROTTLE_DELTA       100.0f
#define ALT_COMMAND_LOW_THROTTLE_DECAY_RATE        2.0f
#define ALT_COMMAND_THROTTLE_ZERO_TOLERANCE        0.01f

typedef enum {
	ALT_COMMAND_STATE_IDLE = 0, ALT_COMMAND_STATE_ADJUSTING, ALT_COMMAND_STATE_SETTLING
} AltCommandState;

typedef enum {
	ALT_COMMAND_TYPE_NORMAL = 0, ALT_COMMAND_TYPE_LANDING, ALT_COMMAND_TYPE_TAKEOFF
} AltCommandType;

extern AltCommandState altCommandState;

void resetAltCommandStates(void);
void startAltCommand(float currentAltitude,float targetAltitude, AltCommandType type);
void abortAltCommand(void);
void manageAltCommand(float dt);
uint8_t isAltCommandActive(void);
uint8_t isAltCommandMode(void);
uint8_t isAltCommandComplete(void);
#endif /* ALT_COMMAND_HELPER_H */

