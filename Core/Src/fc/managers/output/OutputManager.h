#ifndef SRC_FC_MANAGERS_OUTPUT_OUTPUTMANAGER_H_
#define SRC_FC_MANAGERS_OUTPUT_OUTPUTMANAGER_H_

#include <sys/_stdint.h>
#include "../attitude/AttitudeManager.h"
#include "../../io/pwm/PWM.h"

typedef struct _PWM_DATA PWM_DATA;
struct _PWM_DATA {
	int32_t PWM_VALUES[PWM_CHANNEL_COUNT];
	float updateDt;
};

extern PWM_DATA pwmData;

#define OUTPUT_CONTROL_FREQUENCY ATTITUDE_SENSOR_AGT_READ_FREQUENCY
#define OUTPUT_PWM_PRESET RC_CHANNEL_MIN_VALUE

uint8_t initOutputManager(void);
void stopOutputs(void);

#endif
