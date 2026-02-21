#include "OutputManager.h"

#include <sys/_stdint.h>

#include "../../control/ControlData.h"
#include "../../io/pwm/PWM.h"
#include "../../logger/Logger.h"
#include "../../memory/Memory.h"
#include "../../sensors/rc/RCSensor.h"
#include "../../status/FCStatus.h"
#include "../../timers/DeltaTimer.h"
#include "../../timers/GPTimer.h"
#include "../../util/MathUtil.h"
#include "../../FCConfig.h"

void setPWMChannelValue(uint8_t channel, int value);
void idlePWMs(void);
void outputControlTask(void);

PWM_DATA __ATTR_DTCM_BSS pwmData;
uint8_t outputControlInitStatus = 0;

uint8_t initOutputManager(void) {
	uint8_t status = 1;
	status = initPWM(PWM_MODE_ONESHOT);
	if (!status) {
		logString("[Output Manager] >> IO >> PWM > Failed\n");
	} else {
		logString("[Output Manager] >> IO >> PWM > Success\n");
	}
	if (status) {
		initGPTimer6(OUTPUT_CONTROL_FREQUENCY, outputControlTask, 5);
		startGPTimer6();
		outputControlInitStatus = status;
		logString("[Output Manager] >> Init > Success\n");
	} else {
		logString("[Output Manager] >> Init > Failed\n");
	}
	stopOutputs();
	return status;
}

__ATTR_ITCM_TEXT
void updatePWMValues() {
	for (uint8_t indx = 0; indx < PWM_CHANNEL_COUNT; indx++) {
		pwmData.PWM_VALUES[indx] = constrainToRange(pwmData.PWM_VALUES[indx], 0, RC_CHANNEL_DELTA_VALUE);
		setPWMChannelValue(indx, pwmData.PWM_VALUES[indx] + OUTPUT_PWM_PRESET);
	}
}

__ATTR_ITCM_TEXT
void outputControlTask() {
# if DEBUG_ENABLED == 1
	float dt = getDeltaTime(OUTPUT_CONTROL_TIMER_CHANNEL);
	pwmData.updateDt = dt;
#endif
	if (fcStatusData.canFly) {
		float throttleControl = controlData.throttleControl;
		float pitchControl = controlData.pitchControl;
		float rollControl = controlData.rollControl;
		float yawControl = controlData.yawControl;
		pwmData.PWM_VALUES[0] = throttleControl - pitchControl - rollControl + yawControl;
		pwmData.PWM_VALUES[1] = throttleControl - pitchControl + rollControl - yawControl;
		pwmData.PWM_VALUES[2] = throttleControl + pitchControl - rollControl - yawControl;
		pwmData.PWM_VALUES[3] = throttleControl + pitchControl + rollControl + yawControl;
		updatePWMValues();
	} else {
		stopOutputs();
	}

}

__ATTR_ITCM_TEXT
void idlePWMs() {
	if (outputControlInitStatus) {
		for (uint8_t indx = 0; indx < PWM_CHANNEL_COUNT; indx++) {
			pwmData.PWM_VALUES[indx] = 0;
		}
		updatePWMValues();
	}
}

__ATTR_ITCM_TEXT
void setPWMChannelValue(uint8_t channel, int value) {
	updatePWM(channel, value < 0 ? 0 : value);
}

__ATTR_ITCM_TEXT
void stopOutputs(void) {
	idlePWMs();
}

