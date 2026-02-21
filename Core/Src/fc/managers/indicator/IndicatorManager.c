#include "../../io/indicator/Indicator.h"
#include "IndicatorManager.h"
#include "../../FCConfig.h"
#include "../../timers/Scheduler.h"
#include "../../status/FCStatus.h"

#define ERROR_CRASH_THRESHOLD 4
#define ERROR_TX_INACTIVE_THRESHOLD 16

#define PROCESSING_CONFIG_THRESHOLD 2
#define PROCESSING_STABILIZATION_THRESHOLD 4
#define PROCESSING_RC_START_WAITING_THRESHOLD 8
#define PROCESSING_RC_WAITING_THRESHOLD 20

void updateIndictors(void);

uint8_t indicatorManagerInitialized = 0;
uint16_t indicatorCount = 0;

uint8_t initIndicatorManager() {
	if (!indicatorManagerInitialized) {
		indicatorManagerInitialized = initIndicators();
		schedulerAddTask(updateIndictors, INDICATOR_UPDATE_FREQUENCY, INDICATOR_TASK_PRIORITY);
	}
	return indicatorManagerInitialized;
}

void indicateStartUpSuccess() {
	startUpIndicatorOn();
	errorIndicatorOff();
}

void indicateStartUpFailed() {
	startUpIndicatorBlink();
	errorIndicatorOn();
}

uint8_t needIndicatorStateChange(uint16_t indicatorThreshold) {
	if (indicatorThreshold != 0) {
		indicatorCount++;
		if (indicatorCount > indicatorThreshold) {
			indicatorCount = 0;
			return 1;
		}
	}
	return 0;
}

void updateIndictors() {
	if (!fcStatusData.hasInitialized) {
		errorIndicatorOn();
		processingIndicatorOff();
	} else if (fcStatusData.isConfigMode) {
		errorIndicatorOff();
		if (needIndicatorStateChange(PROCESSING_CONFIG_THRESHOLD)) {
			processingIndicatorBlink();
		}
	} else if (fcStatusData.hasCrashed) {
		processingIndicatorOff();
		if (needIndicatorStateChange(ERROR_CRASH_THRESHOLD)) {
			errorIndicatorBlink();
		}
	} else if (!fcStatusData.isTxOn) {
		processingIndicatorOff();
		if (needIndicatorStateChange(ERROR_TX_INACTIVE_THRESHOLD)) {
			errorIndicatorBlink();
		}
	} else if (fcStatusData.canFly) {
		errorIndicatorOff();
		processingIndicatorOn();
	} else if (fcStatusData.canStabilize) {
		errorIndicatorOff();
		if (needIndicatorStateChange(PROCESSING_STABILIZATION_THRESHOLD)) {
			processingIndicatorBlink();
		}
	} else if (fcStatusData.canStart) {
		errorIndicatorOff();
		if (needIndicatorStateChange(PROCESSING_RC_START_WAITING_THRESHOLD)) {
			processingIndicatorBlink();
		}
	} else if (fcStatusData.isTxOn) {
		errorIndicatorOff();
		if (needIndicatorStateChange(PROCESSING_RC_WAITING_THRESHOLD)) {
			processingIndicatorBlink();
		}
	} else {
		errorIndicatorOff();
		processingIndicatorOff();
	}

}
