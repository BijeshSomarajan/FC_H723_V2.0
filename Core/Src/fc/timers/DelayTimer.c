#include "DelayTimer.h"

/**
 * Delays for the given amount of milliseconds
 */
void delayMs(uint32_t delay) {
	LL_mDelay(delay);
}

/**
 * Initializes the delay timer
 */
uint8_t initDelayTimer() {
	return 1;
}
