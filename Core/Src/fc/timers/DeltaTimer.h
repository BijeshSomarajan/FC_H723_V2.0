#ifndef TIMER_DELTA_DELTATIMER_H_
#define TIMER_DELTA_DELTATIMER_H_

#include "DeltaTimer.h"

#include <stdint.h>
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

#define USE_DWT_DELTA_TIMER 1

uint8_t initDeltaTimer(void);
float getDeltaTime(uint8_t channel);
void resetDeltaTime(uint8_t channel);
void resetAllDeltaTimes(void);


#endif
