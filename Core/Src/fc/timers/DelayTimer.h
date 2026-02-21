#ifndef TIMER_DELAY_DELAYTIMER_H_
#define TIMER_DELAY_DELAYTIMER_H_
#include "stm32h7xx_ll_utils.h"
#include <stdint.h>

void delayMs(uint32_t delay);

uint8_t initDelayTimer();

#endif
