#include "DeltaTimer.h"

#if USE_DWT_DELTA_TIMER != 1

#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_bus.h"
#include <stdint.h>

/* -----------------------------------------------------------
 * 🧩 Timer configuration constants
 * -----------------------------------------------------------*/
#define DELTA_TIMER_CHANNELS   8U    // Adjust per requirement

static const uint32_t TIM23_CLOCK_HZ = 275000000UL;  // APB1 timer clock (verify in RCC)
static const uint32_t TIM23_PRESCALER = (TIM23_CLOCK_HZ / 1000000UL) - 1UL; // 1 MHz tick (1 µs)
static const uint32_t TIM23_PERIOD = 0xFFFFFFFFUL;  // 32-bit free-run counter

/* 🧭 Per-channel state (last sample counter) */
static uint32_t deltaTimerPrevCounter[DELTA_TIMER_CHANNELS] = { 0 };

/* -----------------------------------------------------------
 * @brief  Initialize TIM23 as 1 MHz free-running timer
 * -----------------------------------------------------------*/
uint8_t initDeltaTimer(void) {
	/* Enable TIM23 peripheral clock */
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM23);

	/* Configure timer */
	LL_TIM_DisableCounter(TIM23);
	LL_TIM_SetPrescaler(TIM23, TIM23_PRESCALER);
	LL_TIM_SetAutoReload(TIM23, TIM23_PERIOD);
	LL_TIM_SetCounterMode(TIM23, LL_TIM_COUNTERMODE_UP);
	LL_TIM_SetClockDivision(TIM23, LL_TIM_CLOCKDIVISION_DIV1);

	/* Start timer */
	LL_TIM_SetCounter(TIM23, 0U);
	LL_TIM_EnableCounter(TIM23);
	LL_TIM_GenerateEvent_UPDATE(TIM23);

	/* Initialize channel references */
	for (uint8_t i = 0U; i < DELTA_TIMER_CHANNELS; i++) {
		deltaTimerPrevCounter[i] = 0U;
	}

	return 0U; // OK
}

/* -----------------------------------------------------------
 * @brief  Get elapsed time for a specific channel
 * @param  channel  Channel index (0..DELTA_TIMER_CHANNELS-1)
 * @retval float    Elapsed time (seconds)
 * -----------------------------------------------------------*/
float getDeltaTime(uint8_t channel) {
	if (channel >= DELTA_TIMER_CHANNELS) {
		return 0.0f; // Invalid channel
	}
	uint32_t current = LL_TIM_GetCounter(TIM23);
	uint32_t prev = deltaTimerPrevCounter[channel];
	uint32_t delta;
	/* Handle overflow */
	if (current >= prev) {
		delta = current - prev;
	} else {
		delta = (0xFFFFFFFFUL - prev + current + 1UL);
	}
	/* Store new reference */
	deltaTimerPrevCounter[channel] = current;
	/* Convert µs → seconds (1 µs = 1e-6 s) */
	return ((float) delta * 1e-6f);
}

/* -----------------------------------------------------------
 * @brief  Reset a specific delta-time channel
 * -----------------------------------------------------------*/
void resetDeltaTime(uint8_t channel) {
	if (channel < DELTA_TIMER_CHANNELS) {
		deltaTimerPrevCounter[channel] = LL_TIM_GetCounter(TIM23);
	}
}

/* -----------------------------------------------------------
 * @brief  Reset all delta-time channels
 * -----------------------------------------------------------*/
void resetAllDeltaTimes(void) {
	uint32_t now = LL_TIM_GetCounter(TIM23);
	for (uint8_t i = 0U; i < DELTA_TIMER_CHANNELS; i++) {
		deltaTimerPrevCounter[i] = now;
	}
}

#endif
