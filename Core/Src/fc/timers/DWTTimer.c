#include "DeltaTimer.h"
#if USE_DWT_DELTA_TIMER == 1

#include "stm32h7xx.h"
#include <stdint.h>

/* Number of delta time channels */
#define DWT_DELTA_TIMER_CHANNELS   16

/* Store previous CYCCNT for each channel */
static uint32_t dwtPrevCycles[DWT_DELTA_TIMER_CHANNELS] = { 0 };

/* CPU frequency (must match system clock!) */
float CPU_FREQ_HZ = 550000000.0f;   // 550 MHz on STM32H723
float CPU_CYCLE_TO_SECONDS = 1.0f / 550000000.0f;

/* -----------------------------------------------------------
 * @brief  Initialize DWT cycle counter
 * -----------------------------------------------------------*/
uint8_t initDeltaTimer(void) {
	CPU_FREQ_HZ = (float) SystemCoreClock;
	CPU_CYCLE_TO_SECONDS = 1.0f / CPU_FREQ_HZ;

	/* Enable TRC */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

	/* Unlock DWT (if locked) */
	if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0) {
		DWT->LAR = 0xC5ACCE55;  // Required on H7
		DWT->CYCCNT = 0;        // Reset counter
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Enable counter
	}

	/* Initialize channel baselines */
	uint32_t now = DWT->CYCCNT;
	for (uint8_t i = 0; i < DWT_DELTA_TIMER_CHANNELS; i++) {
		dwtPrevCycles[i] = now;
	}

	return 0; // OK
}

/* -----------------------------------------------------------
 * @brief Get delta time for a channel (seconds)
 * -----------------------------------------------------------*/
float getDeltaTime(uint8_t channel) {
	if (channel >= DWT_DELTA_TIMER_CHANNELS) return 0.0f;

	uint32_t current = DWT->CYCCNT;
	uint32_t prev = dwtPrevCycles[channel];

	uint32_t DWTDelta;

	/* Handle 32-bit wrap-around */
	if (current >= prev) DWTDelta = current - prev;
	else DWTDelta = (0xFFFFFFFFUL - prev + current + 1UL);

	/* Update previous count */
	dwtPrevCycles[channel] = current;

	/* Convert cycles → seconds */
	return ((float) DWTDelta * CPU_CYCLE_TO_SECONDS);
}

/* -----------------------------------------------------------
 * @brief Reset a specific channel
 * -----------------------------------------------------------*/
void resetDeltaTime(uint8_t channel) {
	if (channel < DWT_DELTA_TIMER_CHANNELS) dwtPrevCycles[channel] = DWT->CYCCNT;
}

/* -----------------------------------------------------------
 * @brief Reset all channels
 * -----------------------------------------------------------*/
void resetAllDeltaTimes(void) {
	uint32_t now = DWT->CYCCNT;
	for (uint8_t i = 0; i < DWT_DELTA_TIMER_CHANNELS; i++) {
		dwtPrevCycles[i] = now;
	}
}

#endif
