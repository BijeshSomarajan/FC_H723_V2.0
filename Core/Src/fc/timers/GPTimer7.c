#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx.h"
#include <stdint.h>

/* -----------------------------------------------------------
 * Callback type definition
 * -----------------------------------------------------------*/
typedef void (*GPTimerCallback_t)(void);

/* -----------------------------------------------------------
 * Static callback storage
 * -----------------------------------------------------------*/
static GPTimerCallback_t gpTimer7Callback = 0;

/* -----------------------------------------------------------
 * @brief  Initialize GPTimer7 (TIM7) with desired frequency
 * @param  frequencyHz   Desired interrupt frequency
 * @param  callback      Function to call on update event
 * @param  priority      NVIC interrupt priority
 * @retval 0 OK, 1 error
 * -----------------------------------------------------------*/
uint8_t initGPTimer7(uint32_t frequencyHz, GPTimerCallback_t callback, uint8_t priority) {
	if ((frequencyHz == 0U) || (callback == 0)) {
		return 1;
	}

	gpTimer7Callback = callback;

	/* Enable TIM7 clock (APB1 Group 1) */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);

	/* Stop counter while configuring */
	LL_TIM_DisableCounter(TIM7);

	/*
	 * TIM7 clock source
	 * Must match your APB1 timer clock.
	 * Using the same value as TIM4 / TIM6.
	 */
	uint32_t tim_clk = 275000000UL;

	/*
	 * Compute PSC & ARR
	 * Frequency = tim_clk / ((PSC + 1) * (ARR + 1))
	 * TIM7 ARR is 16-bit
	 */
	uint32_t psc = tim_clk / (frequencyHz * 65536UL);
	if (psc > 0xFFFFU) {
		psc = 0xFFFFU;
	}

	uint32_t arr = (tim_clk / ((psc + 1U) * frequencyHz)) - 1U;
	if (arr > 0xFFFFU) {
		arr = 0xFFFFU;
	}

	/* Apply timer configuration */
	LL_TIM_SetPrescaler(TIM7, psc);
	LL_TIM_SetAutoReload(TIM7, arr);
	LL_TIM_SetCounterMode(TIM7, LL_TIM_COUNTERMODE_UP);

	/* Clear update flag and enable interrupt */
	LL_TIM_ClearFlag_UPDATE(TIM7);
	LL_TIM_EnableIT_UPDATE(TIM7);

	/* NVIC configuration */
	NVIC_SetPriority(TIM7_IRQn, priority);
	NVIC_EnableIRQ(TIM7_IRQn);

	/* Force update to load PSC & ARR */
	LL_TIM_GenerateEvent_UPDATE(TIM7);

	return 0;
}

/* -----------------------------------------------------------
 * @brief  Start GPTimer7
 * -----------------------------------------------------------*/
void startGPTimer7(void) {
	LL_TIM_SetCounter(TIM7, 0);
	LL_TIM_EnableCounter(TIM7);
}

/* -----------------------------------------------------------
 * @brief  Stop GPTimer7
 * -----------------------------------------------------------*/
void stopGPTimer7(void) {
	LL_TIM_DisableCounter(TIM7);
}

/* -----------------------------------------------------------
 * @brief  TIM7 interrupt handler
 * -----------------------------------------------------------*/
void TIM7_IRQHandler(void) {
	if (LL_TIM_IsActiveFlag_UPDATE(TIM7)) {
		LL_TIM_ClearFlag_UPDATE(TIM7);

		if (gpTimer7Callback) {
			gpTimer7Callback();
		}
	}
}
