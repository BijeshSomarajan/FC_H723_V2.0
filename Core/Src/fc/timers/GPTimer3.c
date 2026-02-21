#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx.h"
#include <stdint.h>
#include "GPTimer.h"

/* Store callback pointer */
static GPTimerCallback_t gpTimer3Callback = 0;

/* -----------------------------------------------------------
 * @brief  Initialize GPTimer3 (TIM3) with desired frequency
 * @param  frequencyHz   Desired interrupt frequency
 * @param  callback      Function to call on update event
 * @param  priority      NVIC interrupt priority
 * @retval 0 OK, 1 error
 * -----------------------------------------------------------*/
uint8_t initGPTimer3(uint32_t frequencyHz, GPTimerCallback_t callback, uint8_t priority) {
	if ((frequencyHz == 0U) || (callback == 0)) {
		return 1;
	}

	gpTimer3Callback = callback;

	/* Enable TIM3 clock (APB1 Group 1) */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

	/* Stop counter while configuring */
	LL_TIM_DisableCounter(TIM3);

	/* TIM3 clock = APB1 timer clock */
	uint32_t tim_clk = 275000000UL;

	/* Compute PSC & ARR
	 * F = tim_clk / ((PSC + 1) * (ARR + 1))
	 */
	uint32_t psc = (tim_clk / (frequencyHz * 65535UL));
	if (psc > 0xFFFFU) {
		psc = 0xFFFFU;
	}

	uint32_t arr = (tim_clk / ((psc + 1U) * frequencyHz)) - 1U;
	if (arr > 0xFFFFFFFFU) {
		arr = 0xFFFFFFFFU;
	}

	/* Apply timer configuration */
	LL_TIM_SetPrescaler(TIM3, psc);
	LL_TIM_SetAutoReload(TIM3, arr);
	LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP);
	LL_TIM_SetClockDivision(TIM3, LL_TIM_CLOCKDIVISION_DIV1);

	LL_TIM_ClearFlag_UPDATE(TIM3);
	LL_TIM_EnableIT_UPDATE(TIM3);

	/* NVIC configuration */
	NVIC_SetPriority(TIM3_IRQn, priority);
	NVIC_EnableIRQ(TIM3_IRQn);

	/* Force update event to load registers */
	LL_TIM_GenerateEvent_UPDATE(TIM3);

	return 0;
}

/* -----------------------------------------------------------
 * @brief  Start GPTimer3
 * -----------------------------------------------------------*/
void startGPTimer3(void) {
	LL_TIM_SetCounter(TIM3, 0);
	LL_TIM_EnableCounter(TIM3);
}

/* -----------------------------------------------------------
 * @brief  Stop GPTimer3
 * -----------------------------------------------------------*/
void stopGPTimer3(void) {
	LL_TIM_DisableCounter(TIM3);
}

/* -----------------------------------------------------------
 * @brief TIM3 interrupt handler → calls GPTimer3 callback
 * -----------------------------------------------------------*/
void TIM3_IRQHandler(void) {
	if (LL_TIM_IsActiveFlag_UPDATE(TIM3)) {
		LL_TIM_ClearFlag_UPDATE(TIM3);

		if (gpTimer3Callback) {
			gpTimer3Callback();
		}
	}
}
