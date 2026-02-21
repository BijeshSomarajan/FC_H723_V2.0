#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx.h"
#include <stdint.h>

/* -----------------------------------------------------------
 * Callback type
 * -----------------------------------------------------------*/
typedef void (*GPTimerCallback_t)(void);

/* -----------------------------------------------------------
 * Static callback storage
 * -----------------------------------------------------------*/
static GPTimerCallback_t gpTimer6Callback = 0;

/* -----------------------------------------------------------
 * @brief  Initialize GPTimer6 (TIM6) with desired frequency
 * @param  frequencyHz   Desired interrupt frequency
 * @param  callback      Function to call on update event
 * @param  priority      NVIC interrupt priority
 * @retval 0 OK, 1 error
 * -----------------------------------------------------------*/
uint8_t initGPTimer6(uint32_t frequencyHz, GPTimerCallback_t callback, uint8_t priority) {
	if ((frequencyHz == 0U) || (callback == 0)) {
		return 1;
	}

	gpTimer6Callback = callback;

	/* Enable TIM6 clock (APB1 Group 1) */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

	/* Stop counter while configuring */
	LL_TIM_DisableCounter(TIM6);

	/*
	 * TIM6 clock
	 * NOTE:
	 * This must match your APB1 timer clock.
	 * Using the same value you used for TIM4.
	 */
	uint32_t tim_clk = 275000000UL;

	/*
	 * Compute PSC & ARR
	 * Frequency = tim_clk / ((PSC + 1) * (ARR + 1))
	 * TIM6 ARR is 16-bit
	 */
	uint32_t psc = tim_clk / (frequencyHz * 65536UL);
	if (psc > 0xFFFFU) {
		psc = 0xFFFFU;
	}

	uint32_t arr = (tim_clk / ((psc + 1U) * frequencyHz)) - 1U;
	if (arr > 0xFFFFU) {
		arr = 0xFFFFU;
	}

	/* Apply configuration */
	LL_TIM_SetPrescaler(TIM6, psc);
	LL_TIM_SetAutoReload(TIM6, arr);
	LL_TIM_SetCounterMode(TIM6, LL_TIM_COUNTERMODE_UP);

	/* Clear & enable update interrupt */
	LL_TIM_ClearFlag_UPDATE(TIM6);
	LL_TIM_EnableIT_UPDATE(TIM6);

	/* NVIC configuration */
	NVIC_SetPriority(TIM6_DAC_IRQn, priority);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);

	/* Force update event to load registers */
	LL_TIM_GenerateEvent_UPDATE(TIM6);

	return 0;
}

/* -----------------------------------------------------------
 * @brief  Start GPTimer6
 * -----------------------------------------------------------*/
void startGPTimer6(void) {
	LL_TIM_SetCounter(TIM6, 0);
	LL_TIM_EnableCounter(TIM6);
}

/* -----------------------------------------------------------
 * @brief  Stop GPTimer6
 * -----------------------------------------------------------*/
void stopGPTimer6(void) {
	LL_TIM_DisableCounter(TIM6);
}

/* -----------------------------------------------------------
 * @brief  TIM6 interrupt handler
 * -----------------------------------------------------------*/
void TIM6_DAC_IRQHandler(void) {
	if (LL_TIM_IsActiveFlag_UPDATE(TIM6)) {
		LL_TIM_ClearFlag_UPDATE(TIM6);

		if (gpTimer6Callback) {
			gpTimer6Callback();
		}
	}
}

