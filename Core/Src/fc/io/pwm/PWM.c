#include "PWM.h"

#include <stm32h723xx.h>
#include <stm32h7xx_ll_bus.h>
#include <stm32h7xx_ll_gpio.h>
#include <stm32h7xx_ll_tim.h>
#include <sys/_stdint.h>


/* =========================================================
 * Clock & frequency configuration
 * ========================================================= */
#define TIM1_CLK_HZ              275000000UL   /* APB2 timer clock */
#define TIM1_TICK_HZ             8000000UL     /* This gives 1000 steps of resolution for OneShot125 */
#define PWM_FREQ_STANDARD_HZ     400U          /* Standard ESC PWM */
#define PWM_FREQ_ONESHOT_HZ      20000U        /* OneShot / fast PWM */

PWMMode_t pwmMode;
/* =========================================================
 * Initialize TIM1 PWM
 * ========================================================= */
uint8_t initPWM(PWMMode_t mode) {
	pwmMode = mode;
	LL_TIM_InitTypeDef TIM_InitStruct = { 0 };
	LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = { 0 };
	LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = { 0 };
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	uint32_t autoreload;

	/* Select frequency based on mode */
	if (pwmMode == PWM_MODE_ONESHOT) {
		// OneShot125: Max pulse is 250us. Let's set period to 500us (2kHz)
		autoreload = 4000 - PWM_MODE_ONESHOT_FREQUENCY_ADJUST; // (8MHz * 0.0005s)
	} else {
		// Standard: 400Hz (2500us period)
		autoreload = 20000 - PWM_MODE_STANDARD_FREQUENCY_ADJUST; // (8MHz * 0.0025s)
	}

	/* Compute prescaler and period */
	uint32_t prescaler = (TIM1_CLK_HZ / TIM1_TICK_HZ) - 1U;

	/* Enable clocks */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE);
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);

	/* =====================================================
	 * GPIO configuration
	 * ===================================================== */
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1;

	/* TIM1_CH1 -> PE9 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
	LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/* TIM1_CH2/3/4 -> PA9/10/11 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_9 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* =====================================================
	 * Timer base configuration
	 * ===================================================== */
	TIM_InitStruct.Prescaler = prescaler;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = autoreload;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	TIM_InitStruct.RepetitionCounter = 0;

	LL_TIM_Init(TIM1, &TIM_InitStruct);
	LL_TIM_EnableARRPreload(TIM1);
	LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);

	/* =====================================================
	 * PWM channel configuration
	 * ===================================================== */
	TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
	TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.CompareValue = 0;
	TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;

	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);

	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);

	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);

	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);

	/* =====================================================
	 * Break & dead-time (minimal, safe defaults)
	 * ===================================================== */
	TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
	TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
	TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
	TIM_BDTRInitStruct.DeadTime = 0;
	TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
	TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
	TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
	TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_ENABLE;

	LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);

	/* =====================================================
	 * Enable outputs & start timer
	 * ===================================================== */
	LL_TIM_EnableAllOutputs(TIM1);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH4);
	LL_TIM_GenerateEvent_UPDATE(TIM1);
	LL_TIM_EnableCounter(TIM1);

	return 1;
}

/* =========================================================
 * Update PWM duty cycle
 * ========================================================= */
__ATTR_ITCM_TEXT
void updatePWM(uint8_t channel, uint16_t value) {
	if (value < 1000) {
		value = 1000;
	} else if (value > 2000) {
		value = 2000;
	}
	uint32_t ticks;
	if (pwmMode == PWM_MODE_ONESHOT) {
		// OneShot125 is 1/8th the speed of standard PWM, At 8MHz, 1 tick = 0.125us.
		// 1000 "standard units" (125us) becomes exactly 1000 ticks.
		ticks = value;
	} else {
		// Standard PWM: 1000us * 8 ticks/us = 8000 ticks.
		ticks = (uint32_t) value * 8;
	}
	switch (channel) {
	case 0:
		LL_TIM_OC_SetCompareCH1(TIM1, ticks);
		break;
	case 1:
		LL_TIM_OC_SetCompareCH2(TIM1, ticks);
		break;
	case 2:
		LL_TIM_OC_SetCompareCH3(TIM1, ticks);
		break;
	case 3:
		LL_TIM_OC_SetCompareCH4(TIM1, ticks);
		break;
	default:
		break;
	}
}
