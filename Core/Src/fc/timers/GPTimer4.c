#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx.h"
#include <stdint.h>
#include "GPTimer.h"

/* Store callback pointer */
static GPTimerCallback_t gpTimer4Callback = 0;

/* -----------------------------------------------------------
 * @brief  Initialize GPTimer4 (TIM4) with desired frequency
 * @param  frequencyHz   Desired interrupt frequency
 * @param  callback      Function to call on update event
 * @param  priority      NVIC interrupt priority
 * @retval 0 OK, 1 error
 * -----------------------------------------------------------*/
uint8_t initGPTimer4(uint32_t frequencyHz, GPTimerCallback_t callback, uint8_t priority) {
    if ((frequencyHz == 0U) || (callback == 0)) return 1;

    gpTimer4Callback = callback;

    /* Enable TIM4 clock (APB1 Group 1) */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

    /* Stop counter while configuring */
    LL_TIM_DisableCounter(TIM4);

    /* TIM4 clock = APB1 timer clock (same as your TIM23 & TIM24 setup) */
    uint32_t tim_clk = 275000000UL;

    /* Compute PSC & ARR
     * F = tim_clk / ((PSC + 1) * (ARR + 1))
     */
    uint32_t psc = (tim_clk / (frequencyHz * 65535UL));
    if (psc > 0xFFFFU) psc = 0xFFFFU;

    uint32_t arr = (tim_clk / ((psc + 1) * frequencyHz)) - 1U;
    if (arr > 0xFFFFFFFFU) arr = 0xFFFFFFFFU;

    /* Apply timer configuration */
    LL_TIM_SetPrescaler(TIM4, psc);
    LL_TIM_SetAutoReload(TIM4, arr);
    LL_TIM_SetCounterMode(TIM4, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetClockDivision(TIM4, LL_TIM_CLOCKDIVISION_DIV1);

    LL_TIM_ClearFlag_UPDATE(TIM4);
    LL_TIM_EnableIT_UPDATE(TIM4);

    /* NVIC configuration */
    NVIC_SetPriority(TIM4_IRQn, priority);
    NVIC_EnableIRQ(TIM4_IRQn);

    LL_TIM_GenerateEvent_UPDATE(TIM4);

    return 0;
}

/* -----------------------------------------------------------
 * @brief  Start GPTimer4
 * -----------------------------------------------------------*/
void startGPTimer4(void) {
    LL_TIM_SetCounter(TIM4, 0);
    LL_TIM_EnableCounter(TIM4);
}

/* -----------------------------------------------------------
 * @brief  Stop GPTimer4
 * -----------------------------------------------------------*/
void stopGPTimer4(void) {
    LL_TIM_DisableCounter(TIM4);
}

/* -----------------------------------------------------------
 * @brief TIM4 interrupt handler → calls GPTimer4 callback
 * -----------------------------------------------------------*/
void TIM4_IRQHandler(void) {
    if (LL_TIM_IsActiveFlag_UPDATE(TIM4)) {
        LL_TIM_ClearFlag_UPDATE(TIM4);

        if (gpTimer4Callback) gpTimer4Callback();
    }
}
