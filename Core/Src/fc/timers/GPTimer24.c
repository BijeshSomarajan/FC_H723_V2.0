#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx.h"
#include <stdint.h>
#include "GPTimer.h"

/* Stored callback pointer */
static GPTimerCallback_t gpTimer24Callback = 0;

/* -----------------------------------------------------------
 * @brief  Initialize GPTimer24 (TIM24) with given frequency
 * @param  frequencyHz   Desired interrupt frequency
 * @param  callback      Function to call in ISR
 * @param  priority      NVIC priority level
 * @retval 0 OK, 1 error
 * -----------------------------------------------------------*/
uint8_t initGPTimer24(uint32_t frequencyHz, GPTimerCallback_t callback, uint8_t priority) {
    if ((frequencyHz == 0U) || (callback == 0)) return 1;

    gpTimer24Callback = callback;

    /* Enable TIM24 peripheral clock */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM24);

    /* Stop counter while configuring */
    LL_TIM_DisableCounter(TIM24);

    /* Timer clock */
    uint32_t tim_clk = 275000000UL;

    /* Compute PSC & ARR
     * F = tim_clk / ((PSC + 1)*(ARR + 1))
     */
    uint32_t psc = (tim_clk / (frequencyHz * 65535UL));
    if (psc > 0xFFFFU) psc = 0xFFFFU;

    uint32_t arr = (tim_clk / ((psc + 1) * frequencyHz)) - 1U;
    if (arr > 0xFFFFFFFFU) arr = 0xFFFFFFFFU;

    /* Apply timer configuration */
    LL_TIM_SetPrescaler(TIM24, psc);
    LL_TIM_SetAutoReload(TIM24, arr);
    LL_TIM_SetCounterMode(TIM24, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetClockDivision(TIM24, LL_TIM_CLOCKDIVISION_DIV1);

    LL_TIM_ClearFlag_UPDATE(TIM24);
    LL_TIM_EnableIT_UPDATE(TIM24);

    /* NVIC configuration */
    NVIC_SetPriority(TIM24_IRQn, priority);
    NVIC_EnableIRQ(TIM24_IRQn);

    LL_TIM_GenerateEvent_UPDATE(TIM24);

    return 0;
}

/* -----------------------------------------------------------
 * @brief  Start GPTimer24
 * -----------------------------------------------------------*/
void startGPTimer24(void) {
    LL_TIM_SetCounter(TIM24, 0);
    LL_TIM_EnableCounter(TIM24);
}

/* -----------------------------------------------------------
 * @brief  Stop GPTimer24
 * -----------------------------------------------------------*/
void stopGPTimer24(void) {
    LL_TIM_DisableCounter(TIM24);
}

/* -----------------------------------------------------------
 * @brief  TIM24 interrupt handler → calls GPTimer24 callback
 * -----------------------------------------------------------*/
void TIM24_IRQHandler(void) {
    if (LL_TIM_IsActiveFlag_UPDATE(TIM24)) {
        LL_TIM_ClearFlag_UPDATE(TIM24);

        if (gpTimer24Callback) gpTimer24Callback();
    }
}
