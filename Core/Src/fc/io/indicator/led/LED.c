#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_gpio.h"

#include <stdint.h>

#include "../Indicator.h"
/*
Indicator
--------
PE3 ------> GPIO_INDICATOR1
PE4 ------> GPIO_INDICATOR2
PE5 ------> GPIO_INDICATOR3
*/
#define LED_PORT    GPIOE
#define LED1_PIN     LL_GPIO_PIN_3
#define LED2_PIN     LL_GPIO_PIN_4
#define LED3_PIN     LL_GPIO_PIN_5

uint8_t deviceLedsInit() {

	// For STM32H7, GPIOE is on the AHB4 bus.
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE);

	LL_GPIO_SetPinMode(LED_PORT, LED1_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(LED_PORT, LED1_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(LED_PORT, LED1_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
	LL_GPIO_SetPinPull(LED_PORT, LED1_PIN, LL_GPIO_PULL_DOWN);

	LL_GPIO_SetPinMode(LED_PORT, LED2_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(LED_PORT, LED2_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(LED_PORT, LED2_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
	LL_GPIO_SetPinPull(LED_PORT, LED2_PIN, LL_GPIO_PULL_DOWN);

	LL_GPIO_SetPinMode(LED_PORT, LED3_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(LED_PORT, LED3_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(LED_PORT, LED3_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
	LL_GPIO_SetPinPull(LED_PORT, LED3_PIN, LL_GPIO_PULL_DOWN);

	return 1;
}

void deviceLed1On(){
	LL_GPIO_SetOutputPin(LED_PORT, LED1_PIN);
}

void deviceLed1Off(){
	LL_GPIO_ResetOutputPin(LED_PORT, LED1_PIN);
}

void deviceLed1Toggle(void) {
	LL_GPIO_TogglePin(LED_PORT, LED1_PIN);
}

void deviceLed2On(){
	LL_GPIO_SetOutputPin(LED_PORT, LED2_PIN);
}

void deviceLed2Off(){
	LL_GPIO_ResetOutputPin(LED_PORT, LED2_PIN);
}

void deviceLed2Toggle(void) {
	LL_GPIO_TogglePin(LED_PORT, LED2_PIN);
}

void deviceLed3On(){
	LL_GPIO_SetOutputPin(LED_PORT, LED3_PIN);
}

void deviceLed3Off(){
	LL_GPIO_ResetOutputPin(LED_PORT, LED3_PIN);
}

void deviceLed3Toggle(void) {
	LL_GPIO_TogglePin(LED_PORT, LED3_PIN);
}
