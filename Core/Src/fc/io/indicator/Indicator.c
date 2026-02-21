#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_gpio.h"

#include <stdint.h>
#include "Indicator.h"
#include "led/LED.h"

uint8_t initIndicators(void) {
	return deviceLedsInit();
}

void startUpIndicatorOff() {
	deviceLed1Off();
}

void startUpIndicatorBlink() {
	deviceLed1Toggle();
}

void startUpIndicatorOn() {
	deviceLed1On();
}

void processingIndicatorOff() {
	deviceLed3Off();
}

void processingIndicatorBlink() {
	deviceLed3Toggle();
}

void processingIndicatorOn() {
	deviceLed3On();
}

void errorIndicatorOff() {
	deviceLed2Off();
}

void errorIndicatorBlink() {
	deviceLed2Toggle();
}

void errorIndicatorOn() {
	deviceLed2On();
}
