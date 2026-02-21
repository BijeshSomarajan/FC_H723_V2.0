#ifndef SRC_FC_SENSORS_INDICATOR_INDICATOR_H_
#define SRC_FC_SENSORS_INDICATOR_INDICATOR_H_

#include <stdint.h>

uint8_t deviceLedsInit(void);

void deviceLed1Toggle(void) ;
void deviceLed1On();
void deviceLed1Off();

void deviceLed2Toggle(void) ;
void deviceLed2On();
void deviceLed2Off();

void deviceLed3Toggle(void) ;
void deviceLed3On();
void deviceLed3Off();

#endif
