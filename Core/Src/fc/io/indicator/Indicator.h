#ifndef SRC_FC_INDICATOR_H_
#define SRC_FC__INDICATOR_H_

#include <stdint.h>

uint8_t initIndicators(void);

void startUpIndicatorOff();
void startUpIndicatorBlink();
void startUpIndicatorOn();

void processingIndicatorOff();
void processingIndicatorBlink();
void processingIndicatorOn();

void errorIndicatorOff();
void errorIndicatorBlink();
void errorIndicatorOn();

#endif
