#ifndef SRC_FC_IO_PWM_PWM_H_
#define SRC_FC_IO_PWM_PWM_H_
#include <stdint.h>
#include "../../memory/Memory.h"

#define PWM_CHANNEL_COUNT 4

typedef enum {
    PWM_MODE_STANDARD,   // ~400–500 Hz (ESC PWM)
    PWM_MODE_ONESHOT     // ~8–20 kHz (OneShot / fast PWM)
} PWMMode_t;

#define PWM_MODE_ONESHOT_FREQUENCY_ADJUST 50
#define PWM_MODE_STANDARD_FREQUENCY_ADJUST 5

uint8_t initPWM(PWMMode_t mode);

void updatePWM(uint8_t channel,uint16_t value);

#endif /* SRC_FC_IO_PWM_PWM_H_ */
