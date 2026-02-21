#ifndef _LOWPASSFILTER_H_
#define _LOWPASSFILTER_H_

#include <math.h>

#include "../util/MathUtil.h"

typedef struct _LOWPASSFILTER LOWPASSFILTER;

struct _LOWPASSFILTER {
	float rc;
	float cutOff;
	float output;
	float alpha;
};

void lowPassFilterInit(LOWPASSFILTER *self, float cutOff);
void lowPassFilterReset(LOWPASSFILTER *self);
void lowPassFilterResetToValue(LOWPASSFILTER *self , float value);
float lowPassFilterUpdate(LOWPASSFILTER *self, float data, float dt);
void lowPassFilterSetCutOff(LOWPASSFILTER *self, float cutOff);
#endif
