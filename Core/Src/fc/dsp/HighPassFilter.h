#ifndef HIGHPASSFILTER_H_
#define HIGHPASSFILTER_H_
#include <math.h>

#include "../util/MathUtil.h"

typedef struct _HIGHPASSFILTER HIGHPASSFILTER;

struct _HIGHPASSFILTER {
	float pData;
	float output;
	float cutOff;
	float rc;
};

void highPassFilterInit(HIGHPASSFILTER* self, float cutoff);
void highPassFilterReset(HIGHPASSFILTER *self);
float highPassFilterUpdate(HIGHPASSFILTER* self, float data, float dt);

#endif /* HIGHPASSFILTER_H_ */
