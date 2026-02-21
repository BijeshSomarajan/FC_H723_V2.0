#ifndef CORE_DSP_FILTER_LEAKYINTEGRATIONFLILTER_H_
#define CORE_DSP_FILTER_LEAKYINTEGRATIONFLILTER_H_
#include "../util/MathUtil.h"

typedef struct _LEAKYINTEGRATIONFILTER LEAKYINTEGRATIONFILTER;

struct _LEAKYINTEGRATIONFILTER {
	float tau; // The "leak" factor (usually 0.99 or similar)
	float output;
};

void leakyIntegrationFilterInit(LEAKYINTEGRATIONFILTER *self, float tau);
void leakyIntegrationFilterReset(LEAKYINTEGRATIONFILTER *self,float value);
float leakyIntegrationFilterUpdate(LEAKYINTEGRATIONFILTER *self, float input, float dt);

#endif /* CORE_DSP_FILTER_LEAKYINTEGRATIONFLILTER_H_ */
