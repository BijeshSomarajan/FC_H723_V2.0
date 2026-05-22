#ifndef SRC_FC_DSP_INTERPOLATOR_H_
#define SRC_FC_DSP_INTERPOLATOR_H_

typedef struct {
	float prev;
	float curr;
	float alpha;
	float duration;  // total interpolation time (seconds)
} INTERPOLATOR;

void interpolatorInit(INTERPOLATOR *i, float initialValue, float duration);
void interpolatorSetTarget(INTERPOLATOR *i, float newTarget);
float interpolatorUpdate(INTERPOLATOR *i, float dt);
void interpolatorReset(INTERPOLATOR *i, float initialValue);
#endif
