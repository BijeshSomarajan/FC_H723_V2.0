#include "LowPassFilter.h"

/************************************************************************/
/* Initialize the low pass filter                                       */
/************************************************************************/
void lowPassFilterInit(LOWPASSFILTER *self, float cutOff) {
	self->cutOff = cutOff;
	self->rc = 1.0f / (self->cutOff * 2.0f * ONE_PI);
	self->output = 0;
}

/**
 * Sets the cut off , Retains the output
 */
void lowPassFilterSetCutOff(LOWPASSFILTER *self, float cutOff) {
	self->cutOff = cutOff;
	self->rc = 1.0f / (self->cutOff * 2.0f * ONE_PI);
}

/************************************************************************/
/* Resets the low pass filter                                       */
/************************************************************************/
void lowPassFilterReset(LOWPASSFILTER *self) {
	self->output = 0;
}

/************************************************************************/
/* Resets the low pass filter to a value                                */
/************************************************************************/
void lowPassFilterResetToValue(LOWPASSFILTER *self, float value) {
	self->output = value;
}

/************************************************************************/
/* Do low pass update                                                */
/************************************************************************/

float lowPassFilterUpdate(LOWPASSFILTER *self, float data, float dt) {
	float alpha = dt / (self->rc + dt);
	self->output = self->output + (alpha * (data - self->output));
	return self->output;
}

