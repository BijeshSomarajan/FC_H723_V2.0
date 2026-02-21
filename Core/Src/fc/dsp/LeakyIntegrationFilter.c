#include "LeakyIntegrationFlilter.h"

void leakyIntegrationFilterInit(LEAKYINTEGRATIONFILTER *self, float tau) {
	// A smaller alpha makes the filter "leak" more slowly, leading to a smoother output.
	// A larger alpha causes the output to track the input more closely.
	self->tau = tau;
}

float leakyIntegrationFilterUpdate(LEAKYINTEGRATIONFILTER *self, float input, float dt) {
	// 1. Calculate the decay factor based on Tau
	// This ensures the leak is consistent regardless of 1kHz or 3.2kHz
	float leakFactor = 1.0f - (dt / self->tau);
	// Safety clamp to prevent negative leak if dt > tau (unlikely at 3.2kHz)
	if (leakFactor < 0.0f) {
		leakFactor = 0.0f;
	}
	// 2. The unit-preserving update:
	// Velocity = (Previous_Velocity * Leak) + (Acceleration * dt)
	self->output = (self->output * leakFactor) + (input * dt);
	return self->output;
}

void leakyIntegrationFilterReset(LEAKYINTEGRATIONFILTER *self, float value) {
	self->output = value;
}

