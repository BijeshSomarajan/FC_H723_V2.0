#include "../util/MathUtil.h"
#include "BiQuadFilter.h"

void biQuadFilterReset(BIQUADFILTER *self) {
	self->x1 = 0;
	self->x2 = 0;
	self->y1 = 0;
	self->y2 = 0;
}

void biQuadCalculateCoeffs(BIQUADFILTER *self) {

	float omega = 2 * ONE_PI * self->center_freq / self->sample_rate;

#if BIQUAD_USE_APPROX_TRIG == 1
	float sn = sinApprox(omega);
	float cs = cosApprox(omega);
#else
	float sn = sinf(omega);
	float cs = cosf(omega);
#endif

	float alpha = sn / (2 * self->Q);
	//Only used for peaking and shelving filters
	float gain_abs = 0;
	float beta = 0;

	if (self->type == BIQUAD_PEAK || self->type == BIQUAD_LOWSHELF || self->type == BIQUAD_HIGHSHELF) {
		gain_abs = powf(10, self->gainDB / 40.0f);

#if BIQUAD_USE_FAST_SQRT == 1
		beta = fastSqrtf(2 * gain_abs);
#else
		beta = sqrtf(2 * gain_abs);
#endif

	}

	switch (self->type) {
	case BIQUAD_BANDPASS:
		self->b0 = alpha;
		self->b1 = 0;
		self->b2 = -alpha;
		self->a0 = 1 + alpha;
		self->a1 = -2 * cs;
		self->a2 = 1 - alpha;
		break;
	case BIQUAD_LOWPASS:
		self->b0 = (1 - cs) / 2;
		self->b1 = 1 - cs;
		self->b2 = (1 - cs) / 2;
		self->a0 = 1 + alpha;
		self->a1 = -2 * cs;
		self->a2 = 1 - alpha;
		break;
	case BIQUAD_HIGHPASS:
		self->b0 = (1 + cs) / 2;
		self->b1 = -(1 + cs);
		self->b2 = (1 + cs) / 2;
		self->a0 = 1 + alpha;
		self->a1 = -2 * cs;
		self->a2 = 1 - alpha;
		break;
	case BIQUAD_NOTCH:
		self->b0 = 1;
		self->b1 = -2 * cs;
		self->b2 = 1;
		self->a0 = 1 + alpha;
		self->a1 = -2 * cs;
		self->a2 = 1 - alpha;
		break;
	case BIQUAD_PEAK:
		self->b0 = 1 + (alpha * gain_abs);
		self->b1 = -2 * cs;
		self->b2 = 1 - (alpha * gain_abs);
		self->a0 = 1 + (alpha / gain_abs);
		self->a1 = -2 * cs;
		self->a2 = 1 - (alpha / gain_abs);
		break;
	case BIQUAD_LOWSHELF:
		self->b0 = gain_abs * ((gain_abs + 1) - (gain_abs - 1) * cs + beta * sn);
		self->b1 = 2 * gain_abs * ((gain_abs - 1) - (gain_abs + 1) * cs);
		self->b2 = gain_abs * ((gain_abs + 1) - (gain_abs - 1) * cs - beta * sn);
		self->a0 = (gain_abs + 1) + (gain_abs - 1) * cs + beta * sn;
		self->a1 = -2 * ((gain_abs - 1) + (gain_abs + 1) * cs);
		self->a2 = (gain_abs + 1) + (gain_abs - 1) * cs - beta * sn;
		break;
	case BIQUAD_HIGHSHELF:
		self->b0 = gain_abs * ((gain_abs + 1) + (gain_abs - 1) * cs + beta * sn);
		self->b1 = -2 * gain_abs * ((gain_abs - 1) + (gain_abs + 1) * cs);
		self->b2 = gain_abs * ((gain_abs + 1) + (gain_abs - 1) * cs - beta * sn);
		self->a0 = (gain_abs + 1) - (gain_abs - 1) * cs + beta * sn;
		self->a1 = 2 * ((gain_abs - 1) - (gain_abs + 1) * cs);
		self->a2 = (gain_abs + 1) - (gain_abs - 1) * cs - beta * sn;
		break;
	}

	// prescale flter constants
	self->b0 /= self->a0;
	self->b1 /= self->a0;
	self->b2 /= self->a0;
	self->a1 /= self->a0;
	self->a2 /= self->a0;
}

void biQuadFilterInit(BIQUADFILTER *self, int type, float center_freq, float sample_rate, float Q, float gainDB) {
	Q = (Q == 0) ? 1e-9 : Q;
	self->type = type;
	self->sample_rate = sample_rate;
	self->Q = Q;
	self->gainDB = gainDB;
	self->center_freq = center_freq;

	biQuadFilterReset(self);
	biQuadCalculateCoeffs(self);
}

void biQuadFilterSetCenterFreq(BIQUADFILTER *self, float center_freq) {
	self->center_freq = center_freq;
	biQuadCalculateCoeffs(self);
}

float biQuadFilterUpdate(BIQUADFILTER *self, float data) {
	self->y = self->b0 * data + self->b1 * self->x1 + self->b2 * self->x2 - self->a1 * self->y1 - self->a2 * self->y2;
	self->x2 = self->x1;
	self->x1 = data;
	self->y2 = self->y1;
	self->y1 = self->y;
	return (self->y);
}



