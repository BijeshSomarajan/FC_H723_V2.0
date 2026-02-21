#ifndef CONTROL_PID_H_
#define CONTROL_PID_H_

#include "../dsp/LowPassFilter.h"

typedef struct _PID PID;
struct _PID {
	//Proportional Constant
	float kp;
	//Integral Constant
	float ki;
	//Derivative Constant
	float kd;
	//Smoothening for d term
	float dLPFk;
	//Previous Error
	float pInput;
	//integral value
	float p, i, d;
	//Final PID
	float pid;
	//Output limits
	float limitMin;
	float limitMax;
	//Output limits
	float limitPMin;
	float limitPMax;
	float limitDMin;
	float limitDMax;
	float limitIMin;
	float limitIMax;
	//Suspension/pause Flags
	uint8_t suspendITerm;
	//D term low pass filter
	LOWPASSFILTER dLPF;
};

void pidInit(PID *self, float p_kp, float p_ki, float p_kd, float dLPFk);
void pidSetPIDOutputLimits(PID *self, float pidMin, float pidMax);

void pidSetPOutputLimits(PID *self, float p_Min, float p_Max);
void pidSetDOutputLimits(PID *self, float d_Min, float d_Max);
void pidSetIOutputLimits(PID *self, float i_Min, float i_Max);

void pidReset(PID *self);
void pidResetI(PID *self);
void pidResetD(PID *self);
void pidUpdate(PID *self, float angle, float sp, float dt);

void pidUpdateWithGains(PID *self, float input, float sp, float dt, float kpGain, float kiGain, float kdGain);

void pidSetKP(PID *self, float p_kp);
void pidSetKI(PID *self, float p_ki);
void pidSetKD(PID *self, float p_kd);

#endif /* CONTROL_PID_H_ */
