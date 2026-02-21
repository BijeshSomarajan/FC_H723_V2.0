#include "Pid.h"

#include "../memory/Memory.h"

/************************************************************************/
/* Initialization of the PID params                                     */
/************************************************************************/
void pidInit(PID *self, float p_kp, float p_ki, float p_kd, float dLPFk) {
	self->kp = p_kp;
	self->ki = p_ki;
	self->kd = p_kd;
	if (dLPFk != 0) {
		self->dLPFk = dLPFk;
		lowPassFilterInit(&self->dLPF, dLPFk);
	} else {
		self->dLPFk = 0;
	}
	self->pInput = 0;
	self->i = 0;
}

/************************************************************************/
/* Set proportional gain                                                */
/************************************************************************/
void pidSetKP(PID *self, float p_kp) {
	self->kp = p_kp;
}

/************************************************************************/
/* Set integral gain                                                    */
/************************************************************************/
void pidSetKI(PID *self, float p_ki) {
	self->ki = p_ki;
}

/************************************************************************/
/* Set derivative gain                                                  */
/************************************************************************/
void pidSetKD(PID *self, float p_kd) {
	self->kd = p_kd;
}

/************************************************************************/
/* Set out put limits                                                   */
/************************************************************************/
void pidSetPIDOutputLimits(PID *self, float pidMin, float pidMax) {
	self->limitMax = pidMax;
	self->limitMin = pidMin;

	self->limitPMax = pidMax;
	self->limitPMin = pidMin;

	self->limitIMax = pidMax;
	self->limitIMin = pidMin;

	self->limitDMax = pidMax;
	self->limitDMin = pidMin;
}

/************************************************************************/
/* Set I out put limits                                                   */
/************************************************************************/
void pidSetPOutputLimits(PID *self, float p_Min, float p_Max) {
	self->limitPMax = p_Max;
	self->limitPMin = p_Min;
}

/************************************************************************/
/* Set I out put limits                                                   */
/************************************************************************/
void pidSetIOutputLimits(PID *self, float i_Min, float i_Max) {
	self->limitIMax = i_Max;
	self->limitIMin = i_Min;
}

/************************************************************************/
/* Set D out put limits                                                   */
/************************************************************************/
void pidSetDOutputLimits(PID *self, float d_Min, float d_Max) {
	self->limitDMax = d_Max;
	self->limitDMin = d_Min;
}

/************************************************************************/
/* Resets the PID ,integrated values                                   */
/************************************************************************/
void pidReset(PID *self) {
	self->pInput = 0;
	self->i = 0;
	self->pid = 0;
	if (self->dLPFk != 0) {
		lowPassFilterReset(&self->dLPF);
	}
}

/************************************************************************/
/* Resets the PID ,I term                                               */
/************************************************************************/
void pidResetI(PID *self) {
	self->i = 0;
}

/**
 Resets the PID ,D term
 */
void pidResetD(PID *self) {
	self->pInput = 0;
	if (self->dLPFk != 0) {
		lowPassFilterReset(&self->dLPF);
	}
}


__ATTR_ITCM_TEXT
void pidUpdateWithGains(PID *self, float input, float sp, float dt, float kpGain, float kiGain, float kdGain) {
    float et = sp - input;
    // 1. Proportional - Dynamic Gain Applied
    // Stored in self->p for independent monitoring
    self->p = constrainToRangeF(self->kp * kpGain * et, self->limitPMin, self->limitPMax);
    // 2. Integral - LOGIC: Scaling the Rate of Integration
    // This is the primary defense against the "Backward Dip" (suction)
    if (self->ki != 0.0f) {
        if (self->suspendITerm != 1) {
            // Precision grouping: (Error * (Base_Ki * Dynamic_Gain)) * Time
            // This allows the gain to crush the I-term authority during tilt
            self->i += (et * (self->ki * kiGain)) * dt;
            // Anti-windup clamping (Correct Order: Value, Min, Max)
            self->i = constrainToRangeF(self->i, self->limitIMin, self->limitIMax);
        }
    } else {
        self->i = 0.0f;
    }
    // 3. Derivative - Dynamic Gain + Measurement-based Logic
    float dTerm = 0.0f;
    if (self->kd != 0.0f) {
        // Calculate velocity of sensor change (ignores setpoint "kick")
        float deltaInput = (input - self->pInput);
        float dRaw = (dt > 0.0f) ? (deltaInput / dt) : 0.0f;
        if (self->dLPFk != 0) {
            // Apply gain to the filtered result for noise suppression
            dTerm = -self->kd * kdGain * lowPassFilterUpdate(&self->dLPF, dRaw, dt);
        } else {
            dTerm = -self->kd * kdGain * dRaw;
        }
        // Store current input for next 1kHz/3.2kHz cycle
        self->pInput = input;
        // Apply D-term specific limits
        dTerm = constrainToRangeF(dTerm, self->limitDMin, self->limitDMax);
    }
    self->d = dTerm; // Store isolated D term
    // 4. Final Output - Summing Isolated Components
    // Combine P, I, and D into a single control variable
    float totalPID = self->p + self->i + self->d;
    // Final global clamping to stay within motor/ESC headroom
    self->pid = constrainToRangeF(totalPID, self->limitMin, self->limitMax);
}

__ATTR_ITCM_TEXT
void pidUpdate(PID *self, float input, float sp, float dt) {
    float et = sp - input;
    // 1. Proportional - Standard
    // Calculate and store independently for logging/monitoring
    self->p = constrainToRangeF(self->kp * et, self->limitPMin, self->limitPMax);
    // 2. Integral - Mathematical grouping for 1kHz+ precision
    if (self->ki != 0.0f) {
        if (self->suspendITerm != 1) {
            // Group (Error * Ki) before dt to maintain floating point resolution
            // This prevents precision loss when dt is very small (e.g., 0.0003125)
            self->i += (et * self->ki) * dt;

            // Anti-windup clamping (Correct Order: Value, Min, Max)
            self->i = constrainToRangeF(self->i, self->limitIMin, self->limitIMax);
        }
    } else {
        self->i = 0.0f;
    }
    // 3. Derivative - Measurement-based to prevent setpoint kick
    float dTerm = 0.0f;
    if (self->kd != 0.0f) {
        // Calculate velocity of the sensor change, ignoring setpoint jumps
        float deltaInput = (input - self->pInput);
        float dRaw = (dt > 0.0f) ? (deltaInput / dt) : 0.0f;
        if (self->dLPFk != 0) {
            // Filter the raw derivative to handle sensor noise/vibration
            dTerm = -self->kd * lowPassFilterUpdate(&self->dLPF, dRaw, dt);
        } else {
            dTerm = -self->kd * dRaw;
        }
        // Store input for next cycle calculation
        self->pInput = input;
        // Apply D-term specific limits
        dTerm = constrainToRangeF(dTerm, self->limitDMin, self->limitDMax);
    }
    self->d = dTerm; // Store isolated D term
    // 4. Final Output - Summing isolated components
    // Combining isolated P, I, and D terms for final actuator command
    float totalPID = self->p + self->i + self->d;
    // Final global output limiting
    self->pid = constrainToRangeF(totalPID, self->limitMin, self->limitMax);
}
