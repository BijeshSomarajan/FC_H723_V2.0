#include "MathUtil.h"

/*
 * MathUtil.c
 *
 * Collection of lightweight math helpers and fast approximations used across
 * the flight controller. Functions here favor speed and predictable runtime
 * over perfect precision. Many approximations are polynomial/Pade and are
 * suitable for control loops where low latency is critical.
 *
 * Notes:
 * - Functions return values within the mathematically defined ranges unless
 *   the input is clamped explicitly (e.g., acos/asin approximations clamp to [-1,1]).
 * - fastSqrtf / fastInvSqrtf use bit-level tricks; they are fast but less
 *   precise than standard library sqrtf. Use only where acceptable.
 */

/* --- Internal fast sqrt struct --- */
union FAST_SQRT_DATA {
	float f;
	uint32_t i;
};

/* --- Polynomial Coefficients ---
 * Coefficients used by the approximation polynomials below. These are tuned
 * to provide good accuracy within the primary operating ranges of the flight
 * controller while keeping evaluation cheap.
 */
const float sinPolyCoef3 = -1.666568107e-1f;  // cubic term for sin approximation
const float sinPolyCoef5 = 8.312366210e-3f;   // 5th-order term
const float sinPolyCoef7 = -1.849218155e-4f;  // 7th-order term

const float atanPolyCoef1 = 0.9998660f;       // atan approximation coefficients
const float atanPolyCoef2 = -0.3302995f;
const float atanPolyCoef3 = 0.1801410f;
const float atanPolyCoef4 = -0.0851330f;
const float atanPolyCoef5 = 0.0208351f;

const float atan2PolyCoef1 = 3.14551665884836e-07f; // atan2 Pade-style coeffs
const float atan2PolyCoef2 = 0.99997356613987f;
const float atan2PolyCoef3 = 0.14744007058297684f;
const float atan2PolyCoef4 = 0.3099814292351353f;
const float atan2PolyCoef5 = 0.05030176425872175f;
const float atan2PolyCoef6 = 0.1471039133652469f;
const float atan2PolyCoef7 = 0.6444640676891548f;

/* ============================================================
 TRIG APPROXIMATIONS
 ============================================================ */
/* Fast sine approximation
 * - Input: x in radians (any range). The implementation reduces x into the
 *   principal range [-pi, pi] (and further to [-pi/2, pi/2]) for the
 *   polynomial approximation to remain accurate.
 * - Output: approximate sin(x). Error is small for typical flight angles but
 *   is not IEEE-754 accurate for all inputs.
 */
float sinApprox(float x) {
	while ( x > ONE_PI ) {
		x -= TWO_PI;
	}
	while ( x < -ONE_PI ) {
		x += TWO_PI;
	}

	if (x > HALF_PI) {
		x = ONE_PI - x;
	} else if (x < -HALF_PI) {
		x = -ONE_PI - x;
	}

	float x2 = x * x;
	return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * sinPolyCoef7));
}

/* Fast cosine approximation
 * - Implemented via a phase-shift of sinApprox for code reuse.
 */
float cosApprox(float x) {
	return sinApprox(x + HALF_PI);
}

/* Fast tangent approximation
 * - Returns sin(x)/cos(x) using the fast approximations; clamps to 0 when
 *   the cosine is very small to avoid large spikes or NaN.
 */
float tanApprox(float x) {
	float c = cosApprox(x);
	if (fabsf(c) < 1e-6f) {
		return 0.0f; // avoid division by near-zero
	}
	return sinApprox(x) / c;
}

/* atan(x) polynomial approximation with range reduction
 * - Uses symmetry and reciprocal range reduction to keep the polynomial
 *   evaluation in a small domain for accuracy.
 * - Input: any real x. Output: approximate atan(x) in radians.
 */
float atanApprox(float x) {
	float ax = fabsf(x);
	float z = (ax > 1.0f) ? (1.0f / ax) : ax;

	float z2 = z * z;
	float res = z * (atanPolyCoef1 + z2 * (atanPolyCoef2 + z2 * (atanPolyCoef3 + z2 * (atanPolyCoef4 + z2 * atanPolyCoef5))));

	if (ax > 1.0f) {
		res = HALF_PI - res;
	}
	return (x < 0.0f) ? -res : res;
}

/* atan2(y, x) using Pade approximation
 * - Fast approximation of atan2 that handles quadrants and avoids costly
 *   library calls. Suitable for angle construction in control loops.
 * - Returns angle in radians in range [-pi, pi].
 */
float atan2Approx(float y, float x) {
	float ax = fabsf(x);
	float ay = fabsf(y);

	float maxVal = FC_MAX(ax, ay);
	if (maxVal < 1e-6f) {
		return 0.0f; // both inputs nearly zero -> angle undefined, return 0
	}

	float ratio = FC_MIN(ax, ay) / maxVal;

	float res = -((((atan2PolyCoef5 * ratio - atan2PolyCoef4) * ratio - atan2PolyCoef3) * ratio - atan2PolyCoef2) * ratio - atan2PolyCoef1) / ((atan2PolyCoef7 * ratio + atan2PolyCoef6) * ratio + 1.0f);

	if (ay > ax) {
		res = HALF_PI - res;
	}
	if (x < 0.0f) {
		res = ONE_PI - res;
	}
	if (y < 0.0f) {
		res = -res;
	}

	return res;
}

/* acos approximation
 * - Input is clamped to [-1, 1] to avoid NaNs from sqrt.
 * - Uses a combination of fastSqrtf and a small polynomial for performance.
 * - Output: angle in range [0, pi].
 */
float acosApproxFast(float x) {
	if (x > 1.0f) {
		x = 1.0f;
	} else if (x < -1.0f) {
		x = -1.0f;
	}

	float ax = fabsf(x);
	float result = fastSqrtf(1.0f - ax) * (1.5707288f + ax * (-0.2121144f + ax * (0.0742610f + ax * -0.0187293f)));

	return (x < 0.0f) ? (ONE_PI - result) : result;
}

/* asin approximation
 * - Variant that uses standard sqrtf for slightly better precision. Input is
 *   clamped to [-1, 1]. Output in range [-pi/2, pi/2].
 */

float asinApprox(float x) {
	if (x > 1.0f) {
		x = 1.0f;
	} else if (x < -1.0f) {
		x = -1.0f;
	}

	float ax = fabsf(x);
	float result = sqrtf(1.0f - ax) * (1.5707288f + ax * (-0.2121144f + ax * (0.0742610f + ax * -0.0187293f)));

	return (x < 0.0f) ? (result - HALF_PI) : (HALF_PI - result);
}

/* asin approximation using fast sqrt
 * - Faster but less precise than asinApprox. Use when performance is critical.
 */

float asinApproxFast(float x) {
	if (x > 1.0f) {
		x = 1.0f;
	} else if (x < -1.0f) {
		x = -1.0f;
	}

	float ax = fabsf(x);
	float result = fastSqrtf(1.0f - ax) * (1.5707288f + ax * (-0.2121144f + ax * (0.0742610f + ax * -0.0187293f)));

	return (x < 0.0f) ? (result - HALF_PI) : (HALF_PI - result);
}

/* ============================================================
 FAST SQRT / INV SQRT
 ============================================================ */

/* fastInvSqrtf
 * - Fast inverse square root approximation (Quake III style) adapted for
 *   single-precision floats.
 * - Input: val > 0. Returns 1/sqrt(val). For non-positive inputs returns 0.
 * - Accuracy: one Newton-style iteration applied; good for control loops but
 *   not as accurate as 1.0f / sqrtf(val).
 */
float fastInvSqrtf(float val) {
	if (val <= 0.0f) {
		return 0.0f;
	}

	float xhalf = 0.5f * val;

	union FAST_SQRT_DATA conv;
	conv.f = val;
	conv.i = 0x5f3759df - (conv.i >> 1);

	conv.f = conv.f * (1.5f - xhalf * conv.f * conv.f);
	return conv.f;
}

/* fastSqrtf
 * - Fast square root implemented via fastInvSqrtf: sqrt(x) ~= x * invSqrt(x).
 * - Input: x >= 0. Returns approximate sqrt(x), 0 for non-positive inputs.
 */
float fastSqrtf(float x) {
	if (x <= 0.0f) {
		return 0.0f;
	}
	return x * fastInvSqrtf(x);
}

/* ============================================================
 RANGE / UTILITIES
 ============================================================ */

/* mapToRangeFloat
 * - Linearly maps a value from one range to another.
 * - Parameters:
 *   - inValue: input value to map
 *   - minInRange, maxInRange: input range endpoints
 *   - minOutRange, maxOutRange: output range endpoints
 * - If input range is effectively zero, returns minOutRange to avoid
 *   division by zero.
 */
float mapToRangeFloat(float inValue, float minInRange, float maxInRange, float minOutRange, float maxOutRange) {
	float range = maxInRange - minInRange;
	if (fabsf(range) < 1e-6f) {
		return minOutRange;
	}

	float ratio = (inValue - minInRange) / range;
	return minOutRange + ratio * (maxOutRange - minOutRange);
}

/* constrainToRange
 * - Constrain a signed 32-bit integer to [minValue, maxValue].
 */
int32_t constrainToRange(int32_t rawValue, int32_t minValue, int32_t maxValue) {
	if (rawValue < minValue) {
		return minValue;
	}
	if (rawValue > maxValue) {
		return maxValue;
	}
	return rawValue;
}

/* constrainToRangeF
 * - Constrain a float to [minValue, maxValue].
 */
float constrainToRangeF(float rawValue, float minValue, float maxValue) {
	if (rawValue < minValue) {
		return minValue;
	}
	if (rawValue > maxValue) {
		return maxValue;
	}
	return rawValue;
}

/* applyDeadBandInt16
 * - Applies a deadband around neutralValue for 16-bit signed integers.
 * - If the difference between value and neutralValue is smaller than
 *   boundary, returns neutralValue. Otherwise shifts the value toward the
 *   neutral by the boundary amount to remove small jitter.
 */
int16_t applyDeadBandInt16(int16_t neutralValue, int16_t value, int16_t boundary) {
	int16_t d = value - neutralValue;
	if (abs(d) >= boundary) {
		return (d < 0) ? (value + boundary) : (value - boundary);
	}
	return neutralValue;
}

/* applyDeadBandFloat
 * - Floating-point version of applyDeadBandInt16.
 */
float applyDeadBandFloat(float neutralValue, float value, float boundary) {
	float d = value - neutralValue;
	if (fabsf(d) >= boundary) {
		return (d < 0.0f) ? (value + boundary) : (value - boundary);
	}
	return neutralValue;
}

/* ============================================================
 DEG <-> RAD CONVERSION
 ============================================================ */

/* convertRadToDeg
 * - Convert radians to degrees.
 */
float convertRadToDeg(float rads) {
	return rads * PI_BY_180_INVERSE;
}

/* convertDegToRad
 * - Convert degrees to radians.
 */
float convertDegToRad(float deg) {
	return deg * PI_BY_180;
}

/* ============================================================
 BIT OPERATIONS
 ============================================================ */

/* shiftBitsRight
 * - Arithmetic right shift for 16-bit signed values that attempts to
 *   compensate by rounding toward zero for negative values.
 */
int16_t shiftBitsRight(int16_t value, int16_t nBits) {
	int16_t result = value >> nBits;
	if (value < 0) {
		result++;
	}
	return result;
}

/* shiftBitsLeft32
 * - Left shift for 32-bit integers that preserves sign without using
 *   compiler-specific arithmetic shift behavior on negatives.
 */
int32_t shiftBitsLeft32(int32_t value, int32_t nBits) {
	return (value < 0) ? -((-value) << nBits) : (value << nBits);
}

/* isInRangeF
 * - Return non-zero (1) if rawValue is within [minValue, maxValue], inclusive.
 */
uint8_t isInRangeF(float rawValue, float minValue, float maxValue) {
	return (rawValue >= minValue && rawValue <= maxValue);
}
