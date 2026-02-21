#ifndef UTILS_MATHUTILS_H__
#define UTILS_MATHUTILS_H__

#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "../memory/Memory.h"

#define GRAVITY_MSS     9.80665f    // Gravity in m/s^2
#define GRAVITY_CMS     980.665f    // Gravity in cm/s^2

/* --- Math Constants --- */
#define HALF_PI            1.57079632679489661923f
#define ONE_PI             3.14159265358979323846f
#define TWO_PI             6.28318530717958647692f
#define PI_BY_180          0.01745329251994329577f
#define PI_BY_180_INVERSE   57.2957795130823208768f

/* --- Macros --- */
#ifndef FC_MAX
#define FC_MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef FC_MIN
#define FC_MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

/* --- Trigonometry Approximations --- */
float sinApprox(float x);
float cosApprox(float x);
float tanApprox(float x);
float atanApprox(float x);
float atan2Approx(float y, float x);
float asinApprox(float x);
float asinApproxFast(float x);
float acosApproxFast(float x);

/* --- Fast Math Kernels --- */
float fastInvSqrtf(float val);
float fastSqrtf(float x);
float invSqrtf(float x);

/* --- Conversion & Mapping --- */
float convertRadToDeg(float rads);
float convertDegToRad(float deg);
float mapToRangeFloat(float inValue, float minInRange, float maxInRange, float minOutRange, float maxOutRange);

/* --- Constraints & Deadbands --- */
int32_t constrainToRange(int32_t rawValue, int32_t minValue, int32_t maxValue);
float constrainToRangeF(float rawValue, float minValue, float maxValue);
int16_t applyDeadBandInt16(int16_t neutralValue, int16_t value, int16_t boundary);
float applyDeadBandFloat(float neutralValue, float value, float boundary);
uint8_t isInRangeF(float rawValue, float minValue, float maxValue);
uint8_t isInAbsRangeF(float rawValue, float absValue);

/* --- Bit Manipulation for Embedded Sensors --- */
int16_t shiftBitsRight(int16_t value, int16_t nBits);
int16_t shiftBitsLeft(int16_t value, int16_t nBits);
int32_t shiftBitsLeft32(int32_t value, int32_t nBits);

/* --- Power Functions --- */
float power2f(float x);
float power3f(float x);
#endif
