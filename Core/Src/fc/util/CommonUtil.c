#include "CommonUtil.h"
#include "../memory/Memory.h"
#include "MathUtil.h"

void bubbleSortF(float *arr, int n) {
	int i, j, temp;
	for (i = 0; i < n - 1; i++) {
		for (j = 0; j < n - i - 1; j++) {
			if (arr[j] > arr[j + 1]) {
				temp = arr[j];
				arr[j] = arr[j + 1];
				arr[j + 1] = temp;
			}
		}
	}
}

void bubbleSortDecF(float *arr, int n) {
	int i, j;
	float temp;
	for (i = 0; i < n - 1; i++) {
		for (j = 0; j < n - i - 1; j++) {
			if (arr[j] < arr[j + 1]) { // Change the comparison to sort in descending order
				temp = arr[j];
				arr[j] = arr[j + 1];
				arr[j + 1] = temp;
			}
		}
	}
}


/**
 * @brief Fast S-Curve utility without powf()
 * @param input: Normalized input (0.0 to 1.0)
 * @param sharpness: Curvature intensity (0.0 to 1.0)
 */
__ATTR_ITCM_TEXT
float generateSCurve(float input, float sharpness) {
    input = constrainToRangeF(input, 0.0f, 1.0f);
    sharpness = constrainToRangeF(sharpness, 0.0f, 1.0f);

    // 1. Calculate the Cubic Smoothstep (The "S" shape)
    // Formula: 3x^2 - 2x^3
    float x2 = input * input;
    float smooth = x2 * (3.0f - 2.0f * input);

    // 2. Linearly interpolate between the linear input and the smooth output
    // This allows the "sharpness" parameter to mix the curve intensity
    // Result = input + sharpness * (smooth - input)
    return input + sharpness * (smooth - input);
}


/**
 * @brief Applies a quadratic (y^2/S) curve to an input with an integrated noise gate.
 * Use this to smooth out "jittery" sensors or create non-linear responses.
 * * @param input     The raw value to be shaped (y).
 * @param divisor   The stability factor (S). Higher values "flatten" the curve.
 * @param deadzone  The threshold below which the output is forced to 0.0f.
 * @return float    The shaped value (d2). Always returns positive or zero.
 */
float applyQuadraticShaper(float input, float divisor, float deadzone) {
    float absInput = fabsf(input);

    // 1. Noise Gate (The "Anti-Rumble" check)
    if (absInput < deadzone) {
        return 0.0f;
    }

    // 2. The requested d2 logic: (y * y) / S
    // We use a safety check to prevent division by zero
    float sSafe = fmaxf(divisor, 0.0001f);
    float d2 = (input * input) / sSafe;

    return d2;
}
