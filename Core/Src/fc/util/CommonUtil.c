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
