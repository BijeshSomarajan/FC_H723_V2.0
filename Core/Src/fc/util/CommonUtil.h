#ifndef _COMMONUTIL_H_
#define _COMMONUTIL_H_
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
float generateSCurve(float input, float sharpness) ;
void bubbleSortF(float* arr, int n);
void bubbleSortDecF(float* arr, int n);
float applyQuadraticShaper(float input, float divisor, float deadzone);
#endif
