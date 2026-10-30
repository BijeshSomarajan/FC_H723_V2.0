#ifndef _COMMONUTIL_H_
#define _COMMONUTIL_H_
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
float generateSCurve(float input, float sharpness);
void bubbleSortF(float *arr, int n);
void bubbleSortDecF(float *arr, int n);
float applyQuadraticShaper(float input, float divisor, float deadzone);
float convertPascalToHectoPascal(float pascal);
float wrapAngleTo360Scale(float angle);

uint16_t readU16BE(const uint8_t *p);
uint32_t readU32BE(const uint8_t *p);
int32_t readI32BE(const uint8_t *p);

#endif
