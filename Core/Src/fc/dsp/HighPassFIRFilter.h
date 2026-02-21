#ifndef FC_FCDSP_INCLUDE_HIGHPASSFIRFILTER_H_
#define FC_FCDSP_INCLUDE_HIGHPASSFIRFILTER_H_

#include <math.h>
#include <stdint.h>

#define HPF_FIR_FILTER_LENGTH 16
typedef struct {
    float buffer[HPF_FIR_FILTER_LENGTH];
    float *coeffs; // Pointer to the filter coefficients
    uint8_t buffer_index;
} HighPassFIRFilter;

void highPassFIRFilterInit(HighPassFIRFilter *fir);
float highpassFIRFilterUpdate(HighPassFIRFilter *fir, float input);

#endif /* FC_FCDSP_INCLUDE_HIGHPASSFIRFILTER_H_ */
