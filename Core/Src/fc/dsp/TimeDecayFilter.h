/*
 * exponentialDecayFilter.h
 *
 *  Created on: 31-Aug-2025
 *      Author: bijes
 */

#ifndef FC_FCDSP_INCLUDE_TIMEDECAYFILTER_H_
#define FC_FCDSP_INCLUDE_TIMEDECAYFILTER_H_

// Define a structure to hold the filter state
typedef struct {
    float tau; // Filter time constant in seconds
    float prev_output;
} TimeDecayFilter;

void initTimeDecayFilter(TimeDecayFilter* filter, float timeConstant) ;
float updateTimeDecayFilter(TimeDecayFilter* filter, float input, float dt);
void resetTimeDecayFilter(TimeDecayFilter *filter, float resetValue) ;

#endif /* FC_FCDSP_INCLUDE_TIMEDECAYFILTER_H_ */
