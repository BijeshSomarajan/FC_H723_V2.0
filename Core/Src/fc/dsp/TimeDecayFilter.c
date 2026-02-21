#include "timeDecayFilter.h"
#include "math.h"

void initTimeDecayFilter(TimeDecayFilter *filter, float timeConstant) {
	/*
	timeConstant : In seconds, determines the filter's responsiveness. 
	A smaller tau  means the filter is more responsive and less smooth, 
	while a larger tau makes it less responsive and more smooth
	*/
	if (timeConstant <= 0.0) {
		filter->tau = 0.0; // No filtering if time constant is invalid
	} else {
		filter->tau = timeConstant;
	}
	filter->prev_output = 0.0;
}

float updateTimeDecayFilter(TimeDecayFilter *filter, float input, float dt) {
	if (filter->tau <= 0.0 || dt <= 0.0) {
		return input; // Return raw input if filter disabled or no time has passed
	}
	// Calculate alpha based on the elapsed time
	float alpha = 1.0 - expf(-dt / filter->tau);
	// Update the filtered value using the standard leaky filter formula
	float output = alpha * input + (1.0f - alpha) * filter->prev_output;
	filter->prev_output = output;
	return output;
}

void resetTimeDecayFilter(TimeDecayFilter *filter, float resetValue) {
	filter->prev_output = resetValue;
}