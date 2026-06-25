#ifndef SRC_FC_SENSORS_POSITION_OFLOW_H_
#define SRC_FC_SENSORS_POSITION_OFLOW_H_
#include <sys/_stdint.h>

typedef struct _OFlowData OFlowData;
struct _OFlowData {
	//Global buffer for read write operations
	uint8_t buffer[8];
	//If there was a motion
	uint8_t motion;
	//The heart beat
	uint8_t observation;
	//The deltaX and deltaY
	int16_t deltaRawX;
	int16_t deltaRawY;
	uint8_t qualRaw;

	float xRad;
	float yRad;
    float qual;
	float updateDt;
};

uint8_t initOFlow(void);
void resetOFlow(void);
uint8_t readOFlowData(void);
uint8_t loadOFlowData(void);

//Global variable
extern OFlowData oFlowData;
#endif /* SRC_FC_SENSORS_POSITION_OFLOW_H_ */
