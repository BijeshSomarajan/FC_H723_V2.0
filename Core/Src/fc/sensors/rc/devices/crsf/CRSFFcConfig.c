#include "CRSFFcConfig.h"
CRSFFcConfigCallBack fcConfigCallbackFun;

void registerCRSFFcConfigCallBack(CRSFFcConfigCallBack callback) {
	fcConfigCallbackFun = callback;
}

void manageCRSFFcConfigFrame(const uint8_t *payload) {
	if (payload == NULL) {
		return;
	}
	// payload[0..2] = destination, origin, sequence
	uint8_t configCommandType = payload[3];
	uint8_t configCommand = payload[4];
    if(fcConfigCallbackFun!= NULL){
    	fcConfigCallbackFun(configCommandType,configCommand);
    }
}

