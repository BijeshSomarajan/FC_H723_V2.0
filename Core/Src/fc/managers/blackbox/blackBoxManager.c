#include "blackBoxManager.h"

#include <stdio.h>
#include <string.h>

#include "../../logger/Logger.h"
#include "blackbox.h"

float blackBoxUpdateDt = 0;
char blackBoxRecordBuffer[256];
float blackBoxTime = 0;

uint8_t initBlackBoxManager() {
	uint8_t status = initBlackbox();
	if (!status) {
		logString("[BlackBox Manager] >> IO >> Failed\n");
	} else {
		logString("[BlackBox Manager] >> IO >> Success\n");

		status = startBlackboxSession() ;
		if(status){
			logString("[BlackBox Manager] >> Blackbox Session >> Success\n");
			updateBlackBox(0.5f);
			endBlackboxSession();
		}else{
			logString("[BlackBox Manager] >> Blackbox Session >> Failure\n");
		}

	}
	return status;

}

void updateBlackBox(float dt) {
	if (isBlackBoxAvailable()) {
		blackBoxTime += dt;
		blackBoxUpdateDt += dt;
		if (blackBoxUpdateDt >= BLACK_BOX_RECORD_PERIOD) {
			blackBoxUpdateDt = 0;
			recordToBlackbox("HelloB", 6) ;
		}
	} else {
		blackBoxTime = 0;
		blackBoxUpdateDt = 0;
	}
}

