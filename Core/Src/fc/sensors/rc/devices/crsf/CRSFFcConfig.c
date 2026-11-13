#include "CRSFFcConfig.h"

#include "../../../../timers/DelayTimer.h"
#include "CRSF.h"

CRSFFcConfigCallBack fcConfigCallbackFun;
uint8_t crsfConfigSendBuf[64]; //Max size is this.

void registerCRSFFcConfigCallBack(CRSFFcConfigCallBack callback) {
	fcConfigCallbackFun = callback;
}

void manageCRSFFcConfigFrame(const uint8_t *payload) {
	if (payload == NULL) {
		return;
	}
	if (fcConfigCallbackFun != NULL) {
		fcConfigCallbackFun(payload);
	}
}

/* Packs `count` int32_t values into `out` as big-endian bytes.
 * Returns the number of bytes written, or 0 if `out` is too small. */
void packInt32ArrayBE(const int32_t *in, uint8_t count, uint8_t *out) {
	uint8_t *p = out;
	for (uint8_t i = 0; i < count; i++) {
		uint32_t v = (uint32_t) in[i]; /* two's complement, no sign issues */
		*p++ = (uint8_t) (v >> 24);
		*p++ = (uint8_t) (v >> 16);
		*p++ = (uint8_t) (v >> 8);
		*p++ = (uint8_t) (v);
	}
}

void sendCRSFConfigData(const int32_t *payload, uint8_t payloadLength, uint8_t cmd, uint8_t subCmd) {
	crsfConfigSendBuf[0] = cmd;
	crsfConfigSendBuf[1] = subCmd;
	packInt32ArrayBE(payload, payloadLength, &crsfConfigSendBuf[2]);
	for (uint8_t sendCount = 0; sendCount < CRSF_FC_CONFIG_SEND_N_REPEATS; sendCount++) {
		crsfSendTelemetry(CRSF_FC_CONFIG_FRAME_TYPE, crsfConfigSendBuf, (payloadLength * 4) + 2);
		delayMs(CRSF_FC_CONFIG_SEND_REPEAT_DELAY_MS);
	}
}
