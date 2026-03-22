#include <string.h>
#include <sys/_stdint.h>

#include "../../../dsp/CircularQueue.h"
#include "../../../io/uart/UART.h"
#include "../../../logger/Logger.h"
#include "../../../memory/Memory.h"
#include "../../../timers/DelayTimer.h"
#include "../GNSS.h"

#define UBX_RCV_BUFFER_SIZE       100
#define UBX_CIRCULAR_QUEUE_SIZE   UBX_RCV_BUFFER_SIZE * 2
#define UBX_READ_BUFFER_SIZE      UBX_RCV_BUFFER_SIZE
#define UBX_NAV_PVT_LEN           92
// Enable if ONLY NAV-PVT is processed
#define UBX_STRICT_NAV_PVT_ONLY   1

__ATTR_RAM_D2 static uint8_t ubxDataBuffer[UBX_RCV_BUFFER_SIZE];
uint8_t ubxIOReadSize = UBX_RCV_BUFFER_SIZE;
CircularQueue ubxIOQueue;
uint8_t ubxCircularQueueReadBuffer[UBX_READ_BUFFER_SIZE];
GNSS_DATA gnssData;

// --- UBX State Machine ---
typedef enum {
	IDLE, SYNC2, CLASS, ID, LEN1, LEN2, PAYLOAD, CK_A, CK_B
} UBX_STATE;

// --- Context ---
typedef struct {
	UBX_STATE state;
	uint8_t classId;
	uint8_t msgId;
	uint16_t payloadIdx;
	uint16_t targetPayloadLen;
	uint8_t ua, ub;

	uint8_t fixType;
	uint8_t numSV;
	int32_t lon;
	int32_t lat;
	int32_t height;
	uint32_t hAcc;
	int32_t velN;
	int32_t velE;
	int32_t velD;

	uint8_t temp[4];
} UBX_CONTEXT;

static UBX_CONTEXT ubxContext = { .state = IDLE };

#define UBX_BUILD_U32(p) ((uint32_t)(p)[0] | ((uint32_t)(p)[1] << 8) | ((uint32_t)(p)[2] << 16) | ((uint32_t)(p)[3] << 24))
#define UBX_BUILD_I32(p) ((int32_t)UBX_BUILD_U32(p))

// --- Parser ---
uint8_t updateUBXData(UBX_CONTEXT *p, uint8_t *buffer, uint16_t len) {
	uint8_t packet_ready = 0;
	for (uint16_t i = 0; i < len; i++) {
		uint8_t byte = buffer[i];
		switch (p->state) {
		case IDLE:
			if (byte == 0xB5) {
				p->state = SYNC2;
				p->ua = 0;
				p->ub = 0;
			}
			break;
		case SYNC2:
			p->state = (byte == 0x62) ? CLASS : (byte == 0xB5 ? SYNC2 : IDLE);
			break;
		case CLASS:
			p->ua += byte;
			p->ub += p->ua;
			p->classId = byte;
			p->state = ID;
			break;
		case ID:
			p->ua += byte;
			p->ub += p->ua;
			p->msgId = byte;
#if UBX_STRICT_NAV_PVT_ONLY
			if (p->classId != 0x01 || p->msgId != 0x07) {
				p->state = IDLE;
				break;
			}
#endif
			p->state = LEN1;
			break;
		case LEN1:
			p->ua += byte;
			p->ub += p->ua;
			p->targetPayloadLen = byte;
			p->state = LEN2;
			break;
		case LEN2:
			p->ua += byte;
			p->ub += p->ua;
			p->targetPayloadLen |= ((uint16_t) byte << 8);
			p->payloadIdx = 0;
			if (p->targetPayloadLen <= UBX_NAV_PVT_LEN) {
				p->state = (p->targetPayloadLen > 0) ? PAYLOAD : CK_A;
			} else {
				p->state = IDLE;
			}
			break;
		case PAYLOAD:
			p->ua += byte;
			p->ub += p->ua;
			if (p->classId == 0x01 && p->msgId == 0x07) {
				// Fix & Sat Count
				if (p->payloadIdx == 20) {
					p->fixType = byte;
				} else if (p->payloadIdx == 23) {
					p->numSV = byte;
				}
				// Longitude (24-27)
				else if (p->payloadIdx >= 24 && p->payloadIdx <= 27) {
					p->temp[p->payloadIdx - 24] = byte;
					if (p->payloadIdx == 27) {
						p->lon = UBX_BUILD_I32(p->temp);
					}
				}
				// Latitude (28-31)
				else if (p->payloadIdx >= 28 && p->payloadIdx <= 31) {
					p->temp[p->payloadIdx - 28] = byte;
					if (p->payloadIdx == 31) {
						p->lat = UBX_BUILD_I32(p->temp);
					}
				}
				// Height (32-35)
				else if (p->payloadIdx >= 32 && p->payloadIdx <= 35) {
					p->temp[p->payloadIdx - 32] = byte;
					if (p->payloadIdx == 35) {
						p->height = UBX_BUILD_I32(p->temp);
					}
				}
				// Horizontal Accuracy
				else if (p->payloadIdx >= 40 && p->payloadIdx <= 43) {
					p->temp[p->payloadIdx - 40] = byte;
					if (p->payloadIdx == 43) {
						p->hAcc = UBX_BUILD_U32(p->temp);
					}
				}
				// Velocity North (48-51)
				else if (p->payloadIdx >= 48 && p->payloadIdx <= 51) {
					p->temp[p->payloadIdx - 48] = byte;
					if (p->payloadIdx == 51) {
						p->velN = UBX_BUILD_I32(p->temp);
					}
				}
				// Velocity East (52-55)
				else if (p->payloadIdx >= 52 && p->payloadIdx <= 55) {
					p->temp[p->payloadIdx - 52] = byte;
					if (p->payloadIdx == 55) {
						p->velE = UBX_BUILD_I32(p->temp);
					}
				}
				// Velocity Down (56-59)
				else if (p->payloadIdx >= 56 && p->payloadIdx <= 59) {
					p->temp[p->payloadIdx - 56] = byte;
					if (p->payloadIdx == 59) {
						p->velD = UBX_BUILD_I32(p->temp);
					}
				}
			}
			p->payloadIdx++;
			if (p->payloadIdx >= p->targetPayloadLen) {
				p->state = CK_A;
			}
			break;
		case CK_A:
			if (byte == p->ua) {
				p->state = CK_B;
			} else {
				p->state = (byte == 0xB5) ? SYNC2 : IDLE;
			}
			break;
		case CK_B:
			if (byte == p->ub) {
				if (p->classId == 0x01 && p->msgId == 0x07 && p->targetPayloadLen == UBX_NAV_PVT_LEN) {
					gnssData.fixStatus = (p->fixType >= 3);
					gnssData.satCount = p->numSV;
					gnssData.latitude = p->lat * 1e-7f;
					gnssData.longitude = p->lon * 1e-7f;
					gnssData.altMts = p->height * 1e-3f;
					gnssData.hAccMts = p->hAcc * 1e-3f;
					gnssData.velN = p->velN * 1e-3f;
					gnssData.velE = p->velE * 1e-3f;
					gnssData.velD = p->velD * 1e-3f;
					gnssData.msgCount++;
					if (gnssData.msgCount > GPS_MSG_COUNT_MAX) {
						gnssData.msgCount = 0;
					}
					packet_ready = 1;
				}
			}
			p->state = (byte == 0xB5) ? SYNC2 : IDLE;
			break;
		default:
			p->state = IDLE;
			break;
		}
	}
	return packet_ready;
}

// --- DMA Callback ---
void _processUBXData(uint8_t *data, uint16_t len) {
	circularQueueWrite(&ubxIOQueue, data, len);
}

// --- Read ---
uint8_t readGNSSData(void) {
	uint16_t available = circularQueueAvailableData(&ubxIOQueue);
	if (available > 0) {
		uint16_t readLen = (available > UBX_READ_BUFFER_SIZE) ? UBX_READ_BUFFER_SIZE : available;
		readLen = circularQueueRead(&ubxIOQueue, ubxCircularQueueReadBuffer, readLen);
		return updateUBXData(&ubxContext, ubxCircularQueueReadBuffer, readLen);
	}
	return 0;
}

// --- Init ---
uint8_t initGNSS(void) {
	if (uart7Init()) {
		logString("[UBX] UART Init OK\n");
		circularQueueInit(&ubxIOQueue, UBX_CIRCULAR_QUEUE_SIZE);
		if (uart7ReadStart(ubxDataBuffer, ubxIOReadSize, _processUBXData)) {
			delayMs(GPS_MSG_TRANSMIT_DELAY);
			resetGNSS();
			logString("[UBX] RX Start OK\n");
			return 1;
		}
	}
	logString("[UBX] Init Failed\n");
	return 0;
}

// --- Reset ---
void resetGNSS(void) {
	gnssData.fixStatus = 0;
	gnssData.latitude = 0;
	gnssData.longitude = 0;
	gnssData.altMts = 0;
	gnssData.satCount = 0;
	memset(&ubxContext, 0, sizeof(UBX_CONTEXT));
	ubxContext.state = IDLE;
}

