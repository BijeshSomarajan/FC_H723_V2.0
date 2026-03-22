#ifndef SRC_FC_SENSORS_POSITION_GNSS_H_
#define SRC_FC_SENSORS_POSITION_GNSS_H_
#include <sys/_stdint.h>

#define GPS_MSG_TRANSMIT_DELAY 50
#define GPS_MSG_COUNT_MAX 1000

typedef struct _GNSS_DATA GNSS_DATA;
struct _GNSS_DATA {
	double latitude;
	double longitude;
	float velN;
	float velE;
	float velD;
	float altMts;
	float hAccMts;
	uint8_t satCount;
	uint8_t fixStatus;
	uint16_t msgCount;
	float updateDt;
};

extern GNSS_DATA gnssData;

uint8_t initGNSS(void);
void resetGNSS();
uint8_t readGNSSData(void);

#endif
