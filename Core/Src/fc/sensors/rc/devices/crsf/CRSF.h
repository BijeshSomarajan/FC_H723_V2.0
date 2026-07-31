#ifndef SRC_FC_SENSORS_RC_DEVICES_CRSF_H_
#define SRC_FC_SENSORS_RC_DEVICES_CRSF_H_

#include <math.h>
#include <stdint.h>
#include <inttypes.h>

#define CRSF_CHANNEL_COUNT 10
#define CRSF_CHANNEL_MIN_VALUE 1000
#define CRSF_CHANNEL_MAX_VALUE 2000
#define CRSF_CHANNEL_MID_VALUE 1500
#define CRSF_MAX_MESSAGE_COUNT 500
#define CRSF_DATA_FRAME_FREQUENCY 200 //142 Hz in actual

#define CRSF_BUFFSIZE 32
#define CRSF_SYNC_BYTE 0xC8        // Target address: Flight Controller
#define CRSF_FRAME_RC_CHANNELS 0x16 // Payload frame type for RC sticks/switches
#define CRSF_MAX_FRAME_SIZE 64     // Safe margin for largest telemetry packets
#define CRSF_FRAMETYPE_MSP_RESP  0x7B

#pragma pack(push, 1)
typedef struct {
	unsigned int ch0 :11;
	unsigned int ch1 :11;
	unsigned int ch2 :11;
	unsigned int ch3 :11;
	unsigned int ch4 :11;
	unsigned int ch5 :11;
	unsigned int ch6 :11;
	unsigned int ch7 :11;
	unsigned int ch8 :11;
	unsigned int ch9 :11;
	unsigned int ch10 :11;
	unsigned int ch11 :11;
	unsigned int ch12 :11;
	unsigned int ch13 :11;
	unsigned int ch14 :11;
	unsigned int ch15 :11;
} CRSF_CHANNELS;

typedef struct {
	uint8_t device_address;
	uint8_t frame_length;
	uint8_t frame_type; // Will be 0x16
	CRSF_CHANNELS channels;
	uint8_t crc;
} CRSF_FRAME;

#pragma pack(pop)

uint8_t initCRSF(void);
uint16_t getCRSFChannelValue(uint8_t channel);
void setCRSFChannelValue(uint8_t channel, uint16_t value);
uint8_t readCRSF(void);
void resetCRSFState(void);
uint8_t isCRSFActive(void);
uint16_t getCRSFFrameRate(void);



#endif /* SRC_FC_SENSORS_RC_DEVICES_CRSF_H_ */
