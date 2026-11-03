#ifndef SRC_FC_SENSORS_RC_DEVICES_CRSF_CRSFFCCONFIG_H_
#define SRC_FC_SENSORS_RC_DEVICES_CRSF_CRSFFCCONFIG_H_
#include <sys/_stdint.h>
#include <stddef.h>

void manageCRSFFcConfigFrame(const uint8_t *payload);
#define CRSF_FC_CONFIG_FRAME_TYPE  0x7D

#define CRSF_FC_CONFIG_SEND_N_REPEATS 2
#define CRSF_FC_CONFIG_SEND_REPEAT_DELAY_MS 100

typedef void (*CRSFFcConfigCallBack)(const uint8_t *payload);
void registerCRSFFcConfigCallBack(CRSFFcConfigCallBack callback);
void sendCRSFConfigData(const int32_t *payload, uint8_t payloadLength,uint8_t cmd , uint8_t subCmd);

#endif
