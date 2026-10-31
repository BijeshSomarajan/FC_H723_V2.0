#ifndef SRC_FC_SENSORS_RC_DEVICES_CRSF_CRSFFCCONFIG_H_
#define SRC_FC_SENSORS_RC_DEVICES_CRSF_CRSFFCCONFIG_H_
#include <sys/_stdint.h>
#include <stddef.h>

void manageCRSFFcConfigFrame(const uint8_t *payload);
#define CRSF_FC_CONFIG_FRAME_TYPE  0x7D

typedef void (*CRSFFcConfigCallBack)(uint8_t fcConfigCommandType,uint8_t fcConfigCommand);
void registerCRSFFcConfigCallBack(CRSFFcConfigCallBack callback);

#endif
