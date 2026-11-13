#ifndef SRC_FC_MANAGERS_CONFIG_CONFIGHELPERMOBILEAPP_H_
#define SRC_FC_MANAGERS_CONFIG_CONFIGHELPERMOBILEAPP_H_

#include <sys/_stdint.h>
#include "ConfigHelperCommon.h"
#define CONFIG_COMMAND_TYPE_CONFIG 5

uint8_t initConfigHelperMobileApp(void);
void sendConfigDataToMobileApp(int32_t *data, int32_t length, int32_t cmd, int32_t subCmd);
void mergeConfigDataFromMobileApp(int32_t *data, ConfigDataPacket *pkt);

#endif /* SRC_FC_MANAGERS_CONFIG_CONFIGHELPERMOBILEAPP_H_ */
