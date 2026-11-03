#ifndef SRC_FC_MANAGERS_CONFIG_CONFIGHELPERDESKTOPAPP_H_
#define SRC_FC_MANAGERS_CONFIG_CONFIGHELPERDESKTOPAPP_H_

#include <stdint.h>
#include "../../calibration/Calibration.h"
#include "ConfigHelperCommon.h"

uint8_t initConfigHelper(void);
void sendConfigDataToDektopApp(int32_t *data, int32_t length, int32_t cmd, int32_t subCmd);
void mergeConfigDataFromDesktopApp(int32_t *data, ConfigDataPacket *pkt) ;
#endif
