#ifndef SRC_FC_MANAGERS_INDICATOR_INDICATORMANAGER_H_
#define SRC_FC_MANAGERS_INDICATOR_INDICATORMANAGER_H_
#define INDICATOR_UPDATE_FREQUENCY 60

uint8_t initIndicatorManager();
void indicateStartUpSuccess();
void indicateStartUpFailed();
#endif
