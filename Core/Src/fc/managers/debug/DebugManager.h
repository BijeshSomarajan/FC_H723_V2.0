#ifndef SRC_FC_MANAGERS_DEBUG_DEBUGMANAGER_H_
#define SRC_FC_MANAGERS_DEBUG_DEBUGMANAGER_H_
#include <stdint.h>

#define DEBUG_FREQUENCY 25.0f

uint8_t isDebugEnabled(void);
uint8_t initDebugManager(void);
void doDebugManagement(float dt);

#endif
