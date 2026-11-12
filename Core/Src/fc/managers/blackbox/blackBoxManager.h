#ifndef MANAGER_BLACKBOX_BLACKBOXMANAGER_H_
#define MANAGER_BLACKBOX_BLACKBOXMANAGER_H_

#include <sys/_stdint.h>

#define BLACK_BOX_RECORD_PERIOD (1.0f/100.0f)
#define BLACK_BOX_SYNC_PERIOD (1.0f/2.0f)
#define BLACK_BOX_MANAGER_DEBUG 0

uint8_t initBlackBoxManager();
void doBlackboxManagement();

#endif /* MANAGER_BLACKBOX_BLACKBOXMANAGER_H_ */
