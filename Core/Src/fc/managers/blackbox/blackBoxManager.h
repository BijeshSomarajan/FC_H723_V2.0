#ifndef MANAGER_BLACKBOX_BLACKBOXMANAGER_H_
#define MANAGER_BLACKBOX_BLACKBOXMANAGER_H_

#include <sys/_stdint.h>

#define BLACK_BOX_RECORD_PERIOD (1.0f/50.0f)

uint8_t initBlackBoxManager();
void updateBlackBox(float dt);

#endif /* MANAGER_BLACKBOX_BLACKBOXMANAGER_H_ */
