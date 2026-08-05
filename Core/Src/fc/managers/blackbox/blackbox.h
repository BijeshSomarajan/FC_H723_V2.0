#ifndef FC_CORE_BLACKBOX_BLACKBOX_H_
#define FC_CORE_BLACKBOX_BLACKBOX_H_

#include <sys/_stdint.h>


uint8_t initBlackbox(void);
uint8_t isBlackBoxAvailable(void);
uint8_t startBlackboxSession(void);
uint32_t recordToBlackbox(char* data, uint16_t len);
uint8_t endBlackboxSession(void);

#define BLACK_BOX_MAX_FILES 9999
#define BLACK_BOX_ENABLED 1

#endif /* FC_CORE_BLACKBOX_BLACKBOX_H_ */
