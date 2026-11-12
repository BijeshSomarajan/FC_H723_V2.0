#ifndef FC_CORE_BLACKBOX_BLACKBOX_H_
#define FC_CORE_BLACKBOX_BLACKBOX_H_

#include <sys/_stdint.h>


uint8_t initBlackbox(void);
uint8_t isBlackBoxAvailable(void);
uint8_t startBlackboxSession(void);
uint32_t recordToBlackbox(const void* data, uint16_t len); /* queues only, never touches the card */
void processBlackbox(void);                                /* drains the queue to the card; call from the main loop */
uint8_t endBlackboxSession(void);
uint8_t syncBlackbox(void);                                /* writes everything queued, then f_sync */
uint32_t getBlackBoxMaxOPsMs();
uint8_t isBlackboxIdle(void);
char* getBlackBoxSessionFileName();
void printBlackboxDebug(void) ;

#define BLACK_BOX_MAX_FILES 9999
#define BLACK_BOX_RING_SIZE    2048
#define BLACK_BOX_WRITE_CHUNK   512
#define BLACK_BOX_ENABLED 1

#define BLACK_BOX_DEBUG_ENABLED 0
#endif /* FC_CORE_BLACKBOX_BLACKBOX_H_ */
