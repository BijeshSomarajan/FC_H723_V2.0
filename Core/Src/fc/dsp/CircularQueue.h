#ifndef FC_CORE_UTIL_QUEUE_H_
#define FC_CORE_UTIL_QUEUE_H_

#include <stdint.h>
#include <string.h>

typedef struct {
    uint8_t *buffer;
    uint16_t size;
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} CircularQueue;

void circularQueueInit(CircularQueue *q,  uint16_t size);
uint8_t circularQueueWrite(CircularQueue *q,  uint8_t *data, uint16_t len);
uint8_t circularQueueRead(CircularQueue *q, uint8_t *data, uint16_t len);
uint16_t circularQueueAvailableData(CircularQueue *q);
uint16_t circularQueueFreeSpace(CircularQueue *q);

#endif
