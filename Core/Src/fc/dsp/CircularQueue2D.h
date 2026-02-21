#ifndef CIRCULAR_QUEUE_2D_H
#define CIRCULAR_QUEUE_2D_H

#include <stdint.h>
#include <string.h>

typedef struct {
	uint8_t *buffer;
	uint16_t columnSize;
	uint16_t capacity;
	uint16_t head;
	uint16_t tail;
	uint16_t count;
} CircularQueue2D;

void circularQueue2DInit(CircularQueue2D *q, uint8_t *queueBuffer, uint16_t columnSize, uint16_t capacity);

uint8_t circularQueue2DWrite(CircularQueue2D *q, const uint8_t *data);

uint8_t circularQueue2DRead(CircularQueue2D *q, uint8_t *data);

uint8_t circularQueue2DHasDataAvailable(const CircularQueue2D *q);

uint16_t circularQueue2DGetCount(const CircularQueue2D *q);

uint8_t circularQueue2DIsEmpty(const CircularQueue2D *q);

uint8_t circularQueue2DIsFull(const CircularQueue2D *q);

#endif /* CIRCULAR_QUEUE_2D_H */
