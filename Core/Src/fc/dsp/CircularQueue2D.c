#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "CircularQueue2D.h"

void circularQueue2DInit(CircularQueue2D *q, uint8_t *queueBuffer, uint16_t columnSize, uint16_t capacity) {
	if (!q) {
		return;
	}
	if (capacity == 0 || columnSize == 0 || !queueBuffer) {
		q->buffer = NULL;
		q->columnSize = 0;
		q->capacity = 0;
		q->head = q->tail = q->count = 0;
		return;
	}
	q->buffer = queueBuffer;
	q->columnSize = columnSize;
	q->capacity = capacity;
	q->head = 0;
	q->tail = 0;
	q->count = 0;
}

uint8_t circularQueue2DWrite(CircularQueue2D *q, const uint8_t *data) {
	if (!q || !q->buffer) {
		return 0;
	}
	if (q->count == q->capacity) {
		return 0;
	}
	uint8_t *write_ptr = q->buffer + (q->tail * q->columnSize);
	memcpy(write_ptr, data, q->columnSize);
	q->tail = (q->tail + 1) % q->capacity;
	q->count++;
	return 1;
}

uint8_t circularQueue2DRead(CircularQueue2D *q, uint8_t *data) {
	if (!q || !q->buffer) {
		return 0;
	}
	if (q->count == 0) {
		return 0;
	}
	uint8_t *read_ptr = q->buffer + (q->head * q->columnSize);
	memcpy(data, read_ptr, q->columnSize);
	q->head = (q->head + 1) % q->capacity;
	q->count--;
	return 1;
}

uint8_t circularQueue2DHasDataAvailable(const CircularQueue2D *q) {
	if (!q) {
		return 0;
	}
	return q->count > 0;
}
