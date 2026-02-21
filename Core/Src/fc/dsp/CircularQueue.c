#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "CircularQueue.h"

void circularQueueInit(CircularQueue *q, uint16_t size) {
	if (!q) {
		return;
	}
	if (size == 0) {
		q->buffer = NULL;
		q->size = 0;
		q->head = 0;
		q->tail = 0;
		q->count = 0;
		return;
	}
	q->buffer = (uint8_t*) malloc(size);
	if (!q->buffer) {
		q->size = 0;
		q->head = 0;
		q->tail = 0;
		q->count = 0;
		return;
	}
	q->size = size;
	q->head = 0;
	q->tail = 0;
	q->count = 0;
}

uint8_t circularQueueWrite(CircularQueue *q, uint8_t *data, uint16_t len) {
	if (!q || !q->buffer) {
		return 0;
	}
	uint8_t writtenCount = 0;
	for (uint8_t i = 0; i < len; i++) {
		if (q->count == q->size) {
			break; // Queue full
		}
		q->buffer[q->tail] = data[i];
		q->tail = (q->tail + 1) % q->size;
		q->count++;
		writtenCount++;
	}
	return writtenCount;
}

uint8_t circularQueueRead(CircularQueue *q, uint8_t *data, uint16_t len) {
	if (!q || !q->buffer) {
		return 0;
	}
	uint8_t readCount = 0;
	for (uint8_t i = 0; i < len; i++) {
		if (q->count == 0) {
			break; // Queue empty
		}
		data[i] = q->buffer[q->head];
		q->head = (q->head + 1) % q->size;
		q->count--;
		readCount++;
	}
	return readCount;
}

uint16_t circularQueueAvailableData(CircularQueue *q) {
	if (!q) {
		return 0;
	}
	return q->count;
}

/* Optional helper: remaining free space */

uint16_t circularQueueFree(CircularQueue *q) {
	if (!q) {
		return 0;
	}
	return q->size - q->count;
}
