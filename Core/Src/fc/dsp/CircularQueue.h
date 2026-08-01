#ifndef CIRCULAR_QUEUE_H
#define CIRCULAR_QUEUE_H

#include <stdint.h>

/*
 * Single-producer / single-consumer (SPSC) lock-free byte ring.
 *
 * Safe for exactly ONE producer context and ONE consumer context running
 * concurrently with NO locks and NO interrupt disabling -- e.g. a DMA/UART
 * interrupt calling circularQueueWrite() while the main loop calls
 * circularQueueRead(). Two producers or two consumers are NOT safe.
 *
 *   Producer : circularQueueWrite(), circularQueueFree()
 *   Consumer : circularQueueRead(),  circularQueueAvailableData()
 *
 * head is written only by the consumer, tail only by the producer; each side
 * only reads the other's index. Aligned 16-bit loads/stores are atomic on
 * Cortex-M, which is what makes this correct on a single core with no counter
 * shared between the two sides.
 *
 * Contract matches the original: init by size, partial-fill write, any size.
 * Requesting N bytes gives N usable bytes (one extra slot is allocated
 * internally to distinguish full from empty).
 */
typedef struct {
    uint8_t          *buffer;
    uint16_t          size;   /* physical size = requested + 1        */
    volatile uint16_t head;   /* read index  (owned by consumer)      */
    volatile uint16_t tail;   /* write index (owned by producer)      */
} CircularQueue;

/* Allocates (size + 1) bytes so usable capacity == size. On alloc failure or
 * size == 0 the queue is left inert (all ops return 0). Call once at startup. */
void     circularQueueInit(CircularQueue *q, uint16_t size);

/* Producer. Partial fill: writes as many of len bytes as fit, returns the
 * count actually written (0..len). A short return means the ring was near
 * full -- the remainder was dropped. */
uint16_t circularQueueWrite(CircularQueue *q, const uint8_t *data, uint16_t len);

/* Consumer. Reads up to len bytes, returns count actually read (0..len). */
uint16_t circularQueueRead(CircularQueue *q, uint8_t *data, uint16_t len);

/* Consumer-side snapshot of readable bytes. */
uint16_t circularQueueAvailableData(CircularQueue *q);

/* Producer-side snapshot of writable bytes. */
uint16_t circularQueueFree(CircularQueue *q);

#endif /* CIRCULAR_QUEUE_H */
