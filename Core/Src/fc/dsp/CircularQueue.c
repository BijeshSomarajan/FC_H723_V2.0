#include <stdlib.h>
#include <string.h>
#include "CircularQueue.h"

/*
 * Memory barrier: real DMB on ARM (orders the byte copy against the index
 * publish across the ISR/loop boundary); compiler barrier off-target so this
 * still builds and tests on a host. Named to avoid shadowing CMSIS __DMB().
 */
#if defined(__ARM_ARCH)
  #define CQ_BARRIER() __asm volatile("dmb sy" ::: "memory")
#else
  #define CQ_BARRIER() __asm volatile("" ::: "memory")
#endif

/* Readable bytes, computed without an unsigned underflow so ANY size works
 * (no power-of-two / mask requirement). tail and head are snapshots. */
static uint16_t cqAvail(uint16_t tail, uint16_t head, uint16_t size) {
    if (tail >= head) {
        return (uint16_t) (tail - head);
    }
    return (uint16_t) (size - head + tail);
}

void circularQueueInit(CircularQueue *q, uint16_t size) {
    if (!q) {
        return;
    }
    q->buffer = NULL;
    q->size   = 0;
    q->head   = 0;
    q->tail   = 0;
    if (size == 0u || size > 65534u) {   /* +1 slot must fit in uint16_t */
        return;
    }
    q->buffer = (uint8_t*) malloc((size_t) size + 1u);
    if (!q->buffer) {
        return;                          /* stays inert */
    }
    q->size = (uint16_t) (size + 1u);    /* physical size; usable == size */
}

uint16_t circularQueueWrite(CircularQueue *q, const uint8_t *data, uint16_t len) {
    if (!q || !q->buffer || len == 0u) {
        return 0;
    }

    uint16_t head = q->head;                        /* consumer's index */
    uint16_t tail = q->tail;                         /* our index        */
    uint16_t avail = cqAvail(tail, head, q->size);
    uint16_t freeBytes = (uint16_t) (q->size - 1u - avail);

    if (len > freeBytes) {
        len = freeBytes;                             /* partial fill: clamp */
    }
    if (len == 0u) {
        return 0;
    }

    uint16_t firstPart = (uint16_t) (q->size - tail);   /* bytes to buffer end */
    if (firstPart > len) {
        firstPart = len;
    }
    memcpy(&q->buffer[tail], data, firstPart);
    if (len > firstPart) {
        memcpy(&q->buffer[0], data + firstPart, (size_t) (len - firstPart));
    }

    CQ_BARRIER();                                    /* data lands before publish */
    uint32_t nt = (uint32_t) tail + len;             /* < 2*size, no u16 overflow */
    if (nt >= q->size) {
        nt -= q->size;
    }
    q->tail = (uint16_t) nt;
    return len;
}

uint16_t circularQueueRead(CircularQueue *q, uint8_t *data, uint16_t len) {
    if (!q || !q->buffer || len == 0u) {
        return 0;
    }

    uint16_t tail = q->tail;                          /* producer's index */
    uint16_t head = q->head;                           /* our index        */
    uint16_t avail = cqAvail(tail, head, q->size);

    if (len > avail) {
        len = avail;
    }
    if (len == 0u) {
        return 0;
    }

    CQ_BARRIER();                                      /* read tail before data */

    uint16_t firstPart = (uint16_t) (q->size - head);
    if (firstPart > len) {
        firstPart = len;
    }
    memcpy(data, &q->buffer[head], firstPart);
    if (len > firstPart) {
        memcpy(data + firstPart, &q->buffer[0], (size_t) (len - firstPart));
    }

    CQ_BARRIER();                                      /* finish read before free */
    uint32_t nh = (uint32_t) head + len;
    if (nh >= q->size) {
        nh -= q->size;
    }
    q->head = (uint16_t) nh;
    return len;
}

uint16_t circularQueueAvailableData(CircularQueue *q) {
    if (!q || !q->buffer) {
        return 0;
    }
    uint16_t tail = q->tail, head = q->head;
    return cqAvail(tail, head, q->size);
}

uint16_t circularQueueFree(CircularQueue *q) {
    if (!q || !q->buffer) {
        return 0;
    }
    uint16_t tail = q->tail, head = q->head;
    return (uint16_t) (q->size - 1u - cqAvail(tail, head, q->size));
}
