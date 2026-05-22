#include "DoubleBuffer.h"
#include <sys/_stdint.h>

/* ------------------------------------------------------------------
 * Init
 * ------------------------------------------------------------------
 */
void doubleBufferInit(DoubleBuffer *db, void *mem, uint16_t elemSize) {
    if (!db || !mem || elemSize == 0) {
        return;
    }
    db->buffer = (uint8_t *)mem;
    db->elemSize = elemSize;

    db->readIndex  = 0;
    db->writeIndex = 1;
}

void doubleBufferWrite(DoubleBuffer *db, const void *data) {
    uint8_t w = db->writeIndex;
    // Copy into inactive buffer
    memcpy(db->buffer + (w * db->elemSize), data, db->elemSize);
    // Publish (atomic on Cortex-M for uint8_t)
    db->readIndex = w;
    // Flip write buffer
    db->writeIndex = w ^ 1;
}

/* ------------------------------------------------------------------ */
/* Read (Main loop) */
/* ------------------------------------------------------------------ */
void doubleBufferRead(DoubleBuffer *db, void *out) {
    uint8_t r = db->readIndex;
    memcpy(out, db->buffer + (r * db->elemSize), db->elemSize);
}

/* ------------------------------------------------------------------ */
/* Zero-copy read (fastest) */
/* ------------------------------------------------------------------ */
void* doubleBufferGetReadPtr(DoubleBuffer *db) {
    return (void *)(db->buffer + (db->readIndex * db->elemSize));
}


