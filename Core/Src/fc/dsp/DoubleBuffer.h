#ifndef SRC_FC_DSP_DOUBLEBUFFER_H_
#define SRC_FC_DSP_DOUBLEBUFFER_H_
#include <stdint.h>
#include <string.h>

/* ------------------------------------------------------------------
 * Double Buffer Structure
 * ------------------------------------------------------------------ */
typedef struct {
	uint8_t *buffer;        // Memory: must be 2 * elemSize
	uint16_t elemSize;
	volatile uint8_t readIndex;   // Reader sees this
	uint8_t writeIndex;           // Writer uses this
} DoubleBuffer;

void doubleBufferInit(DoubleBuffer *db, void *mem, uint16_t elemSize);
void doubleBufferWrite(DoubleBuffer *db, const void *data);
void doubleBufferRead(DoubleBuffer *db, void *out);
void* doubleBufferGetReadPtr(DoubleBuffer *db);

#endif /* SRC_FC_DSP_DOUBLEBUFFER_H_ */
