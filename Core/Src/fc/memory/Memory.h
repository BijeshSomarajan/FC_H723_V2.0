#ifndef SRC_FC_IO_MEMORY_MEMORY_H_
#define SRC_FC_IO_MEMORY_MEMORY_H_
#include <stdint.h>

#define __ATTR_RAM_D2    __attribute__ ((section(".RAM_D2"), aligned(32)))
#define __ATTR_RAM_D3    __attribute__ ((section(".sram4_data"), aligned(32)))

#define TCM_ENABLED 1

#if TCM_ENABLED == 1

#define __ATTR_ITCM_TEXT __attribute__ ((section(".itcm_text"), aligned(4)))
#define __ATTR_DTCM_DATA __attribute__ ((section(".dtcm_data"))) // Initialized data
#define __ATTR_DTCM_BSS  __attribute__ ((section(".dtcm_bss")))  // Uninitialized data

#else
#define __ATTR_ITCM_TEXT
#define __ATTR_DTCM_DATA
#define __ATTR_DTCM_BSS
#endif

uint8_t initMemory(void);

#endif
