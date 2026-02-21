#include "main.h"
#include "Memory.h"

#if TCM_ENABLED == 1
/*
 * Linker Script Symbols for ITCM Code (.itcm_text)
 * LMA (Load Memory Address) is the source in Flash.
 * VMA (Virtual Memory Address) is the execution destination in ITCMRAM.
 */
extern uint32_t _litcm;  // Source (LMA): Start address of ITCM code in FLASH
extern uint32_t _sitcm;  // Destination (VMA): Start address of ITCM code in ITCMRAM
extern uint32_t _eitcm;  // Destination End (VMA): End address of ITCM code in ITCMRAM

/*
 * Linker Script Symbols for DTCM Initialized Data (.dtcm_data)
 */
extern uint32_t _ldtcmdata;  // Source (LMA): Start address of DTCM data in FLASH
extern uint32_t _sdtcmdata;  // Destination (VMA): Start address of DTCM data in DTCMRAM
extern uint32_t _edtcmdata;  // Destination End (VMA): End address of DTCM data in DTCMRAM

/*
 * Linker Script Symbols for DTCM Uninitialized Data (.dtcm_bss)
 * DTCM BSS only needs the VMA start and end for zero-filling.
 */
extern uint32_t _sdtcmbss;   // Destination (VMA): Start address of DTCM BSS in DTCMRAM
extern uint32_t _edtcmbss;   // Destination End (VMA): End address of DTCM BSS in DTCMRAM

void configureTCM(void) {
	uint32_t *pSource;
	uint32_t *pDest;
	uint32_t *pEnd;

	// --- 1. Copy Initialized ITCM Code (.itcm_text) ---
	pSource = &_litcm;      // Load Address (Flash)
	pDest = &_sitcm;      // Execution Address (ITCMRAM)
	pEnd = &_eitcm;      // End Address

	while ( pDest < pEnd ) {
		*pDest++ = *pSource++;
	}

	// --- 2. Copy Initialized DTCM Data (.dtcm_data) ---
	pSource = &_ldtcmdata;  // Load Address (Flash)
	pDest = &_sdtcmdata;  // Execution Address (DTCMRAM)
	pEnd = &_edtcmdata;  // End Address

	while ( pDest < pEnd ) {
		*pDest++ = *pSource++;
	}

	// --- 3. Zero-Fill Uninitialized DTCM Data (.dtcm_bss) ---
	pDest = &_sdtcmbss;     // Start Address (DTCMRAM)
	pEnd = &_edtcmbss;     // End Address

	while ( pDest < pEnd ) {
		*pDest++ = 0;
	}
}

uint8_t initMemory() {
	configureTCM();
	return 1;
}

#else

uint8_t initMemory(){
	return 1;
}

#endif
