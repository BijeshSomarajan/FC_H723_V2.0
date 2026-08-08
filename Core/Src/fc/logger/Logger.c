#include <string.h>
#include "../io/uart/UART.h"
#include "Logger.h"

uint8_t initLogger() {
	uint8_t status = uart5Init();
	if (status) {
		logString("Logger initialized.\n");
	}
	return status;
}

void logString(char *data) {
	uart5Write((uint8_t*) data, strlen(data));
	//uart5WriteDMA((uint8_t*) data, strlen(data));
}

void logBytes(uint8_t *data,uint16_t len) {
	uart5Write((uint8_t*) data, len);
	//uart5WriteDMA((uint8_t*) data, strlen(data));
}
