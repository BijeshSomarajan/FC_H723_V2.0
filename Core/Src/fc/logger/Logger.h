#ifndef CORE_COMMON_LOGGER_H_
#define CORE_COMMON_LOGGER_H_

#include <stdarg.h>
#include <stdio.h>
#include <inttypes.h>

#define LOGGER_PRINTF_BUFFER_SIZE 256
uint8_t initLogger();
void logString(char *data);
void logBytes(uint8_t *data,uint16_t len);
void logPrintf(const char *format, ...);

#endif
