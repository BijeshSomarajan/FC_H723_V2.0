#ifndef CORE_COMMON_LOGGER_H_
#define CORE_COMMON_LOGGER_H_

#include <stdio.h>
#include <inttypes.h>

uint8_t initLogger();
void logString(char *data);
void logBytes(uint8_t *data,uint16_t len);
#endif
