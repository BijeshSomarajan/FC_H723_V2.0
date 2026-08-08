#ifndef SRC_FC_IO_UART_UART_H_
#define SRC_FC_IO_UART_UART_H_
#include <stdint.h>

#include "../../memory/Memory.h"

#define UART_BAUD_RATE_9600     9600
#define UART_BAUD_RATE_115200   115200
#define UART_BAUD_RATE_420000   420000

typedef void (*UART_RxCallback_t)(uint8_t *data, uint16_t len);

uint8_t uart4Init(uint32_t baudRate);
void uart4WriteDMA(uint8_t *data, uint16_t len);
void uart4Write(uint8_t *data, uint16_t len);
uint8_t uart4ReadStart(uint8_t *data, uint16_t len, UART_RxCallback_t callback);

uint8_t uart5Init(void);
void uart5WriteDMA(uint8_t *data, uint16_t len);
void uart5Write(uint8_t *data, uint16_t len);
uint8_t uart5ReadStart(uint8_t *data, uint32_t len, UART_RxCallback_t callback);

uint8_t uart7Init(uint32_t baudRate);
void uart7DeInit(void);
void uart7WriteDMA(uint8_t *data, uint16_t len);
void uart7Write(uint8_t *data, uint16_t len);
uint8_t uart7ReadStart(uint8_t *data, uint16_t len, UART_RxCallback_t callback);


#endif /* SRC_FC_IO_UART_UART_H_ */
