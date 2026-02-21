#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_dma.h"
#include <stdio.h>
#include <string.h>
#include "UART.h"

#define UART5_BAUD_RATE   115200

__ATTR_RAM_D2 UART_RxCallback_t uart5RxCallback = NULL;
__ATTR_RAM_D2 uint8_t *uart5RxBuffer = NULL;

uint32_t uart5RxBufferLength = 0;
uint32_t uart5RxBufferLengthHalf = 0;
uint8_t uart5Initialized = 0;

void DMA1_Stream3_IRQHandler(void) {
	if (LL_DMA_IsActiveFlag_TC3(DMA1)) {
		LL_DMA_ClearFlag_TC3(DMA1);
	}
	if (LL_DMA_IsActiveFlag_TE3(DMA1)) {
		LL_DMA_ClearFlag_TE3(DMA1);
		LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_3);
	}
}

void DMA1_Stream2_IRQHandler(void) {
	// === Half Transfer (first half of buffer) ===
	if (LL_DMA_IsActiveFlag_HT2(DMA1)) {
		LL_DMA_ClearFlag_HT2(DMA1);
		if (uart5RxBufferLength > 0 && uart5RxBuffer != NULL) {
			SCB_InvalidateDCache_by_Addr((uint32_t*) uart5RxBuffer, uart5RxBufferLengthHalf);
			if (uart5RxCallback) {
				uart5RxCallback(uart5RxBuffer, (uint16_t) uart5RxBufferLengthHalf);
			}
		}
	}
	// === Transfer Complete (second half of buffer) ===
	if (LL_DMA_IsActiveFlag_TC2(DMA1)) {
		LL_DMA_ClearFlag_TC2(DMA1);
		if (uart5RxBufferLength && uart5RxBuffer) {
			uint8_t *second_half = uart5RxBuffer + uart5RxBufferLengthHalf;
			// Invalidate D-Cache for second half
			SCB_InvalidateDCache_by_Addr((uint32_t*) second_half, uart5RxBufferLengthHalf);
			// Notify application with second half
			if (uart5RxCallback) {
				uart5RxCallback(second_half, (uint16_t) uart5RxBufferLengthHalf);
			}
		}
	}
	// === Transfer Error ===
	if (LL_DMA_IsActiveFlag_TE2(DMA1)) {
		LL_DMA_ClearFlag_TE2(DMA1);
	}
	// === Direct Mode Error ===
	if (LL_DMA_IsActiveFlag_DME2(DMA1)) {
		LL_DMA_ClearFlag_DME2(DMA1);
	}
	// === FIFO Error ===
	if (LL_DMA_IsActiveFlag_FE2(DMA1)) {
		LL_DMA_ClearFlag_FE2(DMA1);
	}
}

void uart5DMAConfigTX() {
	// Ensure stream is disabled before config
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_3);
	while ( LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_3) ) {
	}
	// Clear all flags
	LL_DMA_ClearFlag_TC3(DMA1);
	LL_DMA_ClearFlag_TE3(DMA1);
	LL_DMA_ClearFlag_DME3(DMA1);
	LL_DMA_ClearFlag_FE3(DMA1);
	//UART5_TX Init
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_3, LL_DMAMUX1_REQ_UART5_TX);

	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_3, LL_DMA_PRIORITY_LOW);

	LL_DMA_SetMode(DMA1, LL_DMA_STREAM_3, LL_DMA_MODE_NORMAL);

	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_3, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_3, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_3, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_3, LL_DMA_MDATAALIGN_BYTE);

	LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_3);

	// Enable transfer complete interrupt (optional)
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_3);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_3);

	// DMA1_Stream3_IRQn interrupt configuration
	NVIC_SetPriority(DMA1_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 0));
	NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

void uart5DMAConfigRX() {
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_2);
	while ( LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_2) ) {
	}

	LL_DMA_ClearFlag_TC2(DMA1);
	LL_DMA_ClearFlag_HT2(DMA1);
	LL_DMA_ClearFlag_TE2(DMA1);
	LL_DMA_ClearFlag_DME2(DMA1);
	LL_DMA_ClearFlag_FE2(DMA1);

	/* UART5_RX Init */
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_2, LL_DMAMUX1_REQ_UART5_RX);

	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_2, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMode(DMA1, LL_DMA_STREAM_2, LL_DMA_MODE_CIRCULAR);

	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_BYTE);

	LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_2);

	LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_2);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_2);

	NVIC_SetPriority(DMA1_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
	NVIC_EnableIRQ(DMA1_Stream2_IRQn);
}

void uart5DMAConfig(void) {
	/* DMA controller clock enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	uart5DMAConfigTX();
	uart5DMAConfigRX();
	//Link DMA and Enable
	LL_USART_EnableDMAReq_TX(UART5);
}

/**
 * @brief Initializes the UART5 peripheral.
 */
void uart5Config() {
	LL_USART_InitTypeDef UART_InitStruct = { 0 };
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	LL_RCC_SetUSARTClockSource(LL_RCC_USART234578_CLKSOURCE_PCLK1);
	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);

	/**UART5 GPIO Configuration
	 PB12   ------> UART5_RX
	 PB13   ------> UART5_TX
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_12 | LL_GPIO_PIN_13;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_14;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	UART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
	UART_InitStruct.BaudRate = UART5_BAUD_RATE;
	UART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	UART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	UART_InitStruct.Parity = LL_USART_PARITY_NONE;
	UART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	UART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	UART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;

	LL_USART_Init(UART5, &UART_InitStruct);
	LL_USART_DisableFIFO(UART5);
	LL_USART_SetTXFIFOThreshold(UART5, LL_USART_FIFOTHRESHOLD_1_8);
	LL_USART_SetRXFIFOThreshold(UART5, LL_USART_FIFOTHRESHOLD_1_8);
	LL_USART_ConfigAsyncMode(UART5);
}

uint8_t uart5Init() {
	if (!uart5Initialized) {
		uart5Config();
		uart5DMAConfig();
		//Enable UART5
		LL_USART_Enable(UART5);
		/* Wait for UART5 initialization */
		while ( (!(LL_USART_IsActiveFlag_TEACK(UART5))) || (!(LL_USART_IsActiveFlag_REACK(UART5))) ) {
		}
		uart5Initialized = 1;
	}
	return uart5Initialized;
}

void uart5WriteDMA(uint8_t *data, uint16_t len) {
	// Make DMA see latest buffer content
	SCB_CleanDCache_by_Addr((uint32_t*) data, len);
	// Disable DMA before updating
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_3);
	while ( LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_3) ) {
	}
	// Configure memory and peripheral addresses
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_3, (uint32_t) data, LL_USART_DMA_GetRegAddr(UART5, LL_USART_DMA_REG_DATA_TRANSMIT), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_3, len);
	// Enable stream to start transmission
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_3);
}

void uart5Write(uint8_t *data, uint16_t len) {
	for (uint16_t i = 0; i < len; i++) {
		// Wait until TXE (Transmit Data Register Empty) is set
		while ( !LL_USART_IsActiveFlag_TXE(UART5) ) {
		}
		// Send one byte
		LL_USART_TransmitData8(UART5, data[i]);
	}
	// Wait until TC (Transmission Complete) is set
	while ( !LL_USART_IsActiveFlag_TC(UART5) ) {
	}
}

uint8_t uart5ReadStart(uint8_t *data, uint32_t len, UART_RxCallback_t callback) {
	if (uart5Initialized) {
		LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_2);
		uart5RxCallback = callback;
		uart5RxBuffer = data;
		uart5RxBufferLength = len;
		uart5RxBufferLengthHalf = uart5RxBufferLength / 2;

		while ( LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_2) ) {
		}

		// Clear all flags for stream2
		LL_DMA_ClearFlag_TC2(DMA1);
		LL_DMA_ClearFlag_HT2(DMA1);
		LL_DMA_ClearFlag_TE2(DMA1);
		LL_DMA_ClearFlag_DME2(DMA1);
		LL_DMA_ClearFlag_FE2(DMA1);

		LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_2, LL_USART_DMA_GetRegAddr(UART5, LL_USART_DMA_REG_DATA_RECEIVE), (uint32_t) data, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

		LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_2, len);

		SCB_InvalidateDCache_by_Addr((uint32_t*) data, len);

		// Ensure memory ops complete before enabling DMA
		__DSB();
		__ISB();

		// Enable stream to start transmission
		LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_2);

		// Clear all USART RX error conditions
		LL_USART_ClearFlag_ORE(UART5);
		LL_USART_ClearFlag_FE(UART5);
		LL_USART_ClearFlag_NE(UART5);
		LL_USART_ClearFlag_PE(UART5);

		// Flush the FIFO (recommended)
		LL_USART_RequestRxDataFlush(UART5);

		LL_USART_EnableDMAReq_RX(UART5);

		return 1;
	}
	return 0;
}
