#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_dma.h"
#include <stdio.h>
#include <string.h>
#include "UART.h"

#define UART4_BAUD_RATE   115200

__ATTR_RAM_D2 UART_RxCallback_t uart4RxCallback = NULL;
__ATTR_RAM_D2 uint8_t *uart4RxBuffer = NULL;

uint32_t uart4RxBufferLength = 0;
uint32_t uart4RxBufferLengthHalf = 0;
uint8_t uart4Initialized = 0;

/**
 * @brief This function handles DMA2 stream1 global interrupt.
 */
void DMA2_Stream1_IRQHandler(void) {
	if (LL_DMA_IsActiveFlag_TC1(DMA2)) {
		LL_DMA_ClearFlag_TC1(DMA2);
	}
	if (LL_DMA_IsActiveFlag_TE1(DMA2)) {
		LL_DMA_ClearFlag_TE1(DMA2);
		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_1);
	}
}

void DMA2_Stream0_IRQHandler(void) {
	// === Half Transfer (first half of buffer) ===
	if (LL_DMA_IsActiveFlag_HT0(DMA2)) {
		LL_DMA_ClearFlag_HT0(DMA2);
		if (uart4RxBufferLength && uart4RxBuffer) {
			SCB_InvalidateDCache_by_Addr((uint32_t*) uart4RxBuffer, uart4RxBufferLengthHalf);
			if (uart4RxCallback) {
				uart4RxCallback(uart4RxBuffer, (uint16_t) uart4RxBufferLengthHalf);
			}
		}
	}
	// === Transfer Complete (second half of buffer) ===
	if (LL_DMA_IsActiveFlag_TC0(DMA2)) {
		LL_DMA_ClearFlag_TC0(DMA2);
		if (uart4RxBufferLength && uart4RxBuffer) {
			uint8_t *second_half = uart4RxBuffer + uart4RxBufferLengthHalf;
			// Invalidate D-Cache for second half
			SCB_InvalidateDCache_by_Addr((uint32_t*) second_half, uart4RxBufferLengthHalf);
			// Notify application with second half
			if (uart4RxCallback) {
				uart4RxCallback(second_half, (uint16_t) uart4RxBufferLengthHalf);
			}
		}
	}
	// === Transfer Error ===
	if (LL_DMA_IsActiveFlag_TE0(DMA2)) {
		LL_DMA_ClearFlag_TE0(DMA2);
	}
	// === Direct Mode Error ===
	if (LL_DMA_IsActiveFlag_DME0(DMA2)) {
		LL_DMA_ClearFlag_DME0(DMA2);
	}
	// === FIFO Error ===
	if (LL_DMA_IsActiveFlag_FE0(DMA2)) {
		LL_DMA_ClearFlag_FE0(DMA2);
	}
}

void uart4DMAConfigTX() {
	// Ensure stream is disabled before config
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_1);
	while ( LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_1) ) {
	}
	// Clear all flags
	LL_DMA_ClearFlag_TC1(DMA2);
	LL_DMA_ClearFlag_TE1(DMA2);
	LL_DMA_ClearFlag_DME1(DMA2);
	LL_DMA_ClearFlag_FE1(DMA2);
	/* UART4_TX Init */
	LL_DMA_SetPeriphRequest(DMA2, LL_DMA_STREAM_1, LL_DMAMUX1_REQ_UART4_TX);

	LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_1, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_1, LL_DMA_PRIORITY_MEDIUM);
	LL_DMA_SetMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MODE_NORMAL);

	LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_BYTE);

	LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_1);

	// Enable transfer complete interrupt (optional)
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_1);
	LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_1);

	/* DMA2_Stream1_IRQn interrupt configuration */
	NVIC_SetPriority(DMA2_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 2));
	NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}

void uart4DMAConfigRX() {
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
	while ( LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_0) ) {
	}

	LL_DMA_ClearFlag_TC0(DMA2);
	LL_DMA_ClearFlag_HT0(DMA2);
	LL_DMA_ClearFlag_TE0(DMA2);
	LL_DMA_ClearFlag_DME0(DMA2);
	LL_DMA_ClearFlag_FE0(DMA2);

	LL_DMA_SetPeriphRequest(DMA2, LL_DMA_STREAM_0, LL_DMAMUX1_REQ_UART4_RX);

	LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_0, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MODE_CIRCULAR);

	LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_BYTE);

	LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_0);

	LL_DMA_EnableIT_HT(DMA2, LL_DMA_STREAM_0);
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);

	NVIC_SetPriority(DMA2_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 2));
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

void uart4DMAConfig(void) {
	/* DMA controller clock enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
	uart4DMAConfigTX();
	uart4DMAConfigRX();
	//Link DMA and Enable
	LL_USART_EnableDMAReq_TX(UART4);
}

/**
 * @brief Initializes the UART4 peripheral.
 */
void uart4Config() {
	LL_USART_InitTypeDef UART_InitStruct = { 0 };
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	LL_RCC_SetUSARTClockSource(LL_RCC_USART234578_CLKSOURCE_PCLK1);
	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
	/**UART4 GPIO Configuration
	 PA0   ------> UART4_TX
	 PA1   ------> UART4_RX
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	UART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
	UART_InitStruct.BaudRate = UART4_BAUD_RATE;
	UART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	UART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	UART_InitStruct.Parity = LL_USART_PARITY_NONE;
	UART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	UART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	UART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;

	LL_USART_Init(UART4, &UART_InitStruct);
	LL_USART_DisableFIFO(UART4);
	LL_USART_SetTXFIFOThreshold(UART4, LL_USART_FIFOTHRESHOLD_1_8);
	LL_USART_SetRXFIFOThreshold(UART4, LL_USART_FIFOTHRESHOLD_1_8);
	LL_USART_ConfigAsyncMode(UART4);
}

uint8_t uart4Init() {
	if (!uart4Initialized) {
		uart4Config();
		uart4DMAConfig();
		//Enable UART4
		LL_USART_Enable(UART4);
		/* Wait for UART4 initialization */
		while ( (!(LL_USART_IsActiveFlag_TEACK(UART4))) || (!(LL_USART_IsActiveFlag_REACK(UART4))) ) {
		}
		uart4Initialized = 1;
	}
	return uart4Initialized;
}

void uart4WriteDMA(uint8_t *data, uint16_t len) {
	// Make DMA see latest buffer content
	SCB_CleanDCache_by_Addr((uint32_t*) data, len);
	// Disable DMA before updating
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_1);
	while ( LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_1) ) {
	}
	// Configure memory and peripheral addresses
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_1, (uint32_t) data, LL_USART_DMA_GetRegAddr(UART4, LL_USART_DMA_REG_DATA_TRANSMIT), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_1, len);
	// Enable stream to start transmission
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);
}

void uart4Write(uint8_t *data, uint16_t len) {
	for (uint16_t i = 0; i < len; i++) {
		// Wait until TXE (Transmit Data Register Empty) is set
		while ( !LL_USART_IsActiveFlag_TXE(UART4) ) {
		}
		// Send one byte
		LL_USART_TransmitData8(UART4, data[i]);
	}
	// Wait until TC (Transmission Complete) is set
	while ( !LL_USART_IsActiveFlag_TC(UART4) ) {
	}
}

uint8_t uart4ReadStart(uint8_t *data, uint16_t len, UART_RxCallback_t callback) {
	if (uart4Initialized) {
		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
		uart4RxCallback = callback;
		uart4RxBuffer = data;
		uart4RxBufferLength = len;
		uart4RxBufferLengthHalf = uart4RxBufferLength / 2;

		while ( LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_0) ) {
		}

		// Clear all flags for stream2
		LL_DMA_ClearFlag_TC0(DMA2);
		LL_DMA_ClearFlag_HT0(DMA2);
		LL_DMA_ClearFlag_TE0(DMA2);
		LL_DMA_ClearFlag_DME0(DMA2);
		LL_DMA_ClearFlag_FE0(DMA2);

		LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_0, LL_USART_DMA_GetRegAddr(UART4, LL_USART_DMA_REG_DATA_RECEIVE), (uint32_t) data, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

		LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, len);

		SCB_InvalidateDCache_by_Addr((uint32_t*) uart4RxBuffer, len);
		// Ensure memory ops complete before enabling DMA
		__DSB();
		__ISB();

		// Enable stream to start transmission
		LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);

		// Clear all USART RX error conditions
		LL_USART_ClearFlag_ORE(UART4);
		LL_USART_ClearFlag_FE(UART4);
		LL_USART_ClearFlag_NE(UART4);
		LL_USART_ClearFlag_PE(UART4);

		// Flush the FIFO (recommended)
		LL_USART_RequestRxDataFlush(UART4);
		LL_USART_EnableDMAReq_RX(UART4);
		return 1;
	}
	return 0;
}

