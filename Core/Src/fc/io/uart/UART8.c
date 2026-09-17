#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_dma.h"
#include <stdio.h>
#include <string.h>
#include "UART.h"

#define UART8_BAUD_RATE   115200
__ATTR_RAM_D2 UART_RxCallback_t uart8RxCallback = NULL;
__ATTR_RAM_D2 uint8_t *uart8RxBuffer = NULL;
uint32_t uart8RxBufferLength = 0;
uint32_t uart8RxBufferLengthHalf = 0;
uint8_t uart8Initialized = 0;

void DMA2_Stream7_IRQHandler(void) {
	if (LL_DMA_IsActiveFlag_TC7(DMA2)) {
		LL_DMA_ClearFlag_TC7(DMA2);
	}
	if (LL_DMA_IsActiveFlag_TE7(DMA2)) {
		LL_DMA_ClearFlag_TE7(DMA2);
		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7);
	}
}

void DMA2_Stream6_IRQHandler(void) {
	// === Half Transfer (first half of buffer) ===
	if (LL_DMA_IsActiveFlag_HT6(DMA2)) {
		LL_DMA_ClearFlag_HT6(DMA2);
		if (uart8RxBufferLength > 0 && uart8RxBuffer != NULL) {
			SCB_InvalidateDCache_by_Addr((uint32_t*) uart8RxBuffer, uart8RxBufferLengthHalf);
			if (uart8RxCallback) {
				uart8RxCallback(uart8RxBuffer, (uint16_t) uart8RxBufferLengthHalf);
			}
		}
	}

	// === Transfer Complete (second half of buffer) ===
	if (LL_DMA_IsActiveFlag_TC6(DMA2)) {
		LL_DMA_ClearFlag_TC6(DMA2);
		if (uart8RxBufferLength && uart8RxBuffer) {
			uint8_t *second_half = uart8RxBuffer + uart8RxBufferLengthHalf;
			// Invalidate D-Cache for second half
			SCB_InvalidateDCache_by_Addr((uint32_t*) second_half, uart8RxBufferLengthHalf);
			// Notify application with second half
			if (uart8RxCallback) {
				uart8RxCallback(second_half, (uint16_t) uart8RxBufferLengthHalf);
			}
		}
	}
	// === Transfer Error ===
	if (LL_DMA_IsActiveFlag_TE6(DMA2)) {
		LL_DMA_ClearFlag_TE6(DMA2);
	}
	// === Direct Mode Error ===
	if (LL_DMA_IsActiveFlag_DME6(DMA2)) {
		LL_DMA_ClearFlag_DME6(DMA2);
	}
	// === FIFO Error ===
	if (LL_DMA_IsActiveFlag_FE6(DMA2)) {
		LL_DMA_ClearFlag_FE6(DMA2);
	}
}

void uart8DMAConfigTX() {
	// Ensure stream is disabled before config
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7);
	while (LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_7)) {
	}
	// Clear all flags
	LL_DMA_ClearFlag_TC7(DMA2);
	LL_DMA_ClearFlag_TE7(DMA2);
	LL_DMA_ClearFlag_DME7(DMA2);
	LL_DMA_ClearFlag_FE7(DMA2);
	// UART8_TX Init
	LL_DMA_SetPeriphRequest(DMA2, LL_DMA_STREAM_7, LL_DMAMUX1_REQ_UART8_TX);
	LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_7, LL_DMA_PRIORITY_LOW);
	LL_DMA_SetMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MODE_NORMAL);

	LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_7, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_7, LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_7);

	// Enable transfer complete interrupt
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_7);
	LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_7);
	// DMA2_Stream7_IRQn interrupt configuration
	NVIC_SetPriority(DMA2_Stream7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 0));
	NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

void uart8DMAConfigRX() {
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_6);
	while (LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_6)) {
	}
	LL_DMA_ClearFlag_TC6(DMA2);
	LL_DMA_ClearFlag_HT6(DMA2);
	LL_DMA_ClearFlag_TE6(DMA2);
	LL_DMA_ClearFlag_DME6(DMA2);
	LL_DMA_ClearFlag_FE6(DMA2);

	/* UART8_RX Init */
	LL_DMA_SetPeriphRequest(DMA2, LL_DMA_STREAM_6, LL_DMAMUX1_REQ_UART8_RX);
	LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_6, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_6, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMode(DMA2, LL_DMA_STREAM_6, LL_DMA_MODE_CIRCULAR);

	LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_6, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_6, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_6, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_6, LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_6);

	LL_DMA_EnableIT_HT(DMA2, LL_DMA_STREAM_6);
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_6);

	NVIC_SetPriority(DMA2_Stream6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
	NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

void uart8DMAConfig(void) {
	/* DMA controller clock enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
	uart8DMAConfigTX();
	uart8DMAConfigRX();
	// Link DMA and Enable
	LL_USART_EnableDMAReq_TX(UART8);
}

/**
 * @brief Initializes the UART8 peripheral.
 */

void uart8Config() {
	LL_USART_InitTypeDef UART_InitStruct = { 0 };
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	LL_RCC_SetUSARTClockSource( LL_RCC_USART234578_CLKSOURCE_PCLK1);
	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART8);
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE);
	/**
	 * UART8 GPIO Configuration
	 *
	 * PE0   ------> UART8_RX
	 * PE1   ------> UART8_TX
	 */

	GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
	LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	UART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
	UART_InitStruct.BaudRate = UART8_BAUD_RATE;
	UART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	UART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	UART_InitStruct.Parity = LL_USART_PARITY_NONE;
	UART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	UART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	UART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;

	LL_USART_Init(UART8, &UART_InitStruct);
	LL_USART_DisableFIFO(UART8);

	LL_USART_SetTXFIFOThreshold(UART8, LL_USART_FIFOTHRESHOLD_1_8);
	LL_USART_SetRXFIFOThreshold(UART8, LL_USART_FIFOTHRESHOLD_1_8);
	LL_USART_ConfigAsyncMode(UART8);
}

uint8_t uart8Init() {

	if (!uart8Initialized) {
		uart8Config();
		uart8DMAConfig();
		// Enable UART8
		LL_USART_Enable(UART8);
		/* Wait for UART8 initialization */
		while ((!LL_USART_IsActiveFlag_TEACK(UART8)) || (!LL_USART_IsActiveFlag_REACK(UART8))) {
		}
		uart8Initialized = 1;
	}
	return uart8Initialized;
}

void uart8WriteDMA(uint8_t *data, uint16_t len) {
	// Make DMA see latest buffer content
	SCB_CleanDCache_by_Addr((uint32_t*) data, len);
	// Disable DMA before updating
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7);
	while (LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_7)) {
	}
	// Configure memory and peripheral addresses
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_7, (uint32_t) data, LL_USART_DMA_GetRegAddr(UART8, LL_USART_DMA_REG_DATA_TRANSMIT), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, len);
	// Enable stream to start transmission
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);

}

void uart8Write(uint8_t *data, uint16_t len) {
	for (uint16_t i = 0; i < len; i++) {
		// Wait until TXE (Transmit Data Register Empty) is set
		while (!LL_USART_IsActiveFlag_TXE(UART8)) {
		}
		// Send one byte
		LL_USART_TransmitData8(UART8, data[i]);
	}
	// Wait until TC (Transmission Complete) is set
	while (!LL_USART_IsActiveFlag_TC(UART8)) {
	}
}

uint8_t uart8ReadStart(uint8_t *data, uint32_t len, UART_RxCallback_t callback) {
	if (uart8Initialized) {
		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_6);
		uart8RxCallback = callback;
		uart8RxBuffer = data;
		uart8RxBufferLength = len;
		uart8RxBufferLengthHalf = uart8RxBufferLength / 2;
		while (LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_6)) {
		}
		// Clear all flags for stream6
		LL_DMA_ClearFlag_TC6(DMA2);
		LL_DMA_ClearFlag_HT6(DMA2);
		LL_DMA_ClearFlag_TE6(DMA2);
		LL_DMA_ClearFlag_DME6(DMA2);
		LL_DMA_ClearFlag_FE6(DMA2);

		LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_6, LL_USART_DMA_GetRegAddr(UART8, LL_USART_DMA_REG_DATA_RECEIVE), (uint32_t) data, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
		LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_6, len);
		SCB_InvalidateDCache_by_Addr((uint32_t*) data, len);
		// Ensure memory ops complete before enabling DMA
		__DSB();
		__ISB();
		// Enable stream to start reception
		LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_6);
		// Clear all USART RX error conditions
		LL_USART_ClearFlag_ORE(UART8);
		LL_USART_ClearFlag_FE(UART8);
		LL_USART_ClearFlag_NE(UART8);
		LL_USART_ClearFlag_PE(UART8);
		// Flush the FIFO (recommended)
		LL_USART_RequestRxDataFlush(UART8);
		LL_USART_EnableDMAReq_RX(UART8);
		return 1;
	}
	return 0;

}

