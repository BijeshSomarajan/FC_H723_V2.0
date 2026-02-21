#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_dma.h"
#include <stdio.h>
#include <string.h>
#include "UART.h"

#define UART7_BAUD_RATE   115200

__ATTR_RAM_D2 UART_RxCallback_t uart7RxCallback = NULL;
__ATTR_RAM_D2 uint8_t *uart7RxBuffer = NULL;

uint32_t uart7RxBufferLength = 0;
uint32_t uart7RxBufferLengthHalf = 0;
uint8_t uart7Initialized = 0;

/**
 * @brief This function handles DMA1 stream1 global interrupt.
 */
void DMA1_Stream5_IRQHandler(void) {
	if (LL_DMA_IsActiveFlag_TC5(DMA1)) {
		LL_DMA_ClearFlag_TC5(DMA1);
	}
	if (LL_DMA_IsActiveFlag_TE5(DMA1)) {
		LL_DMA_ClearFlag_TE5(DMA1);
	}
}

void DMA1_Stream4_IRQHandler(void) {
	// === Half Transfer (first half of buffer) ===
	if (LL_DMA_IsActiveFlag_HT4(DMA1)) {
		LL_DMA_ClearFlag_HT4(DMA1);
		if (uart7RxBufferLength > 0 && uart7RxBuffer != NULL) {
			SCB_InvalidateDCache_by_Addr((uint32_t*) uart7RxBuffer, uart7RxBufferLengthHalf);
			if (uart7RxCallback) {
				uart7RxCallback(uart7RxBuffer, (uint16_t) uart7RxBufferLengthHalf);
			}

		}
	}
	// === Transfer Complete (second half of buffer) ===
	if (LL_DMA_IsActiveFlag_TC4(DMA1)) {
		LL_DMA_ClearFlag_TC4(DMA1);
		if (uart7RxBufferLength && uart7RxBuffer) {
			uint8_t *second_half = uart7RxBuffer + uart7RxBufferLengthHalf;
			// Invalidate D-Cache for second half
			SCB_InvalidateDCache_by_Addr((uint32_t*) second_half, uart7RxBufferLengthHalf);
			// Notify application with second half
			if (uart7RxCallback) {
				uart7RxCallback(second_half, (uint16_t) uart7RxBufferLengthHalf);
			}
		}
	}
	// === Transfer Error ===
	if (LL_DMA_IsActiveFlag_TE4(DMA1)) {
		LL_DMA_ClearFlag_TE4(DMA1);
	}
	// === Direct Mode Error ===
	if (LL_DMA_IsActiveFlag_DME4(DMA1)) {
		LL_DMA_ClearFlag_DME4(DMA1);
	}
	// === FIFO Error ===
	if (LL_DMA_IsActiveFlag_FE4(DMA1)) {
		LL_DMA_ClearFlag_FE4(DMA1);
	}
}

void uart7DMAConfigTX() {
	// Ensure stream is disabled before config
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_5);
	while ( LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_5) ) {
	}
	// Clear all flags
	LL_DMA_ClearFlag_TC5(DMA1);
	LL_DMA_ClearFlag_TE5(DMA1);
	LL_DMA_ClearFlag_DME5(DMA1);
	LL_DMA_ClearFlag_FE5(DMA1);
	//uart7_TX Init
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_5, LL_DMAMUX1_REQ_UART7_TX);
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_5, LL_DMA_PRIORITY_LOW);
	LL_DMA_SetMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MODE_CIRCULAR);

	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_BYTE);

	LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_5);

	// Enable transfer complete interrupt (optional)
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_5);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_5);

	// DMA1_Stream5_IRQn interrupt configuration
	NVIC_SetPriority(DMA1_Stream5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 1));
	NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

void uart7DMAConfigRX() {
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_4);
	while ( LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_4) ) {
	}

	LL_DMA_ClearFlag_TC4(DMA1);
	LL_DMA_ClearFlag_HT4(DMA1);
	LL_DMA_ClearFlag_TE4(DMA1);
	LL_DMA_ClearFlag_DME4(DMA1);
	LL_DMA_ClearFlag_FE4(DMA1);

	/* uart7_RX Init */
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_4, LL_DMAMUX1_REQ_UART7_RX);
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_4, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_4, LL_DMA_PRIORITY_LOW);
	LL_DMA_SetMode(DMA1, LL_DMA_STREAM_4, LL_DMA_MODE_CIRCULAR);

	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_4, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_4, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_4, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_4, LL_DMA_MDATAALIGN_BYTE);

	LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_4);

	LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_4);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_4);

	/* DMA1_Stream4_IRQn interrupt configuration */
	NVIC_SetPriority(DMA1_Stream4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 1));
	NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}

void uart7DMAConfig(void) {
	/* DMA controller clock enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	uart7DMAConfigTX();
	uart7DMAConfigRX();
	//Link DMA and Enable
	LL_USART_EnableDMAReq_TX(UART7);
}

/**
 * @brief Initializes the UART7 peripheral.
 */
void uart7Config() {
	LL_USART_InitTypeDef UART_InitStruct = { 0 };
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	LL_RCC_SetUSARTClockSource(LL_RCC_USART234578_CLKSOURCE_PCLK1);
	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART7);
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE);
	/**UART7 GPIO Configuration
	 PE7   ------> UART7_RX
	 PE8   ------> UART7_TX
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_7 | LL_GPIO_PIN_8;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
	LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	UART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
	UART_InitStruct.BaudRate = 115200;
	UART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	UART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	UART_InitStruct.Parity = LL_USART_PARITY_NONE;
	UART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	UART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	UART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;

	LL_USART_Init(UART7, &UART_InitStruct);
	LL_USART_DisableFIFO(UART7);
	LL_USART_SetTXFIFOThreshold(UART7, LL_USART_FIFOTHRESHOLD_1_8);
	LL_USART_SetRXFIFOThreshold(UART7, LL_USART_FIFOTHRESHOLD_1_8);
	LL_USART_ConfigAsyncMode(UART7);
}

uint8_t uart7Init() {
	if (!uart7Initialized) {
		uart7Config();
		uart7DMAConfig();
		//Enable UART7
		LL_USART_Enable(UART7);
		/* Wait for UART7 initialization */
		while ( (!(LL_USART_IsActiveFlag_TEACK(UART7))) || (!(LL_USART_IsActiveFlag_REACK(UART7))) ) {
		}
		uart7Initialized = 1;
	}
	return uart7Initialized;
}

void uart7WriteDMA(uint8_t *data, uint16_t len) {
	// Make DMA see latest buffer content
	SCB_CleanDCache_by_Addr((uint32_t*) data, len);
	// Disable DMA before updating
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_5);
	while ( LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_5) ) {
	}
	// Configure memory and peripheral addresses
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_5, (uint32_t) data, LL_USART_DMA_GetRegAddr(UART7, LL_USART_DMA_REG_DATA_TRANSMIT), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_5, len);
	// Enable stream to start transmission
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);
}

void uart7Write(uint8_t *data, uint16_t len) {
	for (uint16_t i = 0; i < len; i++) {
		// Wait until TXE (Transmit Data Register Empty) is set
		while ( !LL_USART_IsActiveFlag_TXE(UART7) ) {
		}
		// Send one byte
		LL_USART_TransmitData8(UART7, data[i]);
	}
	// Wait until TC (Transmission Complete) is set
	while ( !LL_USART_IsActiveFlag_TC(UART7) ) {
	}
}

uint8_t uart7ReadStart(uint8_t *data, uint16_t len, UART_RxCallback_t callback) {
	if (uart7Initialized) {
		LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_4);
		uart7RxCallback = callback;
		uart7RxBuffer = data;
		uart7RxBufferLength = len;
		uart7RxBufferLengthHalf = uart7RxBufferLength / 2;
		while ( LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_4) ) {
		}

		LL_DMA_ConfigAddresses( DMA1, LL_DMA_STREAM_4, LL_USART_DMA_GetRegAddr(UART7, LL_USART_DMA_REG_DATA_RECEIVE), (uint32_t) data, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

		LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, len);

		SCB_InvalidateDCache_by_Addr((uint32_t*) data, len);
		// Ensure memory ops complete before enabling DMA
		__DSB();
		__ISB();

		// Enable stream to start transmission
		LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_4);

		// Clear all USART RX error conditions
		LL_USART_ClearFlag_ORE(UART7);
		LL_USART_ClearFlag_FE(UART7);
		LL_USART_ClearFlag_NE(UART7);
		LL_USART_ClearFlag_PE(UART7);

		// Flush the FIFO (recommended)
		LL_USART_RequestRxDataFlush(UART7);

		LL_USART_EnableDMAReq_RX(UART7);

		return 1;
	}
	return 0;
}
