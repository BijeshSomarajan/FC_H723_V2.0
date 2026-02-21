#include "SPI.h"

#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_dmamux.h"
#include "stm32h7xx_ll_utils.h"
#include <string.h>

/* ---------- INTERNAL STATE ---------- */
static uint8_t spi4Initialized = 0;

/* ---------- STATIC HELPERS ---------- */
static void spi4CsInit(void);
static void spi4SelectDevice(uint8_t device);
static void spi4DeselectAllDevices(void);
static uint8_t spi4ClearDMAFlags(void);
static void spi4InitDMA(void);

#define SPI4_DMA_MAX_TRANSFER_LEN   32
/* place buffers in D2 (AXI) RAM so DMA can access them */
__ATTR_RAM_D2 static uint8_t spi4_tx_buf[SPI4_DMA_MAX_TRANSFER_LEN];
__ATTR_RAM_D2 static uint8_t spi4_rx_buf[SPI4_DMA_MAX_TRANSFER_LEN];

static spi_callback_t spi4_async_callback = NULL;
static uint16_t spi4_async_len = 0;
static volatile uint8_t spi4_async_busy = 0;
static volatile uint8_t spi4_transfer_complete = 0;

/* ------------------------------------------------ */
/*                 GPIO / CS Control               */
/* ------------------------------------------------ */
static void spi4CsInit(void) {
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE);

	LL_GPIO_InitTypeDef gpio = { 0 };
	gpio.Pin = LL_GPIO_PIN_15;
	gpio.Mode = LL_GPIO_MODE_OUTPUT;
	gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(GPIOE, &gpio);

	spi4DeselectAllDevices();
}

static void spi4SelectDevice(uint8_t device) {
	spi4DeselectAllDevices();
	if (device == FC_SPI4_DEVICE1) {
		LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_15);
	}
}

static void spi4DeselectAllDevices(void) {
	LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_15);
}

static uint8_t spi4ClearDMAFlags(void) {
	/* RX: Stream0 */
	LL_DMA_ClearFlag_TC0(DMA1);
	LL_DMA_ClearFlag_HT0(DMA1);
	LL_DMA_ClearFlag_TE0(DMA1);
	LL_DMA_ClearFlag_DME0(DMA1);
	LL_DMA_ClearFlag_FE0(DMA1);

	/* TX: Stream1 */
	LL_DMA_ClearFlag_TC1(DMA1);
	LL_DMA_ClearFlag_HT1(DMA1);
	LL_DMA_ClearFlag_TE1(DMA1);
	LL_DMA_ClearFlag_DME1(DMA1);
	LL_DMA_ClearFlag_FE1(DMA1);

	/* SPI flags */
	LL_SPI_ClearFlag_OVR(SPI4);
	LL_SPI_ClearFlag_EOT(SPI4);
	LL_SPI_ClearFlag_TXTF(SPI4);
	LL_SPI_ClearFlag_MODF(SPI4);

	/* Drain RX FIFO if any stale data present */
	volatile uint32_t dummy;
	uint32_t timeout = SPI_IO_TIMEOUT_COUNT;
	while ( LL_SPI_IsActiveFlag_RXP(SPI4) ) {
		dummy = SPI4->RXDR;
		(void) dummy;
		if (timeout-- == 0) return 0;
	}
	return 1;
}

static void spi4EnableDMAInterrupts(void) {
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_0);
	NVIC_SetPriority(DMA1_Stream0_IRQn, 2);
	NVIC_EnableIRQ(DMA1_Stream0_IRQn);
}

/* RX stream handler - this finishes async transfers */
void DMA1_Stream0_IRQHandler(void) {
	/* Transfer complete */
	if (LL_DMA_IsActiveFlag_TC0(DMA1)) {
		LL_DMA_ClearFlag_TC0(DMA1);

		/* Stop RX & TX DMA */
		LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
		LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);

		LL_SPI_DisableDMAReq_RX(SPI4);
		LL_SPI_DisableDMAReq_TX(SPI4);

		spi4DeselectAllDevices();

		/* Make CPU view of rx buffer coherent (payload length + 1 reg) */
		SCB_InvalidateDCache_by_Addr((uint32_t*) spi4_rx_buf, spi4_async_len + 1);

		if (spi4_async_callback) {
			spi4_async_callback(spi4_rx_buf + 1, spi4_async_len);
		}

		spi4_async_callback = NULL;
		spi4_async_len = 0;
		spi4_async_busy = 0;
		spi4_transfer_complete = 1;
	}

	/* Transfer error handling */
	if (LL_DMA_IsActiveFlag_TE0(DMA1)) {
		LL_DMA_ClearFlag_TE0(DMA1);

		/* Cleanup similar to TC path */
		LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
		LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
		LL_SPI_DisableDMAReq_RX(SPI4);
		LL_SPI_DisableDMAReq_TX(SPI4);

		spi4DeselectAllDevices();

		/* clear async state so caller can recover */
		spi4_async_callback = NULL;
		spi4_async_len = 0;
		spi4_async_busy = 0;
		spi4_transfer_complete = 1;
	}
}

static void spi4InitDMA(void) {
	/* clear pending flags */
	spi4ClearDMAFlags();

	/* ---- RX: DMA1 Stream0 => SPI4_RX ---- */
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
	LL_DMA_DeInit(DMA1, LL_DMA_STREAM_0);

	LL_DMAMUX_SetRequestID(DMAMUX1, LL_DMAMUX_CHANNEL_0, LL_DMAMUX1_REQ_SPI4_RX);
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_0, LL_DMAMUX1_REQ_SPI4_RX);

	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_0, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);

	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);

	LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_BYTE);

	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_0, (uint32_t) &SPI4->RXDR);

	/* ---- TX: DMA1 Stream1 => SPI4_TX ---- */
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
	LL_DMA_DeInit(DMA1, LL_DMA_STREAM_1);

	LL_DMAMUX_SetRequestID(DMAMUX1, LL_DMAMUX_CHANNEL_1, LL_DMAMUX1_REQ_SPI4_TX);
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_1, LL_DMAMUX1_REQ_SPI4_TX);

	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_1, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_1, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MODE_NORMAL);

	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);

	LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_BYTE);

	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_1, (uint32_t) &SPI4->TXDR);

	LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_0);
	LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_1);

	spi4EnableDMAInterrupts();
}

/* ------------------------------------------------ */
/*                 SPI4 Initialization              */
/* ------------------------------------------------ */

uint8_t spi4Init(void) {
	if (spi4Initialized) return 1;

	spi4CsInit();

	LL_RCC_SetSPIClockSource(LL_RCC_SPI45_CLKSOURCE_PCLK2);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI4);
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	/* Pins: PE12 SCK, PE13 MISO, PE14 MOSI */
	LL_GPIO_InitTypeDef gpio = { 0 };
	gpio.Pin = LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14;
	gpio.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio.Alternate = LL_GPIO_AF_5;
	gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOE, &gpio);

	LL_SPI_DeInit(SPI4);

	LL_SPI_InitTypeDef spi = { 0 };
	spi.TransferDirection = LL_SPI_FULL_DUPLEX;
	spi.Mode = LL_SPI_MODE_MASTER;
	spi.DataWidth = LL_SPI_DATAWIDTH_8BIT;
	spi.ClockPolarity = LL_SPI_POLARITY_LOW;
	spi.ClockPhase = LL_SPI_PHASE_1EDGE;
	spi.NSS = LL_SPI_NSS_SOFT;
	spi.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;
	spi.BitOrder = LL_SPI_MSB_FIRST;
	spi.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;

	LL_SPI_SetFIFOThreshold(SPI4, LL_SPI_FIFO_TH_01DATA);
	LL_SPI_Init(SPI4, &spi);

	spi4InitDMA();
	LL_SPI_Enable(SPI4);

	spi4Initialized = 1;
	return 1;
}

uint8_t spi4ReadRegisterAsync(uint8_t reg, uint16_t rxLen, uint8_t device, spi_callback_t callback) {
	if (spi4_async_busy) {
		return 0;
	}

	uint16_t total = rxLen + 1;

	if (total > SPI4_DMA_MAX_TRANSFER_LEN) {
		return 0;
	}

	if (!spi4ClearDMAFlags()) {
		return 0;
	}

	spi4_async_busy = 1;
	spi4_async_callback = callback;
	spi4_async_len = rxLen;

	spi4SelectDevice(device);

	// Prepare TX buffer
	memset(spi4_tx_buf, 0xFF, total);
	spi4_tx_buf[0] = reg | FC_SPI_READ_MASK;

	SCB_CleanDCache_by_Addr((uint32_t*) spi4_tx_buf, total);

	// Setup DMA
	/* Configure DMA memory addresses and lengths */
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);

	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_0, (uint32_t) spi4_rx_buf);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, total);

	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t) spi4_tx_buf);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, total);

	/* Enable streams in correct order: RX then TX (TX must generate clocks) */
	LL_SPI_EnableDMAReq_RX(SPI4);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);

	LL_SPI_EnableDMAReq_TX(SPI4);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);

	LL_SPI_StartMasterTransfer(SPI4);

	return 1;
}

uint8_t spi4ReadRegister(uint8_t regAddr, uint8_t *rxData, uint16_t rxLen, uint8_t device) {
	spi4_transfer_complete = 0;
	if (!spi4ReadRegisterAsync(regAddr, rxLen, device, NULL)) {
		return 0;
	}
	uint32_t timeout_counter = SPI_IO_TIMEOUT_COUNT;
	while ( spi4_transfer_complete == 0 ) {
		if (timeout_counter-- == 0) {
			spi4_async_busy = 0;
			return 0;
		}
	}
	uint16_t total = rxLen + 1;
	SCB_InvalidateDCache_by_Addr((uint32_t*) spi4_rx_buf, total);
	if (total <= SPI4_DMA_MAX_TRANSFER_LEN) {
		memcpy(rxData, spi4_rx_buf + 1, rxLen);
	}
	return 1;
}

uint8_t spi4WriteRegisterAsync(uint8_t regAddr, uint8_t *txData, uint16_t txLen, uint8_t device, spi_callback_t callback) {
	if (spi4_async_busy) {
		return 0;   // async in progress
	}

	uint16_t total = txLen + 1;
	if (total > SPI4_DMA_MAX_TRANSFER_LEN) {
		return 0;
	}

	if (!spi4ClearDMAFlags()) {
		return 0;
	}

	spi4_async_busy = 1;
	spi4_async_callback = callback;
	spi4_async_len = txLen;     // number of bytes user sent

	spi4SelectDevice(device);

	/* Prepare TX buffer */
	spi4_tx_buf[0] = regAddr & FC_SPI_WRITE_MASK;
	memcpy(spi4_tx_buf + 1, txData, txLen);

	SCB_CleanDCache_by_Addr((uint32_t*) spi4_tx_buf, total);

	/* Setup DMA */
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);

	/* RX dummy receive (required for full-duplex SPI) */
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_0, (uint32_t) spi4_rx_buf);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, total);

	/* TX actual data */
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t) spi4_tx_buf);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, total);

	/* Enable DMA interrupt on RX stream */
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_0);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_0);

	/* Start RX then TX to clock the bytes out */
	LL_SPI_EnableDMAReq_RX(SPI4);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);

	LL_SPI_EnableDMAReq_TX(SPI4);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);

	LL_SPI_StartMasterTransfer(SPI4);

	return 1;
}

uint8_t spi4WriteRegister(uint8_t regAddr, uint8_t *txData, uint16_t txLen, uint8_t device) {
	spi4_transfer_complete = 0;
	if (!spi4WriteRegisterAsync(regAddr, txData, txLen, device, NULL)) {
		return 0;
	}
	uint32_t timeout_counter = SPI_IO_TIMEOUT_COUNT;
	while ( spi4_transfer_complete == 0 ) {
		if (timeout_counter-- == 0) {
			spi4_async_busy = 0;
			return 0;
		}
	}
	return 1;
}
