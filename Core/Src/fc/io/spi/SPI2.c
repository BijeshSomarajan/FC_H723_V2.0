#include "SPI.h"

#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_dmamux.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_utils.h"
#include "string.h"

static uint8_t spi2Initialized = 0;

static void spi2DeSelectAllDevices(void);
static void spi2SelectDevice(uint8_t device);
static void spi2CSInit(void);
static uint8_t spi2ClearDMAFlags(void);

#define SPI2_DMA_MAX_TRANSFER_LEN  16
__ATTR_RAM_D2 uint8_t spi2_tx_buf[SPI2_DMA_MAX_TRANSFER_LEN];
__ATTR_RAM_D2 uint8_t spi2_rx_buf[SPI2_DMA_MAX_TRANSFER_LEN];

static spi_callback_t spi2_async_callback = NULL;
static uint16_t spi2_async_len = 0;

static volatile uint8_t spi2_async_busy = 0;
static volatile uint8_t spi2_transfer_complete = 0;

static void spi2CSInit(void) {
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);

	LL_GPIO_InitTypeDef gpio = { 0 };
	gpio.Pin = LL_GPIO_PIN_8;
	gpio.Mode = LL_GPIO_MODE_OUTPUT;
	gpio.Pull = LL_GPIO_PULL_UP;
	gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;

	LL_GPIO_Init(GPIOD, &gpio);

	spi2DeSelectAllDevices();
}

static void spi2SelectDevice(uint8_t device) {
	spi2DeSelectAllDevices();
	if (device == FC_SPI2_DEVICE1) {
		LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_8);
	}
}

static void spi2DeSelectAllDevices(void) {
	LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_8);
}

uint8_t spi2ClearDMAFlags(void) {
	LL_DMA_ClearFlag_TC6(DMA1);
	LL_DMA_ClearFlag_HT6(DMA1);
	LL_DMA_ClearFlag_TE6(DMA1);
	LL_DMA_ClearFlag_DME6(DMA1);
	LL_DMA_ClearFlag_FE6(DMA1);

	LL_DMA_ClearFlag_TC7(DMA1);
	LL_DMA_ClearFlag_HT7(DMA1);
	LL_DMA_ClearFlag_TE7(DMA1);
	LL_DMA_ClearFlag_DME7(DMA1);
	LL_DMA_ClearFlag_FE7(DMA1);

	LL_SPI_ClearFlag_OVR(SPI2);
	LL_SPI_ClearFlag_EOT(SPI2);
	LL_SPI_ClearFlag_TXTF(SPI2);
	LL_SPI_ClearFlag_MODF(SPI2);

	uint32_t timeout_counter = SPI_IO_TIMEOUT_COUNT;
	volatile uint32_t dummy;
	while ( LL_SPI_IsActiveFlag_RXP(SPI2) ) {
		dummy = SPI2->RXDR;
		(void) dummy;
		if (timeout_counter == 0) {
			return 0;
		}
		timeout_counter--;
	}
	return 1;
}

static void spi2EnableDMAInterrupts(void) {
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_6);
	NVIC_SetPriority(DMA1_Stream6_IRQn, 0);
	NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

void spi2InitDMA(void) {
	spi2ClearDMAFlags();

	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_6);
	LL_DMA_DeInit(DMA1, LL_DMA_STREAM_6);

	LL_DMAMUX_SetRequestID(DMAMUX1, LL_DMAMUX_CHANNEL_6, LL_DMAMUX1_REQ_SPI2_RX);
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_6, LL_DMAMUX1_REQ_SPI2_RX);

	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_6, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_6, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MODE_NORMAL);

	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MEMORY_INCREMENT);

	LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_6, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_6, LL_DMA_MDATAALIGN_BYTE);

	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_6, (uint32_t) &SPI2->RXDR);

	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_7);
	LL_DMA_DeInit(DMA1, LL_DMA_STREAM_7);

	LL_DMAMUX_SetRequestID(DMAMUX1, LL_DMAMUX_CHANNEL_7, LL_DMAMUX1_REQ_SPI2_TX);
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_7, LL_DMAMUX1_REQ_SPI2_TX);

	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_7, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMode(DMA1, LL_DMA_STREAM_7, LL_DMA_MODE_NORMAL);

	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_7, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_7, LL_DMA_MEMORY_INCREMENT);

	LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_7, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_7, LL_DMA_MDATAALIGN_BYTE);

	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_7, (uint32_t) &SPI2->TXDR);

	LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_6);
	LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_7);

	spi2EnableDMAInterrupts();
}

uint8_t spi2Init(void) {
	if (spi2Initialized) {
		return 1;
	}
	spi2CSInit();

	LL_RCC_SetSPIClockSource(LL_RCC_SPI123_CLKSOURCE_PLL1Q);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	LL_GPIO_InitTypeDef gpio = { 0 };
	gpio.Pin = LL_GPIO_PIN_10 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
	gpio.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio.Alternate = LL_GPIO_AF_5;
	gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOB, &gpio);

	LL_SPI_DeInit(SPI2);

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

	LL_SPI_SetFIFOThreshold(SPI2, LL_SPI_FIFO_TH_01DATA);

	LL_SPI_Init(SPI2, &spi);

	spi2InitDMA();

	LL_SPI_Enable(SPI2);

	spi2Initialized = 1;

	return 1;
}

void DMA1_Stream6_IRQHandler(void) {
	if (LL_DMA_IsActiveFlag_TC6(DMA1)) {
		LL_DMA_ClearFlag_TC6(DMA1);

		// Stop RX & TX DMA
		LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_6);
		LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_7);

		LL_SPI_DisableDMAReq_RX(SPI2);
		LL_SPI_DisableDMAReq_TX(SPI2);

		spi2DeSelectAllDevices();

		spi2_async_busy = 0;
		spi2_transfer_complete = 1;

		SCB_InvalidateDCache_by_Addr((uint32_t*) spi2_rx_buf, spi2_async_len + 1);

		if (spi2_async_callback) {
			spi2_async_callback(spi2_rx_buf + 1, spi2_async_len);
		}

		spi2_async_callback = NULL;
		spi2_async_len = 0;

	}
	// Transfer error handling: clear error flags and release busy state
	if (LL_DMA_IsActiveFlag_TE6(DMA1)) {
		LL_DMA_ClearFlag_TE6(DMA1);

		// cleanup similar to TC path
		LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_6);
		LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_7);
		LL_SPI_DisableDMAReq_RX(SPI2);
		LL_SPI_DisableDMAReq_TX(SPI2);
		spi2DeSelectAllDevices();

		// clear busy and callback so system can recover
		spi2_async_callback = NULL;
		spi2_async_len = 0;
		spi2_async_busy = 0;
		spi2_transfer_complete = 1;
	}
}

uint8_t spi2ReadRegisterAsync(uint8_t reg, uint16_t rxLen, uint8_t device, spi_callback_t callback) {
	if (spi2_async_busy) {
		return 0;
	}
	uint16_t total = rxLen + 1;
	if (total > SPI2_DMA_MAX_TRANSFER_LEN) {
		return 0;
	}
	if (!spi2ClearDMAFlags()) {
		return 0;
	}
	spi2_async_busy = 1;
	spi2_async_callback = callback;
	spi2_async_len = rxLen;

	spi2SelectDevice(device);

	// Prepare TX buffer
	memset(spi2_tx_buf, 0xFF, total);
	spi2_tx_buf[0] = reg | FC_SPI_READ_MASK;

	SCB_CleanDCache_by_Addr((uint32_t*) spi2_tx_buf, total);

	// Setup DMA
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_6);
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_7);

	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_6, (uint32_t) spi2_rx_buf);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_6, total);

	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_7, (uint32_t) spi2_tx_buf);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_7, total);

	LL_SPI_EnableDMAReq_RX(SPI2);
	LL_SPI_EnableDMAReq_TX(SPI2);

	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_6);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_7);

	LL_SPI_StartMasterTransfer(SPI2);

	return 1;
}

uint8_t spi2ReadRegister(uint8_t regAddr, uint8_t *rxData, uint16_t rxLen, uint8_t device) {
	spi2_transfer_complete = 0;
	if (!spi2ReadRegisterAsync(regAddr, rxLen, device, NULL)) {
		return 0;
	}
	uint32_t timeout_counter = SPI_IO_TIMEOUT_COUNT;
	while ( spi2_transfer_complete == 0 ) {
		if (timeout_counter-- == 0) {
			spi2_async_busy = 0;
			return 0;
		}
	}
	uint16_t total = rxLen + 1;
	SCB_InvalidateDCache_by_Addr((uint32_t*) spi2_rx_buf, total);
	if (total <= SPI2_DMA_MAX_TRANSFER_LEN) {
		memcpy(rxData, spi2_rx_buf + 1, rxLen);
	}
	return 1;
}

uint8_t spi2WriteRegisterAsync(uint8_t regAddr, uint8_t *txData, uint16_t txLen, uint8_t device, spi_callback_t callback) {
	if (spi2_async_busy) {
		return 0;   // async still running
	}

	uint16_t total = txLen + 1;
	if (total > SPI2_DMA_MAX_TRANSFER_LEN) {
		return 0;
	}

	if (!spi2ClearDMAFlags()) {
		return 0;
	}

	spi2_async_busy = 1;
	spi2_async_callback = callback;
	spi2_async_len = txLen;   // bytes to write (for callback)

	spi2SelectDevice(device);

	/* -------- Prepare TX buffer -------- */
	spi2_tx_buf[0] = regAddr & FC_SPI_WRITE_MASK;
	memcpy(spi2_tx_buf + 1, txData, txLen);

	SCB_CleanDCache_by_Addr((uint32_t*) spi2_tx_buf, total);

	/* -------- Setup DMA -------- */
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_6);   // RX
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_7);   // TX

	/* RX dummy buffer (required for full-duplex SPI) */
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_6, (uint32_t) spi2_rx_buf);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_6, total);

	/* TX actual buffer */
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_7, (uint32_t) spi2_tx_buf);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_7, total);

	/* Enable DMA interrupts */
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_6);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_6);

	/* -------- Start DMA (order: RX then TX) -------- */
	LL_SPI_EnableDMAReq_RX(SPI2);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_6);

	LL_SPI_EnableDMAReq_TX(SPI2);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_7);

	LL_SPI_StartMasterTransfer(SPI2);

	return 1;   // async started
}

uint8_t spi2WriteRegister(uint8_t regAddr, uint8_t *txData, uint16_t txLen, uint8_t device) {
	spi2_transfer_complete = 0;
	if (!spi2WriteRegisterAsync(regAddr, txData, txLen, device, NULL)) {
		return 0;
	}
	uint32_t timeout_counter = SPI_IO_TIMEOUT_COUNT;
	while ( spi2_transfer_complete == 0 ) {
		if (timeout_counter-- == 0) {
			spi2_async_busy = 0;
			return 0;
		}
	}
	return 1;
}
