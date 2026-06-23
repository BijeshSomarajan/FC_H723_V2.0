#include "SPI.h"

#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_dmamux.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_utils.h"
#include "string.h"

static uint8_t spi1Initialized = 0;

static void spi1DeSelectAllDevices(void);
static void spi1SelectDevice(uint8_t device);
static void spi1CSInit(void);
static uint8_t spi1ClearDMAFlags(void);

#define SPI1_DMA_MAX_TRANSFER_LEN  16
__ATTR_RAM_D2 uint8_t spi1_tx_buf[SPI1_DMA_MAX_TRANSFER_LEN];
__ATTR_RAM_D2 uint8_t spi1_rx_buf[SPI1_DMA_MAX_TRANSFER_LEN];

static spi_callback_t spi1_async_callback = NULL;
static uint16_t spi1_async_len = 0;

static volatile uint8_t spi1_async_busy = 0;
static volatile uint8_t spi1_transfer_complete = 0;

static void spi1CSInit(void) {
	// Enable clocks for both Port B (for PB5 CS) and Port D (for PD6 Flash Sleep)
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);

	LL_GPIO_InitTypeDef gpio = { 0 };

	// 1. Configure External Peripheral CS (PB5)
	gpio.Pin = LL_GPIO_PIN_5;
	gpio.Mode = LL_GPIO_MODE_OUTPUT;
	gpio.Pull = LL_GPIO_PULL_UP;
	gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO_Init(GPIOB, &gpio);

	// 2. Safely lock down Onboard Flash CS (PD6) to eliminate bus contention
	gpio.Pin = LL_GPIO_PIN_6;
	gpio.Mode = LL_GPIO_MODE_OUTPUT;
	gpio.Pull = LL_GPIO_PULL_NO;
	gpio.Speed = LL_GPIO_SPEED_FREQ_LOW;
	gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO_Init(GPIOD, &gpio);
	LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_6); // Force HIGH to sleep onboard flash

	spi1DeSelectAllDevices();
}

static void spi1SelectDevice(uint8_t device) {
	spi1DeSelectAllDevices();
	if (device == FC_SPI1_DEVICE1) {
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);
	}
}

static void spi1DeSelectAllDevices(void) {
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5);
}

uint8_t spi1ClearDMAFlags(void) {
	/* RX: DMA2 Stream 4 */
	LL_DMA_ClearFlag_TC4(DMA2);
	LL_DMA_ClearFlag_HT4(DMA2);
	LL_DMA_ClearFlag_TE4(DMA2);
	LL_DMA_ClearFlag_DME4(DMA2);
	LL_DMA_ClearFlag_FE4(DMA2);

	/* TX: DMA2 Stream 5 */
	LL_DMA_ClearFlag_TC5(DMA2);
	LL_DMA_ClearFlag_HT5(DMA2);
	LL_DMA_ClearFlag_TE5(DMA2);
	LL_DMA_ClearFlag_DME5(DMA2);
	LL_DMA_ClearFlag_FE5(DMA2);

	LL_SPI_ClearFlag_OVR(SPI1);
	LL_SPI_ClearFlag_EOT(SPI1);
	LL_SPI_ClearFlag_TXTF(SPI1);
	LL_SPI_ClearFlag_MODF(SPI1);

	uint32_t timeout_counter = SPI_IO_TIMEOUT_COUNT;
	volatile uint32_t dummy;
	while ( LL_SPI_IsActiveFlag_RXP(SPI1) ) {
		dummy = SPI1->RXDR;
		(void) dummy;
		if (timeout_counter == 0) {
			return 0;
		}
		timeout_counter--;
	}
	return 1;
}

static void spi1EnableDMAInterrupts(void) {
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_4);
	NVIC_SetPriority(DMA2_Stream4_IRQn, 0);
	NVIC_EnableIRQ(DMA2_Stream4_IRQn);
}

void spi1InitDMA(void) {
	spi1ClearDMAFlags();

	/* ---- RX: DMA2 Stream 4 (Maps to DMAMUX Channel 12) ---- */
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_4);
	LL_DMA_DeInit(DMA2, LL_DMA_STREAM_4);

	LL_DMAMUX_SetRequestID(DMAMUX1, LL_DMAMUX_CHANNEL_12, LL_DMAMUX1_REQ_SPI1_RX);
	LL_DMA_SetPeriphRequest(DMA2, LL_DMA_STREAM_4, LL_DMAMUX1_REQ_SPI1_RX);

	LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_4, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_4, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMode(DMA2, LL_DMA_STREAM_4, LL_DMA_MODE_NORMAL);

	LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_4, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_4, LL_DMA_MEMORY_INCREMENT);

	LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_4, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_4, LL_DMA_MDATAALIGN_BYTE);

	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_4, (uint32_t) &SPI1->RXDR);

	/* ---- TX: DMA2 Stream 5 (Maps to DMAMUX Channel 13) ---- */
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_5);
	LL_DMA_DeInit(DMA2, LL_DMA_STREAM_5);

	LL_DMAMUX_SetRequestID(DMAMUX1, LL_DMAMUX_CHANNEL_13, LL_DMAMUX1_REQ_SPI1_TX);
	LL_DMA_SetPeriphRequest(DMA2, LL_DMA_STREAM_5, LL_DMAMUX1_REQ_SPI1_TX);

	LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_5, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);

	LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT);

	LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_BYTE);

	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_5, (uint32_t) &SPI1->TXDR);

	LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_4);
	LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_5);

	spi1EnableDMAInterrupts();
}

uint8_t spi1Init(void) {
	if (spi1Initialized) {
		return 1;
	}
	spi1CSInit();

	// Shared SPI123 Kernel Clock Source assignment
	LL_RCC_SetSPIClockSource(LL_RCC_SPI123_CLKSOURCE_PLL1Q);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1); // SPI1 resides on the high-performance APB2 bus
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

	// Pins Config: PB3 -> SCK, PB4 -> MISO using Alternate Function 5
	LL_GPIO_InitTypeDef gpio = { 0 };
	gpio.Pin = LL_GPIO_PIN_3 | LL_GPIO_PIN_4;
	gpio.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio.Alternate = LL_GPIO_AF_5;
	gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOB, &gpio);

	// Pins Config: PD7 -> MOSI using Alternate Function 5
	gpio.Pin = LL_GPIO_PIN_7;
	LL_GPIO_Init(GPIOD, &gpio);

	LL_SPI_DeInit(SPI1);

	LL_SPI_InitTypeDef spi = { 0 };
	spi.TransferDirection = LL_SPI_FULL_DUPLEX;
	spi.Mode = LL_SPI_MODE_MASTER;
	spi.DataWidth = LL_SPI_DATAWIDTH_8BIT;
	spi.ClockPolarity = LL_SPI_POLARITY_LOW;
	spi.ClockPhase = LL_SPI_PHASE_1EDGE;
	spi.NSS = LL_SPI_NSS_SOFT;
	spi.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV64;
	spi.BitOrder = LL_SPI_MSB_FIRST;
	spi.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;

	LL_SPI_SetFIFOThreshold(SPI1, LL_SPI_FIFO_TH_01DATA);

	LL_SPI_Init(SPI1, &spi);

	spi1InitDMA();

	LL_SPI_Enable(SPI1);

	spi1Initialized = 1;

	return 1;
}

void DMA2_Stream4_IRQHandler(void) {
	if (LL_DMA_IsActiveFlag_TC4(DMA2)) {
		LL_DMA_ClearFlag_TC4(DMA2);

		// Stop RX & TX DMA
		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_4);
		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_5);

		LL_SPI_DisableDMAReq_RX(SPI1);
		LL_SPI_DisableDMAReq_TX(SPI1);

		spi1DeSelectAllDevices();

		spi1_async_busy = 0;
		spi1_transfer_complete = 1;

		SCB_InvalidateDCache_by_Addr((uint32_t*) spi1_rx_buf, spi1_async_len + 1);

		if (spi1_async_callback) {
			spi1_async_callback(spi1_rx_buf + 1, spi1_async_len);
		}

		spi1_async_callback = NULL;
		spi1_async_len = 0;
	}

	// Transfer error handling
	if (LL_DMA_IsActiveFlag_TE4(DMA2)) {
		LL_DMA_ClearFlag_TE4(DMA2);

		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_4);
		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_5);
		LL_SPI_DisableDMAReq_RX(SPI1);
		LL_SPI_DisableDMAReq_TX(SPI1);
		spi1DeSelectAllDevices();

		spi1_async_callback = NULL;
		spi1_async_len = 0;
		spi1_async_busy = 0;
		spi1_transfer_complete = 1;
	}
}

uint8_t spi1ReadRegisterAsync(uint8_t reg, uint16_t rxLen, uint8_t device, spi_callback_t callback) {
	if (spi1_async_busy) {
		return 0;
	}
	uint16_t total = rxLen + 1;
	if (total > SPI1_DMA_MAX_TRANSFER_LEN) {
		return 0;
	}
	if (!spi1ClearDMAFlags()) {
		return 0;
	}
	spi1_async_busy = 1;
	spi1_async_callback = callback;
	spi1_async_len = rxLen;

	spi1SelectDevice(device);

	// Prepare TX buffer , Note , the padding is inverted
	memset(spi1_tx_buf, 0x00, total);
	//Note the Mask is inverted.
	spi1_tx_buf[0] = reg & FC_SPI_WRITE_MASK;

	SCB_CleanDCache_by_Addr((uint32_t*) spi1_tx_buf, total);

	// Setup DMA
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_4);
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_5);

	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_4, (uint32_t) spi1_rx_buf);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_4, total);

	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_5, (uint32_t) spi1_tx_buf);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_5, total);

	LL_SPI_EnableDMAReq_RX(SPI1);
	LL_SPI_EnableDMAReq_TX(SPI1);

	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_4);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);

	LL_SPI_StartMasterTransfer(SPI1);

	return 1;
}

uint8_t spi1ReadRegister(uint8_t regAddr, uint8_t *rxData, uint16_t rxLen, uint8_t device) {
	spi1_transfer_complete = 0;
	if (!spi1ReadRegisterAsync(regAddr, rxLen, device, NULL)) {
		return 0;
	}
	uint32_t timeout_counter = SPI_IO_TIMEOUT_COUNT;
	while ( spi1_transfer_complete == 0 ) {
		if (timeout_counter-- == 0) {
			spi1_async_busy = 0;
			return 0;
		}
	}
	uint16_t total = rxLen + 1;
	SCB_InvalidateDCache_by_Addr((uint32_t*) spi1_rx_buf, total);
	if (total <= SPI1_DMA_MAX_TRANSFER_LEN) {
		memcpy(rxData, spi1_rx_buf + 1, rxLen);
	}
	return 1;
}

uint8_t spi1WriteRegisterAsync(uint8_t regAddr, uint8_t *txData, uint16_t txLen, uint8_t device, spi_callback_t callback) {
	if (spi1_async_busy) {
		return 0;
	}

	uint16_t total = txLen + 1;
	if (total > SPI1_DMA_MAX_TRANSFER_LEN) {
		return 0;
	}

	if (!spi1ClearDMAFlags()) {
		return 0;
	}

	spi1_async_busy = 1;
	spi1_async_callback = callback;
	spi1_async_len = txLen;

	spi1SelectDevice(device);

	/* -------- Prepare TX buffer -------- */
	spi1_tx_buf[0] = regAddr | FC_SPI_READ_MASK;
	memcpy(spi1_tx_buf + 1, txData, txLen);

	SCB_CleanDCache_by_Addr((uint32_t*) spi1_tx_buf, total);

	/* -------- Setup DMA -------- */
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_4);   // RX
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_5);   // TX

	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_4, (uint32_t) spi1_rx_buf);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_4, total);

	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_5, (uint32_t) spi1_tx_buf);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_5, total);

	/* Enable DMA interrupts */
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_4);
	LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_4);

	/* -------- Start DMA (order: RX then TX) -------- */
	LL_SPI_EnableDMAReq_RX(SPI1);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_4);

	LL_SPI_EnableDMAReq_TX(SPI1);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);

	LL_SPI_StartMasterTransfer(SPI1);

	return 1;
}

uint8_t spi1WriteRegister(uint8_t regAddr, uint8_t *txData, uint16_t txLen, uint8_t device) {
	spi1_transfer_complete = 0;
	if (!spi1WriteRegisterAsync(regAddr, txData, txLen, device, NULL)) {
		return 0;
	}
	uint32_t timeout_counter = SPI_IO_TIMEOUT_COUNT;
	while ( spi1_transfer_complete == 0 ) {
		if (timeout_counter-- == 0) {
			spi1_async_busy = 0;
			return 0;
		}
	}
	return 1;
}
