#include "SPI.h"

#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_ll_bdma.h"
#include "stm32h7xx_LL_DMAmux.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_utils.h"
#include "string.h"

static uint8_t spi6Initialized = 0;

static void spi6DeSelectAllDevices(void);
static void spi6SelectDevice(uint8_t device);
static void spi6CSInit(void);
static uint8_t spi6ClearDMAFlags(void);

#define SPI6_DMA_MAX_TRANSFER_LEN  16
__ATTR_RAM_D3 uint8_t spi6_tx_buf[SPI6_DMA_MAX_TRANSFER_LEN];
__ATTR_RAM_D3 uint8_t spi6_rx_buf[SPI6_DMA_MAX_TRANSFER_LEN];

static spi_callback_t spi6_async_callback = NULL;
static uint16_t spi6_async_len = 0;
static volatile uint8_t spi6_async_busy = 0;
static volatile uint8_t spi6_transfer_complete = 0;

static void spi6CSInit(void) {
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);

	LL_GPIO_InitTypeDef gpio = { 0 };
	gpio.Pin = LL_GPIO_PIN_2;
	gpio.Mode = LL_GPIO_MODE_OUTPUT;
	gpio.Pull = LL_GPIO_PULL_UP;
	gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;

	LL_GPIO_Init(GPIOA, &gpio);
	spi6DeSelectAllDevices();
}

static void spi6SelectDevice(uint8_t device) {
	spi6DeSelectAllDevices();
	if (device == FC_SPI6_DEVICE1) {
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2);
	}
}

static void spi6DeSelectAllDevices(void) {
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_2);
}

uint8_t spi6ClearDMAFlags(void) {
	LL_BDMA_ClearFlag_TC0(BDMA);
	LL_BDMA_ClearFlag_HT0(BDMA);
	LL_BDMA_ClearFlag_TE0(BDMA);

	LL_BDMA_ClearFlag_TC1(BDMA);
	LL_BDMA_ClearFlag_HT1(BDMA);
	LL_BDMA_ClearFlag_TE1(BDMA);

	LL_SPI_ClearFlag_OVR(SPI6);
	LL_SPI_ClearFlag_EOT(SPI6);
	LL_SPI_ClearFlag_TXTF(SPI6);
	LL_SPI_ClearFlag_MODF(SPI6);

	uint32_t timeout_counter = SPI_IO_TIMEOUT_COUNT;
	volatile uint32_t dummy;
	while ( LL_SPI_IsActiveFlag_RXP(SPI6) ) {
		dummy = SPI6->RXDR;
		(void) dummy;
		if (timeout_counter-- == 0) {
			return 0;
		}
	}
	return 1;
}

static void spi6EnableDMAInterrupts(void) {
	LL_BDMA_EnableIT_TC(BDMA, LL_BDMA_CHANNEL_0);
	NVIC_SetPriority(BDMA_Channel0_IRQn, 5);
	NVIC_EnableIRQ(BDMA_Channel0_IRQn);
}

void BDMA_Channel0_IRQHandler(void) {
	if (LL_BDMA_IsActiveFlag_TC0(BDMA)) {
		LL_BDMA_ClearFlag_TC0(BDMA);

		LL_BDMA_DisableChannel(BDMA, LL_BDMA_CHANNEL_0);
		LL_BDMA_DisableChannel(BDMA, LL_BDMA_CHANNEL_1);
		LL_SPI_DisableDMAReq_RX(SPI6);
		LL_SPI_DisableDMAReq_TX(SPI6);

		spi6DeSelectAllDevices();

		SCB_InvalidateDCache_by_Addr((uint32_t*) spi6_rx_buf, spi6_async_len + 1);

		if (spi6_async_callback) {
			spi6_async_callback(spi6_rx_buf + 1, spi6_async_len);
		}

		spi6_async_callback = NULL;
		spi6_async_len = 0;
		spi6_async_busy = 0;
		spi6_transfer_complete = 1;
	}
	if (LL_BDMA_IsActiveFlag_TE0(BDMA)) {
		LL_BDMA_ClearFlag_TE0(BDMA);

		LL_BDMA_DisableChannel(BDMA, LL_BDMA_CHANNEL_0);
		LL_BDMA_DisableChannel(BDMA, LL_BDMA_CHANNEL_1);
		LL_SPI_DisableDMAReq_RX(SPI6);
		LL_SPI_DisableDMAReq_TX(SPI6);
		spi6DeSelectAllDevices();

		spi6_async_busy = 0;
		spi6_transfer_complete = 1;
	}
}

void spi6InitDMA(void) {
	spi6ClearDMAFlags();

	LL_BDMA_DisableChannel(BDMA, LL_BDMA_CHANNEL_0);
	LL_BDMA_DeInit(BDMA, LL_BDMA_CHANNEL_0);

	LL_DMAMUX_SetRequestID(DMAMUX2, LL_BDMA_CHANNEL_0, LL_DMAMUX2_REQ_SPI6_RX);
	LL_BDMA_SetDataTransferDirection(BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_BDMA_SetChannelPriorityLevel(BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_PRIORITY_HIGH);
	LL_BDMA_SetMode(BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_MODE_NORMAL);
	LL_BDMA_SetPeriphIncMode(BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_PERIPH_NOINCREMENT);
	LL_BDMA_SetMemoryIncMode(BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_MEMORY_INCREMENT);
	LL_BDMA_SetPeriphSize(BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_PDATAALIGN_BYTE);
	LL_BDMA_SetMemorySize(BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_MDATAALIGN_BYTE);
	LL_BDMA_SetPeriphAddress(BDMA, LL_BDMA_CHANNEL_0, (uint32_t) &SPI6->RXDR);

	LL_BDMA_DisableChannel(BDMA, LL_BDMA_CHANNEL_1);
	LL_BDMA_DeInit(BDMA, LL_BDMA_CHANNEL_1);

	LL_DMAMUX_SetRequestID(DMAMUX2, LL_BDMA_CHANNEL_1, LL_DMAMUX2_REQ_SPI6_TX);
	LL_BDMA_SetDataTransferDirection(BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_BDMA_SetChannelPriorityLevel(BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_PRIORITY_HIGH);
	LL_BDMA_SetMode(BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_MODE_NORMAL);
	LL_BDMA_SetPeriphIncMode(BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_PERIPH_NOINCREMENT);
	LL_BDMA_SetMemoryIncMode(BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_MEMORY_INCREMENT);
	LL_BDMA_SetPeriphSize(BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_PDATAALIGN_BYTE);
	LL_BDMA_SetMemorySize(BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_MDATAALIGN_BYTE);
	LL_BDMA_SetPeriphAddress(BDMA, LL_BDMA_CHANNEL_1, (uint32_t) &SPI6->TXDR);

	spi6EnableDMAInterrupts();
}

uint8_t spi6Init(void) {
	if (spi6Initialized) {
		return 1;
	}

	spi6CSInit();

	LL_RCC_SetSPIClockSource(LL_RCC_SPI6_CLKSOURCE_PCLK4);
	LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SPI6);
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_BDMA);

	//LL_AHB4_GRP1_ForceReset(LL_AHB4_GRP1_PERIPH_BDMA);
	//LL_AHB4_GRP1_ReleaseReset(LL_AHB4_GRP1_PERIPH_BDMA);

	LL_GPIO_InitTypeDef gpio = { 0 };
	gpio.Pin = LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
	gpio.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio.Alternate = LL_GPIO_AF_8;
	gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &gpio);

	LL_SPI_DeInit(SPI6);

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

	LL_SPI_SetFIFOThreshold(SPI6, LL_SPI_FIFO_TH_01DATA);

	LL_SPI_Init(SPI6, &spi);

	spi6InitDMA();

	LL_SPI_Enable(SPI6);

	spi6Initialized = 1;
	return 1;
}

uint8_t spi6ReadRegisterAsync(uint8_t regAddr, uint16_t rxLen, uint8_t device, spi_callback_t callback) {
	if (spi6_async_busy) {
		return 0;
	}
	uint16_t total = rxLen + 1;
	if (total > SPI6_DMA_MAX_TRANSFER_LEN) {
		return 0;
	}
	if (!spi6ClearDMAFlags()) {
		return 0;
	}

	spi6_async_busy = 1;
	spi6_async_callback = callback;
	spi6_async_len = rxLen;

	spi6SelectDevice(device);

	memset(spi6_tx_buf, 0xFF, total);

	spi6_tx_buf[0] = regAddr | FC_SPI_READ_MASK;

	SCB_CleanDCache_by_Addr((uint32_t*) spi6_tx_buf, total);

	LL_BDMA_DisableChannel(BDMA, LL_BDMA_CHANNEL_0);
	LL_BDMA_DisableChannel(BDMA, LL_BDMA_CHANNEL_1);

	LL_BDMA_SetMemoryAddress(BDMA, LL_BDMA_CHANNEL_0, (uint32_t) spi6_rx_buf);
	LL_BDMA_SetDataLength(BDMA, LL_BDMA_CHANNEL_0, total);

	LL_BDMA_SetMemoryAddress(BDMA, LL_BDMA_CHANNEL_1, (uint32_t) spi6_tx_buf);
	LL_BDMA_SetDataLength(BDMA, LL_BDMA_CHANNEL_1, total);

	LL_SPI_EnableDMAReq_RX(SPI6);
	LL_BDMA_EnableChannel(BDMA, LL_BDMA_CHANNEL_0);

	LL_SPI_EnableDMAReq_TX(SPI6);
	LL_BDMA_EnableChannel(BDMA, LL_BDMA_CHANNEL_1);

	LL_SPI_StartMasterTransfer(SPI6);

	return 1;
}

uint8_t spi6ReadRegister(uint8_t regAddr, uint8_t *rxData, uint16_t rxLen, uint8_t device) {
	spi6_transfer_complete = 0;
	if (!spi6ReadRegisterAsync(regAddr, rxLen, device, NULL)) {
		return 0;
	}
	uint32_t timeout_counter = SPI_IO_TIMEOUT_COUNT;
	while ( spi6_transfer_complete == 0 ) {
		if (timeout_counter-- == 0) {
			spi6_async_busy = 0;
			return 0;
		}
	}
	uint16_t total = rxLen + 1;
	SCB_InvalidateDCache_by_Addr((uint32_t*) spi6_rx_buf, total);
	if (total <= SPI6_DMA_MAX_TRANSFER_LEN) {
		memcpy(rxData, spi6_rx_buf + 1, rxLen);
	}
	return 1;
}

uint8_t spi6WriteRegisterAsync(uint8_t regAddr, uint8_t *txData, uint16_t txLen, uint8_t device, spi_callback_t callback) {
	if (spi6_async_busy) {
		return 0;
	}
	uint16_t total = txLen + 1;
	if (total > SPI6_DMA_MAX_TRANSFER_LEN) return 0;
	if (!spi6ClearDMAFlags()) return 0;

	spi6_async_busy = 1;
	spi6_async_callback = callback;
	spi6_async_len = txLen;

	spi6SelectDevice(device);

	spi6_tx_buf[0] = regAddr & FC_SPI_WRITE_MASK;
	memcpy(spi6_tx_buf + 1, txData, txLen);

	SCB_CleanDCache_by_Addr((uint32_t*) spi6_tx_buf, total);

	LL_BDMA_DisableChannel(BDMA, LL_BDMA_CHANNEL_0);
	LL_BDMA_DisableChannel(BDMA, LL_BDMA_CHANNEL_1);
	LL_BDMA_SetMemoryAddress(BDMA, LL_BDMA_CHANNEL_0, (uint32_t) spi6_rx_buf);
	LL_BDMA_SetDataLength(BDMA, LL_BDMA_CHANNEL_0, total);
	LL_BDMA_SetMemoryAddress(BDMA, LL_BDMA_CHANNEL_1, (uint32_t) spi6_tx_buf);
	LL_BDMA_SetDataLength(BDMA, LL_BDMA_CHANNEL_1, total);

	LL_SPI_EnableDMAReq_RX(SPI6);
	LL_BDMA_EnableChannel(BDMA, LL_BDMA_CHANNEL_0);
	LL_SPI_EnableDMAReq_TX(SPI6);
	LL_BDMA_EnableChannel(BDMA, LL_BDMA_CHANNEL_1);
	LL_SPI_StartMasterTransfer(SPI6);

	return 1;
}

uint8_t spi6WriteRegister(uint8_t regAddr, uint8_t *txData, uint16_t txLen, uint8_t device) {
	spi6_transfer_complete = 0;
	if (!spi6WriteRegisterAsync(regAddr, txData, txLen, device, NULL)) {
		return 0;
	}
	uint32_t timeout_counter = SPI_IO_TIMEOUT_COUNT;
	while ( spi6_transfer_complete == 0 ) {
		if (timeout_counter-- == 0) {
			spi6_async_busy = 0;
			return 0;
		}
	}
	return 1;
}

