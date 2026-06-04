#include "I2C.h"

#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_i2c.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_dmamux.h"
#include "../../memory/Memory.h"
#include "../../logger/Logger.h"
#include <string.h>

/* ---------- INTERNAL STATE ---------- */
static uint8_t i2c1Initialized = 0;
static volatile uint8_t i2c1_async_busy = 0;
static volatile uint8_t i2c1_transfer_complete = 0;
static uint16_t i2c1_async_len = 0;
static volatile i2c_callback_t i2c1_async_callback = NULL;
static uint8_t *i2c1_async_active_buf = NULL;

#define I2C1_DMA_MAX_TRANSFER_LEN   32
/* Allocated in D2 RAM to allow DMA2 access */
__ATTR_RAM_D2 static uint8_t i2c1_tx_buf[I2C1_DMA_MAX_TRANSFER_LEN];
__ATTR_RAM_D2 static uint8_t i2c1_rx_buf[I2C1_DMA_MAX_TRANSFER_LEN];

/* ---------- STATIC HELPERS ---------- */
static uint8_t i2c1ClearDMAFlags(void);
static void i2c1InitDMA(void);
static void i2c1EnableDMAInterrupts(void);

static uint8_t i2c1ClearDMAFlags(void) {
	/* TX: DMA2 Stream 2 */
	LL_DMA_ClearFlag_TC2(DMA2);
	LL_DMA_ClearFlag_HT2(DMA2);
	LL_DMA_ClearFlag_TE2(DMA2);
	LL_DMA_ClearFlag_DME2(DMA2);
	LL_DMA_ClearFlag_FE2(DMA2);

	/* RX: DMA2 Stream 3 */
	LL_DMA_ClearFlag_TC3(DMA2);
	LL_DMA_ClearFlag_HT3(DMA2);
	LL_DMA_ClearFlag_TE3(DMA2);
	LL_DMA_ClearFlag_DME3(DMA2);
	LL_DMA_ClearFlag_FE3(DMA2);

	/* Clear peripheral flags */
	LL_I2C_ClearFlag_NACK(I2C1);
	LL_I2C_ClearFlag_STOP(I2C1);
	LL_I2C_ClearFlag_BERR(I2C1);
	LL_I2C_ClearFlag_OVR(I2C1);

	return 1;
}

static void i2c1EnableDMAInterrupts(void) {
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_2);
	LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_2);
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_3);
	LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_3);

	NVIC_SetPriority(DMA2_Stream2_IRQn, 2);
	NVIC_EnableIRQ(DMA2_Stream2_IRQn);

	NVIC_SetPriority(DMA2_Stream3_IRQn, 2);
	NVIC_EnableIRQ(DMA2_Stream3_IRQn);

	NVIC_SetPriority(I2C1_EV_IRQn, 5);
	NVIC_EnableIRQ(I2C1_EV_IRQn);

	NVIC_SetPriority(I2C1_ER_IRQn, 5);
	NVIC_EnableIRQ(I2C1_ER_IRQn);
}

void I2C1_ER_IRQHandler(void) {
	uint8_t error_active = 0;

	if (LL_I2C_IsActiveFlag_BERR(I2C1)) {
		LL_I2C_ClearFlag_BERR(I2C1);
		error_active = 1;
	}
	if (LL_I2C_IsActiveFlag_ARLO(I2C1)) {
		LL_I2C_ClearFlag_ARLO(I2C1);
		error_active = 1;
	}
	if (LL_I2C_IsActiveFlag_OVR(I2C1)) {
		LL_I2C_ClearFlag_OVR(I2C1);
		error_active = 1;
	}
	/* If a bus error disrupted an active async transaction, force a clean state reset */
	if (error_active && i2c1_async_busy) {
		/* Force abort ongoing DMA operations to release the bus matrix */
		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);
		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_3);

		LL_I2C_DisableDMAReq_TX(I2C1);
		LL_I2C_DisableDMAReq_RX(I2C1);
		LL_I2C_DisableIT_STOP(I2C1);

		i2c1_async_busy = 0;
		i2c1_transfer_complete = 0; /* Force 0 so synchronous wrappers fail immediately */

		/* Notify the async listener of execution failure using a NULL pointer token */
		if (i2c1_async_callback) {
			i2c1_async_callback(NULL, 0);
		}
	}
}

void I2C1_EV_IRQHandler(void) {
	if (LL_I2C_IsActiveFlag_NACK(I2C1)) {
		LL_I2C_ClearFlag_NACK(I2C1);
		LL_I2C_DisableIT_STOP(I2C1);
		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_3);
		i2c1_async_busy = 0;
		i2c1_transfer_complete = 0;
		return;
	}
	if (LL_I2C_IsActiveFlag_STOP(I2C1)) {
		LL_I2C_ClearFlag_STOP(I2C1);
		LL_I2C_DisableIT_STOP(I2C1);
		if (i2c1_async_busy) {
			i2c1_async_busy = 0;
			i2c1_transfer_complete = 1;

			/* Invalidate L1 cache line ahead of CPU reads */
			SCB_InvalidateDCache_by_Addr((uint32_t*) i2c1_async_active_buf, i2c1_async_len);

			if (i2c1_async_callback) {
				/* Passes internal i2c1_rx_buf or i2c1_tx_buf directly to callback */
				i2c1_async_callback(i2c1_async_active_buf, i2c1_async_len);
			}
		}
	}
}

/* TX Stream Handler */
void DMA2_Stream2_IRQHandler(void) {
	if (LL_DMA_IsActiveFlag_TC2(DMA2)) {
		LL_DMA_ClearFlag_TC2(DMA2);
		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);
		LL_I2C_DisableDMAReq_TX(I2C1);
	}
	if (LL_DMA_IsActiveFlag_TE2(DMA2)) {
		LL_DMA_ClearFlag_TE2(DMA2);
		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);
		LL_I2C_DisableDMAReq_TX(I2C1);
		LL_I2C_DisableIT_STOP(I2C1);
		i2c1_async_busy = 0;
		i2c1_transfer_complete = 1;
	}
}

/* RX Stream Handler */
void DMA2_Stream3_IRQHandler(void) {
	if (LL_DMA_IsActiveFlag_TC3(DMA2)) {
		LL_DMA_ClearFlag_TC3(DMA2);
		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_3);
		LL_I2C_DisableDMAReq_RX(I2C1);
       /*
		if (i2c1_async_active_buf != NULL) {
			SCB_InvalidateDCache_by_Addr((uint32_t*) i2c1_async_active_buf, i2c1_async_len);
		}
		*/
	}
	if (LL_DMA_IsActiveFlag_TE3(DMA2)) {
		LL_DMA_ClearFlag_TE3(DMA2);
		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_3);
		LL_I2C_DisableDMAReq_RX(I2C1);
		LL_I2C_DisableIT_STOP(I2C1);
		i2c1_async_busy = 0;
		i2c1_transfer_complete = 0; /* Clear completion flag on hardware bus failure */
	}
}

static void i2c1InitDMA(void) {
	i2c1ClearDMAFlags();

	/* DMAMUX Channels 8 to 15 route requests to DMA2 Streams 0 to 7 */

	/* TX Configuration: DMA2 Stream 2 -> Channel 10 */
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);
	LL_DMA_DeInit(DMA2, LL_DMA_STREAM_2);
	LL_DMAMUX_SetRequestID(DMAMUX1, LL_DMAMUX_CHANNEL_10, LL_DMAMUX1_REQ_I2C1_TX);
	LL_DMA_SetPeriphRequest(DMA2, LL_DMA_STREAM_2, LL_DMAMUX1_REQ_I2C1_TX);

	LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_2, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_2, (uint32_t) &I2C1->TXDR);

	/* RX Configuration: DMA2 Stream 3 -> Channel 11 */
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_3);
	LL_DMA_DeInit(DMA2, LL_DMA_STREAM_3);
	LL_DMAMUX_SetRequestID(DMAMUX1, LL_DMAMUX_CHANNEL_11, LL_DMAMUX1_REQ_I2C1_RX);
	LL_DMA_SetPeriphRequest(DMA2, LL_DMA_STREAM_3, LL_DMAMUX1_REQ_I2C1_RX);

	LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_3, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_3, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMode(DMA2, LL_DMA_STREAM_3, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_3, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_3, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_3, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_3, LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_3, (uint32_t) &I2C1->RXDR);

	LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_2);
	LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_3);

	i2c1EnableDMAInterrupts();
}

/* ------------------------------------------------ */
/* INITIALIZATION                                   */
/* ------------------------------------------------ */

uint8_t initI2C1(void) {
	if (i2c1Initialized) {
		return 1;
	}

	LL_RCC_SetI2CClockSource(LL_RCC_I2C123_CLKSOURCE_PCLK1);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

	LL_GPIO_InitTypeDef gpio = { 0 };
	gpio.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9;
	gpio.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio.Alternate = LL_GPIO_AF_4;
	gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	gpio.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(GPIOB, &gpio);

	LL_I2C_DeInit(I2C1);

	LL_I2C_InitTypeDef i2c = { 0 };
	i2c.PeripheralMode = LL_I2C_MODE_I2C;
	i2c.Timing = 0x10803674;     /* 400kHz , Bus Speed 137.5Mz*/
	//i2c.Timing = 0x40707D94;   /* 100kHz Bus Speed 137.5Mz*/
	i2c.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
	i2c.DigitalFilter = 0;
	i2c.OwnAddress1 = 0;
	i2c.TypeAcknowledge = LL_I2C_ACK;

	LL_I2C_Init(I2C1, &i2c);
	LL_I2C_EnableIT_ERR(I2C1);
	LL_I2C_Enable(I2C1);

	i2c1InitDMA();

	i2c1Initialized = 1;
	return 1;
}

/* ------------------------------------------------ */
/* ASYNCHRONOUS METHODS                             */
/* ------------------------------------------------ */

uint8_t i2c1WriteAsync(uint8_t address, uint8_t *data, uint16_t length, i2c_callback_t callback) {
	if (i2c1_async_busy || LL_I2C_IsActiveFlag_BUSY(I2C1))
		return 0;
	if (length > I2C1_DMA_MAX_TRANSFER_LEN)
		return 0;
	if (!i2c1ClearDMAFlags())
		return 0;

	i2c1_async_busy = 1;
	i2c1_async_len = length;
	i2c1_async_callback = callback;
	i2c1_async_active_buf = i2c1_tx_buf;
	i2c1_transfer_complete = 0;

	/* Safely staging data into specialized D2 RAM bounce array */
	memcpy(i2c1_tx_buf, data, length);

	/* Flush updates out of core cache to volatile RAM for DMA consumption */
	SCB_CleanDCache_by_Addr((uint32_t*) i2c1_tx_buf, length);

	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_2, (uint32_t) i2c1_tx_buf);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, length);

	LL_I2C_EnableDMAReq_TX(I2C1);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
	LL_I2C_EnableIT_STOP(I2C1);

	LL_I2C_HandleTransfer(I2C1, (uint32_t) address << 1, LL_I2C_ADDRESSING_MODE_7BIT, length, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
	return 1;
}

/* SIGNATURE UPDATED: Drop 'data' parameter to target internal bounce buffer exclusively */
uint8_t i2c1ReadAsync(uint8_t address, uint16_t length, i2c_callback_t callback) {
	if (i2c1_async_busy || LL_I2C_IsActiveFlag_BUSY(I2C1))
		return 0;
	if (length > I2C1_DMA_MAX_TRANSFER_LEN)
		return 0;
	if (!i2c1ClearDMAFlags())
		return 0;

	i2c1_async_busy = 1;
	i2c1_async_len = length;
	i2c1_async_callback = callback;
	i2c1_async_active_buf = i2c1_rx_buf; /* Target local bounce array directly */
	i2c1_transfer_complete = 0;

	/* Invalidate regional lines prior to hardware execution to prevent cache hits on stale space */
	SCB_InvalidateDCache_by_Addr((uint32_t*) i2c1_rx_buf, length);

	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_3);
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_3, (uint32_t) i2c1_rx_buf);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_3, length);

	LL_I2C_EnableDMAReq_RX(I2C1);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_3);
	LL_I2C_EnableIT_STOP(I2C1);

	LL_I2C_HandleTransfer(I2C1, (uint32_t) address << 1, LL_I2C_ADDRESSING_MODE_7BIT, length, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
	return 1;
}

/* ------------------------------------------------ */
/* SYNCHRONOUS METHODS                              */
/* ------------------------------------------------ */

uint8_t i2c1Write(uint8_t address, uint8_t *data, uint16_t length) {
	i2c1_transfer_complete = 0;
	if (!i2c1WriteAsync(address, data, length, NULL)) {
		return 0;
	}
	uint32_t timeout_counter = I2C_IO_TIMEOUT_COUNT;

	while (i2c1_transfer_complete == 0 && !LL_I2C_IsActiveFlag_STOP(I2C1)) {
		/* INSTANT BREAKOUT: If ER_IRQHandler cleared busy state, exit immediately */
		if (i2c1_async_busy == 0) {
			return 0;
		}
		if (timeout_counter-- == 0) {
			LL_I2C_DisableIT_STOP(I2C1);
			i2c1_async_busy = 0;
			return 0;
		}
	}

	if (LL_I2C_IsActiveFlag_STOP(I2C1)) {
		LL_I2C_ClearFlag_STOP(I2C1);
		LL_I2C_DisableIT_STOP(I2C1);
		i2c1_async_busy = 0;
		i2c1_transfer_complete = 1;
	}
	return 1;
}

uint8_t i2c1Read(uint8_t address, uint8_t *data, uint16_t length) {
	i2c1_transfer_complete = 0;
	if (!i2c1ReadAsync(address, length, NULL)) {
		return 0;
	}
	uint32_t timeout_counter = I2C_IO_TIMEOUT_COUNT;

	while (i2c1_transfer_complete == 0 && !LL_I2C_IsActiveFlag_STOP(I2C1)) {
		/* INSTANT BREAKOUT: Exit immediately if a bus error killed the transaction */
		if (i2c1_async_busy == 0) {
			return 0;
		}
		if (timeout_counter-- == 0) {
			LL_I2C_DisableIT_STOP(I2C1);
			i2c1_async_busy = 0;
			return 0;
		}
	}

	if (LL_I2C_IsActiveFlag_STOP(I2C1)) {
		LL_I2C_ClearFlag_STOP(I2C1);
		LL_I2C_DisableIT_STOP(I2C1);
		i2c1_async_busy = 0;
		i2c1_transfer_complete = 1;
	}

	/* Synchronous blocking adaptation: Extract data out to user target buffer */
	memcpy(data, i2c1_rx_buf, length);

	return 1;
}

/* ------------------------------------------------ */
/* MISC UTILITY METHODS                             */
/* ------------------------------------------------ */

uint8_t i2c1CheckAddress(uint8_t address) {
	uint32_t timeout = I2C_IO_TIMEOUT_COUNT;
	while (LL_I2C_IsActiveFlag_BUSY(I2C1)) {
		if (--timeout == 0)
			return 0;
	}

	LL_I2C_HandleTransfer(I2C1, (uint32_t) address << 1, LL_I2C_ADDRESSING_MODE_7BIT, 0, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

	timeout = I2C_IO_TIMEOUT_COUNT;
	while (!LL_I2C_IsActiveFlag_STOP(I2C1) && !LL_I2C_IsActiveFlag_NACK(I2C1)) {
		if (--timeout == 0)
			return 0;
	}

	if (LL_I2C_IsActiveFlag_NACK(I2C1)) {
		LL_I2C_ClearFlag_NACK(I2C1);
		timeout = I2C_IO_TIMEOUT_COUNT;
		while (!LL_I2C_IsActiveFlag_STOP(I2C1)) {
			if (--timeout == 0)
				break;
		}
		LL_I2C_ClearFlag_STOP(I2C1);
		return 0;
	}

	LL_I2C_ClearFlag_STOP(I2C1);
	return 1;
}

void i2c1_ScanBus(void) {
	logString(">>>> Scanning I2C1 Bus (0x03 - 0x77)\n");
	uint8_t devices_found = 0;
	char line_buffer[32];
	for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
		if (i2c1CheckAddress(addr)) {
			sprintf(line_buffer, "Found device at address : 0x%02X\n", addr);
			logString(line_buffer);
			devices_found++;
		}
	}
	if (devices_found == 0) {
		logString("No I2C devices detected.\r\n");
	} else {
		sprintf(line_buffer, "<<<< Scan Complete. Total: %d\n", devices_found);
		logString(line_buffer);
	}
}
