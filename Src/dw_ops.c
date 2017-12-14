/*
 * Wrapper of stm32f072 SPI, GPIO driver for DWM1000
 */
#include <libdw1000.h>
#include <string.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>
#include <main.h>
#include <eeprom.h>

extern SPI_HandleTypeDef hspi1;

/*
 * Aligned buffer of 128bytes
 * This is used as a "scratch" buffer to the SPI transfers
 * The problem is that the Cortex-m0 only supports 2Bytes-aligned memory access
 */
uint16_t alignedBuffer[64];

/*
 * Transmit both header and data
 */
static void spiWrite(dwDevice_t* dev, const void *header, size_t headerLength,
		const void* data, size_t dataLength)
{
	(void)dev;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);

	memcpy(alignedBuffer, header, headerLength);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)alignedBuffer, headerLength, HAL_MAX_DELAY);
	memcpy(alignedBuffer, data, dataLength);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)alignedBuffer, dataLength, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
}

/*
 * Send request and receive data
 */
static void spiRead(dwDevice_t* dev, const void *header, size_t headerLength,
		void* data, size_t dataLength)
{
	(void)dev;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);

	memcpy(alignedBuffer, header, headerLength);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)alignedBuffer, headerLength, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, (uint8_t *)alignedBuffer, dataLength, HAL_MAX_DELAY);
	memcpy(data, alignedBuffer, dataLength);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
}

static void spiSetSpeed(dwDevice_t* dev, dwSpiSpeed_t speed)
{
	(void)dev;
	if(speed == dwSpiSpeedLow) {
		MX_SPI1_Init();
	} else {
		MX_SPI1_Init_Fast();
	}
}

/*
 * Tuggle reset pin
 */
static void reset(dwDevice_t* dev)
{
	(void)dev;
	HAL_GPIO_WritePin(DWM1000_RST_GPIO_Port, DWM1000_RST_Pin, 0);
	HAL_Delay(2);
	HAL_GPIO_WritePin(DWM1000_RST_GPIO_Port, DWM1000_RST_Pin, 1);
}

static void delayms(dwDevice_t* dev, unsigned int delay)
{
	(void)dev;
	HAL_Delay(delay);
}

/*
 * function list
 */
dwOps_t dwOps = {
	.spiRead = spiRead,
	.spiWrite = spiWrite,
	.spiSetSpeed = spiSetSpeed,
	.delayms = delayms,
	.reset = reset,
};
