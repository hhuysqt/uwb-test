/*
 * Use internal flash to emulate EEPROM
 */

#include <stm32f0xx_hal.h>
#include <string.h>
#include "eeprom.h"

#define EEPROM_FLASH_ADDR 0x0801f800
#define EEPROM_FLASH_SIZE 200
static char rwbuffer[EEPROM_FLASH_SIZE];

void eepromInit(void)
{
	memcpy(rwbuffer, (const void*)EEPROM_FLASH_ADDR, EEPROM_FLASH_SIZE);
}

bool eepromRead(int address, void* data, size_t length)
{
	memcpy(data, (char*)EEPROM_FLASH_ADDR + address, length);
	memcpy(rwbuffer, (const void*)EEPROM_FLASH_ADDR, EEPROM_FLASH_SIZE);
	return true;
}

bool eepromWrite(int address, void* data, size_t length)
{
	uint16_t *data_to_write = (uint16_t*)rwbuffer;
	bool status = true;

	// read, modify, and write
	memcpy(rwbuffer, (const void*)EEPROM_FLASH_ADDR, EEPROM_FLASH_SIZE);
	memcpy(rwbuffer + address, data, length);

	HAL_FLASH_Unlock();

	FLASH_EraseInitTypeDef erasepage;
	uint32_t pageerror;
	erasepage.TypeErase = FLASH_TYPEERASE_PAGES;
	erasepage.PageAddress = EEPROM_FLASH_ADDR;
	erasepage.NbPages = 1;
	HAL_FLASHEx_Erase(&erasepage, &pageerror);

	for (int i = 0; i < EEPROM_FLASH_SIZE/2; i++) {
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, 
			EEPROM_FLASH_ADDR + i*2, data_to_write[i]) != HAL_OK){
			status = false;
			break;
		}
	}

	HAL_FLASH_Lock();
	return status;
}

