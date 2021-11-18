#include "Flash.h"
#include "EnergocycleSettings.h"

extern EC_FLASH_settings eflash_settings;

#define MY_FLASH_PAGE_ADDR 0x800FC00
#define SETTINGS_WORDS sizeof(eflash_settings)/4

void FLASH_Init(void) {
	/* Next commands may be used in SysClock initialization function
	   In this case using of FLASH_Init is not obligatorily */
	/* Enable Prefetch Buffer */
	__HAL_FLASH_PREFETCH_BUFFER_ENABLE();
	//FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);
	/* Flash 2 wait state */
	__HAL_FLASH_SET_LATENCY(FLASH_LATENCY_2);
	//FLASH_SetLatency( FLASH_Latency_2);
}

void FLASH_ReadSettings(void) {
	//Read settings
	uint32_t *source_addr = (uint32_t *)MY_FLASH_PAGE_ADDR;
	uint32_t *dest_addr = (uint32_t *)&eflash_settings;
	for (uint16_t i=0; i < SETTINGS_WORDS; i++) {
		*dest_addr = *(__IO uint32_t*)source_addr;
		source_addr++;
		dest_addr++;
	}
}

void FLASH_WriteSettings(void) {
	HAL_FLASH_Unlock();

	uint32_t SectorError = 0;
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks = FLASH_BANK_1;
	EraseInitStruct.PageAddress = MY_FLASH_PAGE_ADDR;
	EraseInitStruct.NbPages = 1;
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {

	}

	// Write settings
	uint32_t *source_addr = (uint32_t *)&eflash_settings;
	uint32_t *dest_addr = (uint32_t *) MY_FLASH_PAGE_ADDR;
	for (uint16_t i=0; i < SETTINGS_WORDS; i++) {
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)dest_addr, *source_addr);
		source_addr++;
		dest_addr++;
	}

	HAL_FLASH_Lock();
}

