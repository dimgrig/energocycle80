#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include "stm32f1xx.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"



void FLASH_Init(void);
void FLASH_ReadSettings(void);
void FLASH_WriteSettings(void);



#endif /* INC_FLASH_H_ */
