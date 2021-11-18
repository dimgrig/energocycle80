#include <ledImplementation.h>

#include <stm32f1xx_hal.h>

void GreenOn ( void ) {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);}
void GreenOff ( void ) {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);}

void BlueOn ( void ) {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);}
void BlueOff ( void ) {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);}

void RedOn ( void ) {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);}
void RedOff ( void ) {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);}

void GreenBlink(void)
{
	__attribute__ ((unused)) volatile uint32_t dontCare = 0;
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	for(uint32_t i = 0; i < 300000; i++)
	{
		dontCare = i % 4;
	}
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	for(uint32_t i = 0; i < 300000; i++)
	{
		dontCare = i % 4;
	}
}

void RedBlink(void)
{
	__attribute__ ((unused)) volatile uint32_t dontCare = 0;
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
	for(uint32_t i = 0; i < 300000; i++)
	{
		dontCare = i % 4;
	}
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
	for(uint32_t i = 0; i < 300000; i++)
	{
		dontCare = i % 4;
	}
}

void BlueBlink(void)
{
	__attribute__ ((unused)) volatile uint32_t dontCare = 0;
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
	for(uint32_t i = 0; i < 300000; i++)
	{
		dontCare = i % 4;
	}
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
	for(uint32_t i = 0; i < 30000; i++)
	{
		dontCare = i % 4;
	}
}

iLed GreenLed = { GreenOn, GreenOff, GreenBlink };
iLed BlueLed = { BlueOn, BlueOff, BlueBlink };
iLed RedLed = { RedOn, RedOff, RedBlink  };
