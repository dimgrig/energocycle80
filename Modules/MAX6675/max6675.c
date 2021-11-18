#include "max6675.h"

extern SPI_HandleTypeDef MAX6675_hspi;

void MAX6675_init() {
	MAX6675_SS_DESELECT();
	HAL_Delay(300); //need for MAX6675 ready + 250ms between get_temp calls!!!
}

float MAX6675_get_temp() {
	uint8_t sendbytes[2] = {0, 0};
	uint8_t receivedbytes[2] = {0, 0};

	MAX6675_SS_SELECT();
	HAL_SPI_TransmitReceive(&MAX6675_hspi, (uint8_t*) &sendbytes,
				(uint8_t*) &receivedbytes, 2, 0x1000);
	MAX6675_SS_DESELECT();

	float temp = (float)(((receivedbytes[0] << 8) | receivedbytes[1]) >> 3) * 0.25;
	return temp;
}
