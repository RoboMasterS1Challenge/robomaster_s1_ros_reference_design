#ifndef __SPI_H__
#define __SPI_H__
#include "stm32f7xx_hal.h"
extern SPI_HandleTypeDef hspi1;

uint8_t SPI1_WriteRead(uint8_t Byte)
{
	uint8_t receivedbyte = 0;
	if(HAL_SPI_TransmitReceive(&hspi1,(uint8_t*) &Byte,(uint8_t*) &receivedbyte,1,0x1000)!=HAL_OK)
	{
		return -1;
	}else
	{
	}
	return receivedbyte;
}


#endif /* __SPI_H__ */
