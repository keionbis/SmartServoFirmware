/*
 * MA702.cpp
 *
 *  Created on: Sep 12, 2018
 *      Author: kbisland
 */

#include "MA702.h"
#include "math.h"

MA702::MA702(){
}

void MA702::begin(SPI_HandleTypeDef *hspi1, uint16_t CS_Pin){
	hspi = hspi1;
	_CS_Pin = CS_Pin;
	_CS_Port = *GPIOA;
	HAL_GPIO_WritePin(&_CS_Port, _CS_Pin, GPIO_PIN_SET);
}

double MA702::readAngle(){
	uint16_t angle;
	double angleInDegree;
	angle = readAngleRaw16();
	angleInDegree = (angle*360.0)/4096.0;
	return angleInDegree;
}

uint16_t MA702::readAngleRaw16(){
	uint16_t angle;
	uint16_t angle_tmp[1] = {0};
	GPIOA -> ODR &= ~GPIO_PIN_1;
	//__disable_irq();
	HAL_SPI_Transmit_DMA(hspi, 0x00, 2);
	//HAL_SPI_TransmitReceive(hspi, 0x00, (uint8_t*)angle_tmp, 2, 0xFF);
	while (SPI1->SR & SPI_SR_BSY);
	angle = angle_tmp[0];

	//__enable_irq();
	GPIOA -> ODR |= GPIO_PIN_1;
	angle = angle>>4;
	return angle>>4;
}

uint8_t MA702::readAngleRaw8(){
	uint16_t angle_tmp[1] = {0};
	uint16_t angle;
	HAL_GPIO_WritePin(&_CS_Port, _CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(hspi, 0x00, (uint8_t*)angle_tmp, 2, 0xFF);
	angle = angle_tmp[0];
	HAL_GPIO_WritePin(&_CS_Port, _CS_Pin, GPIO_PIN_SET);
	angle = angle<<8;
	return angle;
}

signed long long MA702::readAngleRaw(){
	bool error;
	uint16_t angle_tmp[2] = {0};

	uint16_t angle;
	uint8_t parity;
	uint8_t highStateCount = 0;
	//HAL_SPI_Init(hspi);_
	HAL_GPIO_WritePin( GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(hspi, 0x00, (uint8_t*)angle_tmp, 1, 0);
	//HAL_SPI_DeInit(hspi);
	while (SPI1->SR & SPI_SR_BSY);
	angle = angle_tmp[0];

	HAL_GPIO_WritePin( GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	angle = angle>>4;
	parity = angle>>8;


	parity = ((parity & 0x80) >> 7);
	for (int i=0;i<16;++i){
		if ((angle & (1 << i)) != 0){
			highStateCount++;
		}
	}
	if ((highStateCount % 2) == 0){
		if (parity == 0){
			error = false;
		}
		else{
			error = true;
		}
	}
	else{
		if (parity == 1){
			error = false;
		}
		else{
			error = true;
		}
	}
	double boundForWrap = 4096/6;
	double maxForWrap =(4096-boundForWrap);
	if(_last_angle>maxForWrap && angle<=boundForWrap)
		rotations+=1;
	else if(_last_angle<boundForWrap && angle>=maxForWrap)
		rotations-=1;
	_last_angle=angle;
	return angle;
}
long int MA702::totalAngle(){
	return ((readAngleRaw())+(rotations*4096)) ;
}
uint8_t MA702::readRegister(uint8_t address){
	uint16_t readbackRegisterValue;
	HAL_GPIO_WritePin(&_CS_Port, _CS_Pin, GPIO_PIN_RESET);
	//HAL_SPI_Transmit(hspi, (uint16_t*)(READ_REG_COMMAND | ((address & 0x1F) << 8) | 0x00), 16, 100);
	HAL_GPIO_WritePin(&_CS_Port, _CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(&_CS_Port, _CS_Pin, GPIO_PIN_RESET);
	//HAL_SPI_TransmitReceive(hspi, 0x00, (uint16_t*)readbackRegisterValue, 16, 100);
	readbackRegisterValue = ((readbackRegisterValue & 0xFF00) >> 8);
	HAL_GPIO_WritePin(&_CS_Port, _CS_Pin, GPIO_PIN_SET);
	return readbackRegisterValue;
}

uint8_t MA702::writeRegister(uint8_t address, uint8_t value){
	uint16_t readbackRegisterValue;
	HAL_GPIO_WritePin(&_CS_Port, _CS_Pin, GPIO_PIN_RESET);
	//HAL_SPI_Transmit(hspi, (uint16_t*)(WRITE_REG_COMMAND | ((address & 0x1F) << 8) | value), 16, 100);
	HAL_GPIO_WritePin(&_CS_Port, _CS_Pin, GPIO_PIN_SET);
	HAL_Delay(2000);
	HAL_GPIO_WritePin(&_CS_Port, _CS_Pin, GPIO_PIN_RESET);
	//HAL_SPI_TransmitReceive(hspi, 0x00, (uint16_t*)readbackRegisterValue, 16, 100);
	readbackRegisterValue = ((readbackRegisterValue & 0xFF00) >> 8);
	HAL_GPIO_WritePin(&_CS_Port, _CS_Pin, GPIO_PIN_RESET);
	return readbackRegisterValue;
}

double MA702::convertRawAngleToDegree(uint8_t rawAngleDataBitLength, uint16_t rawAngle){
	double angleInDegree;
	angleInDegree = (rawAngle*360.0)/((double)pow(double(2), double(rawAngleDataBitLength)));
	return angleInDegree;
}
double MA702::getVelocity(){
	totalAngle();
	uint32_t prevticks = HAL_GetTick();
	int prevAngle = _last_angle;
	int angle = totalAngle();
	uint32_t currticks = HAL_GetTick();
	if(prevticks>currticks){
		currticks = currticks+(80000000-prevticks);
	}
	double time = (currticks-prevticks)/80000000;
	return (prevAngle-angle)/(time);
}

