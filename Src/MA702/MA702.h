/*
 * MA702.h
 *
 *  Created on: Sep 12, 2018
 *      Author: kbisland
 */

#ifndef INC_MA702_MA702_H_
#define INC_MA702_MA702_H_
#include "stm32l4xx_hal.h"


#define READ_REG_COMMAND    (0b010 << 13)
#define WRITE_REG_COMMAND   (0b100 << 13)

#define SSI_MODE            spi_MODE1

#define MA_SPI_MODE_0       SPI_MODE0
#define MA_SPI_MODE_3       SPI_MODE3

class MA702 {
public:
    int rotations = 0;

	MA702();
	void begin(SPI_HandleTypeDef *hspi1, uint16_t);
	void end();
	double readAngle();
	uint16_t readAngle12();
	signed long long readAngleRaw();
	uint16_t readAngleRaw(bool* error);
	uint16_t readAngleRaw16();
	uint8_t readAngleRaw8();
	uint8_t readRegister(uint8_t address);
	uint8_t writeRegister(uint8_t address, uint8_t value);
	void setSpiClockFrequency(uint32_t speedMaximum);
	void setSpiDataMode(uint8_t spiMode);
	void setSpiChipSelectPin(uint8_t spiChipSelectPin);
	double convertRawAngleToDegree(uint8_t rawAngleDataBitLength, uint16_t rawAngle);
    long int totalAngle();
    double getVelocity();

private:
	int _last_angle = 0;
	int _init_angle = 0;
	unsigned long long _last_ticks = 0;
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef _CS_Port;
	uint16_t _CS_Pin = 0;
};

#endif //MAGALPHA_H
