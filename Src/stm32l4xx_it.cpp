#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#include "stm32l4xx_it.h"
#include "PID/PID.h"
#include "main.h"
#include "eeprom.h"
#include "MA702/MA702.h"
#include <string.h> // memcpy

#ifdef __cplusplus
extern "C"
{
#endif

extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern TIM_HandleTypeDef htim7;
uint16_t _TxBuffer[5] = {0}, _RxBuffer[5] = {0};
SPI_HandleTypeDef *hspi;
PID *pos, *vel, *torque;
uint16_t * DevID;
uint16_t _VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777, 0x8888, 0x9999, 0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE};
MA702 *enc;
ADC_HandleTypeDef *ADC;

void NMI_Handler(void)
{

}


void HardFault_Handler(void)
{

	while (1)
	{

	}

}


void MemManage_Handler(void)
{

	while (1)
	{

	}

}


void BusFault_Handler(void)
{
	while (1)
	{

	}

}


void UsageFault_Handler(void)
{
	while (1)
	{

	}

}


void SVC_Handler(void)
{

}


void DebugMon_Handler(void)
{

}

void PendSV_Handler(void)
{

}


void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}


void EXTI4_IRQHandler(){

	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);

	if(!HAL_GPIO_ReadPin(Comms_CS_GPIO_Port, Comms_CS_Pin)){
		__disable_irq();
		HAL_GPIO_TogglePin(GPIOA, LED_Pin);
		int size = sizeof(_RxBuffer)/sizeof(*_RxBuffer);
		HAL_SPI_TransmitReceive(hspi, _TxBuffer, _RxBuffer, size, 0x10*size);
		__enable_irq();
		if(_RxBuffer[0] == *DevID){
			switch(_RxBuffer[1]){
			case (0x47): //Set Position PID Constants
						pos->setTunings(_RxBuffer[2], _RxBuffer[3],_RxBuffer[4]);
			_TxBuffer == _RxBuffer;
			EE_WriteVariable(_VirtAddVarTab[1], (uint32_t)_RxBuffer[2]);
			EE_WriteVariable(_VirtAddVarTab[2], (uint32_t)_RxBuffer[3]);
			EE_WriteVariable(_VirtAddVarTab[3], (uint32_t)_RxBuffer[4]);
			break;
			case (0x48)://Set Velocity PID Constants
						vel->setTunings(_RxBuffer[2], _RxBuffer[3],_RxBuffer[4]);
			_TxBuffer == _RxBuffer;
			EE_WriteVariable(_VirtAddVarTab[4], (uint32_t)_RxBuffer[2]);
			EE_WriteVariable(_VirtAddVarTab[5], (uint32_t)_RxBuffer[3]);
			EE_WriteVariable(_VirtAddVarTab[6], (uint32_t)_RxBuffer[4]);
			break;
			case (0x49)://Set Torque PID Constants
						vel->setTunings(_RxBuffer[2], _RxBuffer[3],_RxBuffer[4]);
			_TxBuffer == _RxBuffer;
			EE_WriteVariable(_VirtAddVarTab[7], (uint32_t)_RxBuffer[2]);
			EE_WriteVariable(_VirtAddVarTab[8], (uint32_t)_RxBuffer[3]);
			EE_WriteVariable(_VirtAddVarTab[9], (uint32_t)_RxBuffer[4]);
			break;
			case (0x91)://Set Position setpoint
			    		pos->setSetPoint((float)_RxBuffer[2]);
			*_TxBuffer = *_RxBuffer;
			_TxBuffer[3] = (int)enc->totalAngle();
			controller = 0;
			break;
			case (0x92)://Set Velocity setpoint
						vel->setSetPoint((float)_RxBuffer[2]);
			controller = 1;
			*_TxBuffer = *_RxBuffer;
			_TxBuffer[3] = (int)enc->getVelocity();
			break;
			case (0x93)://Set Torque setpoint
						torque->setSetPoint((float)_RxBuffer[2]);
			controller = 2;
			*_TxBuffer = *_RxBuffer;
			_TxBuffer[3] =HAL_ADC_GetValue(ADC);
			break;
			case (0x22): //Autoset devid
			    		DevID = &_RxBuffer[1];
			EE_WriteVariable(_VirtAddVarTab[0], (uint32_t)DevID);
			_RxBuffer[1] = _RxBuffer[1]+1;
			*_TxBuffer = *_RxBuffer;
			break;

			}
		}
		else if(_RxBuffer[0] == 0x22 ){//Autoset devid
			DevID = &_RxBuffer[1];
			_RxBuffer[1] = _RxBuffer[1]+1;

			EE_WriteVariable(_VirtAddVarTab[0], (uint32_t)DevID);

		}
		else{
			*_TxBuffer = *_RxBuffer;//Set previous received data to next cycles transmit data
		}


	}
	else if(HAL_GPIO_ReadPin(Comms_CS_GPIO_Port, Comms_CS_Pin)){
		HAL_GPIO_TogglePin(GPIOA, LED_Pin);
	}

}
void sharingIsCaring(SPI_HandleTypeDef *_hspi, PID *_pos, PID *_vel, PID *_Torque, uint16_t *_devID, MA702 *_enc, ADC_HandleTypeDef *_ADC){
	hspi = _hspi;
	pos = _pos;
	vel = _vel;
	torque = _Torque;
	DevID = _devID;
	enc = _enc;
	ADC = _ADC;
}

void DMA1_Channel2_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_spi1_rx);
}


void DMA1_Channel3_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_spi1_tx);
}


void TIM7_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim7);
	runControllers();
}
void DMA1_Channel1_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_adc1);
}

#ifdef __cplusplus

}
#endif
