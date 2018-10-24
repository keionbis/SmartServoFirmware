#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#include "stm32l4xx_it.h"
#include "PID/PID.h"
#include "main.h"
#include "eeprom.h"
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
		hspi->Init.Mode = SPI_MODE_SLAVE;
		HAL_SPI_Init(hspi);
		HAL_GPIO_TogglePin(GPIOA, LED_Pin);
		HAL_SPI_TransmitReceive(hspi, _TxBuffer, _RxBuffer, 5, 0xFF);
		__enable_irq();
		hspi->Init.Mode = SPI_MODE_MASTER;
		HAL_SPI_Init(hspi);
		if(_RxBuffer[0] == *DevID){
			if(_RxBuffer[1] == 0x76){ //Set Values
				pos->setSetPoint((float)_RxBuffer[2]);
				vel->setSetPoint((float)(_RxBuffer[3]/1000));
				torque->setSetPoint((float)(_RxBuffer[4]/1000));
			}
			else if(_RxBuffer[1] == 0x90){
				pos->setTunings(_RxBuffer[2], _RxBuffer[3],_RxBuffer[4]);

				EE_WriteVariable(_VirtAddVarTab[1], (uint32_t)_RxBuffer[2]);
				EE_WriteVariable(_VirtAddVarTab[2], (uint32_t)_RxBuffer[3]);
				EE_WriteVariable(_VirtAddVarTab[3], (uint32_t)_RxBuffer[4]);
			}
			else if(_RxBuffer[1] == 0x91){
				vel->setTunings(_RxBuffer[2], _RxBuffer[3],_RxBuffer[4]);

				EE_WriteVariable(_VirtAddVarTab[4], (uint32_t)_RxBuffer[2]);
				EE_WriteVariable(_VirtAddVarTab[5], (uint32_t)_RxBuffer[3]);
				EE_WriteVariable(_VirtAddVarTab[6], (uint32_t)_RxBuffer[4]);
			}
			else if(_RxBuffer[1] == 0x92){
				torque->setTunings(_RxBuffer[2], _RxBuffer[3],_RxBuffer[4]);

				EE_WriteVariable(_VirtAddVarTab[7], (uint32_t)_RxBuffer[2]);
				EE_WriteVariable(_VirtAddVarTab[8], (uint32_t)_RxBuffer[3]);
				EE_WriteVariable(_VirtAddVarTab[9], (uint32_t)_RxBuffer[4]);
			}

		}
		else if(_RxBuffer[0] == 0x88 ){//Autoset devid
			DevID = &_RxBuffer[1];
			EE_WriteVariable(_VirtAddVarTab[0], (uint32_t)DevID);

		}


	}
	else if(HAL_GPIO_ReadPin(Comms_CS_GPIO_Port, Comms_CS_Pin)){
		HAL_GPIO_TogglePin(GPIOA, LED_Pin);
	}

}
void sharingIsCaring(SPI_HandleTypeDef *_hspi, PID *_pos, PID *_vel, PID *_Torque, uint16_t *_devID){
	hspi = _hspi;
	pos = _pos;
	vel = _vel;
	torque = _Torque;
	DevID = _devID;
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
