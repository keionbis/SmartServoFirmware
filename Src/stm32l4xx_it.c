/**
 ******************************************************************************
 * @file    stm32l4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#include "stm32l4xx_it.h"
#include "eeprom.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

extern uint16_t  devID;
extern float Setpoint;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */
/******************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi3_rx;
extern DMA_HandleTypeDef hdma_spi3_tx;
extern TIM_HandleTypeDef htim7;
extern SPI_HandleTypeDef hspi3;
extern uint16_t _TXData[5];
uint16_t _RxData[5] = {1, 1, 1, 1,1};
extern int controller, prevController;
extern int EndSetPoint;
extern float Pkp, Pki, Pkd, Vkp , Vki , Vkd, Tkp, Tki , Tkd, Out , Pkg, Vkg, Tkg, Pkc, Vkc, Tkc;

/**
 * @brief This function handles Non maskable interrupt.
 */
void initDMA(){
	HAL_SPI_TransmitReceive_DMA(&hspi3, _TXData,_RxData, 5);

}
void NMI_Handler(void)
{
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */

	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
	/* USER CODE BEGIN HardFault_IRQn 1 */

	/* USER CODE END HardFault_IRQn 1 */
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
	/* USER CODE BEGIN MemoryManagement_IRQn 0 */

	/* USER CODE END MemoryManagement_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
		/* USER CODE END W1_MemoryManagement_IRQn 0 */
	}
	/* USER CODE BEGIN MemoryManagement_IRQn 1 */

	/* USER CODE END MemoryManagement_IRQn 1 */
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
	/* USER CODE BEGIN BusFault_IRQn 0 */

	/* USER CODE END BusFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_BusFault_IRQn 0 */
		/* USER CODE END W1_BusFault_IRQn 0 */
	}
	/* USER CODE BEGIN BusFault_IRQn 1 */

	/* USER CODE END BusFault_IRQn 1 */
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
	/* USER CODE BEGIN UsageFault_IRQn 0 */

	/* USER CODE END UsageFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_UsageFault_IRQn 0 */
		/* USER CODE END W1_UsageFault_IRQn 0 */
	}
	/* USER CODE BEGIN UsageFault_IRQn 1 */

	/* USER CODE END UsageFault_IRQn 1 */
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
	/* USER CODE BEGIN SVCall_IRQn 0 */

	/* USER CODE END SVCall_IRQn 0 */
	/* USER CODE BEGIN SVCall_IRQn 1 */

	/* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
	/* USER CODE BEGIN DebugMonitor_IRQn 0 */

	/* USER CODE END DebugMonitor_IRQn 0 */
	/* USER CODE BEGIN DebugMonitor_IRQn 1 */

	/* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
	/* USER CODE BEGIN PendSV_IRQn 0 */

	/* USER CODE END PendSV_IRQn 0 */
	/* USER CODE BEGIN PendSV_IRQn 1 */

	/* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
	/* USER CODE BEGIN SysTick_IRQn 0 */

	/* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
	/* USER CODE BEGIN SysTick_IRQn 1 */

	/* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles EXTI line4 interrupt.
 */
uint16_t DebugArray[500] = {0}, k = 0;
uint16_t DebugTXArray[500] = {0}, h = 0;

int x = 0;
void EXTI4_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI4_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);

	//HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_SET);
	/* USER CODE END EXTI4_IRQn 0 */
	HAL_SPI_TransmitReceive_DMA(&hspi3,_TXData,_RxData, 5);

	switch(_RxData[1]){
	case(0x00):
			prevController = controller;
			controller = 7;
	break;

	case(0x91):
        	controller = 0;
			Setpoint = (int16_t)_RxData[2];

	break;
	case(0x92):
			controller = 1;
			Setpoint = (int16_t)_RxData[2];

	break;
	case(0x93):
			controller = 2;
			Setpoint = (int16_t)_RxData[2];

	break;
	case(0x47):
				Pkp = _RxData[2]/1000;
				Pki = _RxData[3]/1000;
				Pkd = _RxData[4]/1000;

	break;
	case(0x48):
				Vkp = _RxData[2]/1000;
				Vki = _RxData[3]/1000;
				Vkd = _RxData[4]/1000;	break;
	case(0x49):
				Tkp = _RxData[2]/1000;
				Tki = _RxData[3]/1000;
				Tkd = _RxData[4]/1000;	break;

	case(0xA0):
					if(_RxData[3] == 1){
						Pkp = (float)_RxData[2]/100;
					}
					else if(_RxData[3] == 2){
						Vkp = (float)_RxData[2]/100;
					}
					else if(_RxData[3] == 3){
						Tkp = (float)_RxData[2]/100;
					}
	break;
	case(0xA1):
					if(_RxData[3] == 1){
						Pki = (float)_RxData[2]/100;
					}
					else if(_RxData[3] == 2){
						Vki = (float)_RxData[2]/100;
					}
					else if(_RxData[3] == 3){
						Tki = (float)_RxData[2]/100;
					}
	break;
	case(0xA2):
					if(_RxData[3] == 1){
						Pkd = (float)_RxData[2]/100;
					}
					else if(_RxData[3] == 2){
						Vkd = (float)_RxData[2]/100;
					}
					else if(_RxData[3] == 3){
						Tkd = (float)_RxData[2]/100;
					}
	break;
	case(0xA3):
					if(_RxData[3] == 1){
						Pkg = (float)_RxData[2]/100;
					}
					else if(_RxData[3] == 2){
						Vkg = (float)_RxData[2]/100;
					}
					else if(_RxData[3] == 3){
						Tkg = (float)_RxData[2]/100;
					}
	break;
	case(0xA4):
					if(_RxData[3] == 1){
						Pkc = (float)_RxData[2]/100;
					}
					else if(_RxData[3] == 2){
						Vkc = (float)_RxData[2]/100;
					}
					else if(_RxData[3] == 3){
						Tkc = (float)_RxData[2]/100;
					}
	break;

	}


}

/**
 * @brief This function handles DMA1 channel1 global interrupt.
 */
void DMA1_Channel1_IRQHandler(void)
{
	/* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

	/* USER CODE END DMA1_Channel1_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_adc1);
	/* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

	/* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
 * @brief This function handles DMA1 channel2 global interrupt.
 */
void DMA1_Channel2_IRQHandler(void)
{
	/* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

	/* USER CODE END DMA1_Channel2_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_spi1_rx);
	/* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

	/* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
 * @brief This function handles DMA1 channel3 global interrupt.
 */
void DMA1_Channel3_IRQHandler(void)
{
	/* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

	/* USER CODE END DMA1_Channel3_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_spi1_tx);
	/* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

	/* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
 * @brief This function handles TIM7 global interrupt.
 */
void TIM7_IRQHandler(void)
{
	/* USER CODE BEGIN TIM7_IRQn 0 */

	/* USER CODE END TIM7_IRQn 0 */
	HAL_TIM_IRQHandler(&htim7);
	runControllers();
	/* USER CODE BEGIN TIM7_IRQn 1 */

	/* USER CODE END TIM7_IRQn 1 */
}

/**
 * @brief This function handles DMA2 channel1 global interrupt.
 */
void DMA2_Channel1_IRQHandler(void)
{
	/* USER CODE BEGIN DMA2_Channel1_IRQn 0 */

	/* USER CODE END DMA2_Channel1_IRQn 0 */

	HAL_DMA_IRQHandler(&hdma_spi3_rx);
	/* USER CODE BEGIN DMA2_Channel1_IRQn 1 */

	/* USER CODE END DMA2_Channel1_IRQn 1 */
}

/**
 * @brief This function handles DMA2 channel2 global interrupt.
 */
void DMA2_Channel2_IRQHandler(void)
{
	/* USER CODE BEGIN DMA2_Channel2_IRQn 0 */

	/* USER CODE END DMA2_Channel2_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_spi3_tx);
	//	HAL_SPI_TxCpltCallback(&hspi3);

	/* USER CODE BEGIN DMA2_Channel2_IRQn 1 */

	/* USER CODE END DMA2_Channel2_IRQn 1 */
}
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
#ifdef __cplusplus
}
#endif
