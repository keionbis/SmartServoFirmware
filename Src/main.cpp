#include "main.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_flash_ex.h"
#include "stm32l4xx_it.h"
#include "MA702/MA702.h"
#include "PID/PID.h"
#include "eeprom.h"
#include <cmath>

#ifdef __cplusplus
extern "C"
{
#endif

#define Bias 0.05
#define count 2800
#define PWM_Period 800

//Variable Definitions

//EEPROM definitions
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777, 0x8888, 0x9999, 0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE};
uint16_t VarDataTab[NB_OF_VAR] = {0, 0, 0};
uint16_t VarValue = 0;

//HAL DEfinitions
ADC_HandleTypeDef hadc1;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim7;

//Class definitions
PID Position(50, 0.5, 1, 0.00000000038146973);
PID Velocity(0.05, 0, 0, 0.00000000038146973);
PID Torque(0.05, 0, 0, 0.00000000038146973);
MA702 encoder;

//PID constants setup
float Pkp = 8, Pki = 0, Pkd = 0, Vkp = 1, Vki = 0, Vkd = 0, Tkp = 1, Tki = 0, Tkd = 0, Out = 0;

//general Variable definitions
extern uint16_t _RxData[5];
uint16_t _TXData[5] = {1,2,3,4,5};
uint16_t  devID = 3;
uint32_t Data = 0;
float Output, Setpoint, PkpSetpoint = 50;
int controller = 0, EndSetPoint;

//Function definitions
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void SystemClock_Config(void);
static void TIM7_Init(void);
static void GPIO_Init(void);
static void TIM2_Init(void);
static void TIM1_Init(void);
static void SPI1_Init(void);
static void SPI3_Init(void);
static void ADC1_Init(void);
static void Init_Controllers(void);
static void MX_DMA_Init(void);
static void MX_NVIC_Init();
void ReadEEPROM();
int PCLK1_Freq = HAL_RCC_GetPCLK2Freq();
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void AutoTunePIDs(uint8_t type);

int main(void)

{
	GPIO_Init();
	TIM2_Init();
	TIM1_Init();
	HAL_Init();
	SystemClock_Config();
	MX_DMA_Init();

	SPI1_Init();
	ADC1_Init();
	SPI3_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	HAL_FLASH_Unlock();
	EE_Init();
	//ReadEEPROM();
	Init_Controllers();

	//initialize encoder
	encoder.begin(&hspi1, 0);

	//set current encoder as set point
	Setpoint = encoder.totalAngle();
	Setpoint = encoder.totalAngle();
	Position.setSetPoint(Setpoint);

	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	__HAL_TIM_SET_AUTORELOAD(&htim2, PCLK1_Freq / PWM_Period);
	__HAL_TIM_SET_AUTORELOAD(&htim1, PCLK1_Freq / PWM_Period);

	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim1);

	//Position.compute();
	Out = 0;
	GPIOA -> ODR &= ~GPIO_PIN_1;
	GPIOA -> ODR |= GPIO_PIN_10;
	GPIOA -> ODR &= ~GPIO_PIN_12;
	HAL_TIM_MspPostInit(&htim1);
	HAL_TIM_MspPostInit(&htim2);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,0);
	TIM7_Init();
	HAL_TIM_Base_Start(&htim7);

	__HAL_TIM_SET_COUNTER(&htim7, 1);
	__HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);
	while (1) {

	}
}

void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 40;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
	PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
	PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
	PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
	PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

static void TIM7_Init(void)
{
	TIM_MasterConfigTypeDef sMasterConfig;

	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 0;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 2000;
	htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}


}


static void ADC1_Init(void)
{

	ADC_ChannelConfTypeDef sConfig;


	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.NbrOfDiscConversion = 1;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}


	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

static void SPI1_Init(void)
{
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}
static void SPI3_Init(void)
{

	/* SPI3 parameter configuration*/
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_SLAVE;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 7;
	hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

static void TIM1_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 800;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}



}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *huart)
{
	Error_Handler();
}

static void TIM2_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 800;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}


}

static void GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LED_Pin|ENC_CS_Pin|D2_Pin|D1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LED_Pin ENC_CS_Pin D2_Pin D1_Pin */
	GPIO_InitStruct.Pin = LED_Pin|ENC_CS_Pin|D2_Pin|D1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void _Error_Handler(char *file, int line){
	while(1)
	{

	}
}

static void MX_NVIC_Init(void)
{
	/* EXTI4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	/* TIM7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(TIM7_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(TIM7_IRQn);
}

static void MX_DMA_Init(void)
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
	/* DMA2_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);

}



void runControllers(){
	GPIOA->ODR &= ~GPIO_PIN_0;
	if(encoder.totalAngle() == (uint16_t)Setpoint){
		return;
	}
	if(controller ==0 || controller == 1 || controller == 2){
		if(controller == 0){
			Position.setProcessValue(encoder.totalAngle());
			Position.setSetPoint(Setpoint);
			Out = Position.compute();

			_TXData[0] = devID;
			_TXData[1] = 0x91;
			_TXData[2] = encoder.totalAngle();
			if(_TXData[2]<0){
				_TXData[3] = 1 ;
			}
			else{
				_TXData[3] = 0 ;

			}

			//				_TXData[4] = (uint16_t)(Out*1000);

		}
		if(controller == 1){
			Velocity.setSetPoint(Setpoint);
			Velocity.setProcessValue(encoder.getVelocity());
			Out = Position.compute();
			_TXData[2] = encoder.getVelocity();

		}
		if(controller == 2){
			Torque.setSetPoint(Setpoint);
			Torque.setProcessValue(HAL_ADC_GetValue(&hadc1));
			Out = Position.compute();
			_TXData[2] = (uint16_t)(Out*1000);

		}

		if(Out>0){
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PCLK1_Freq / PWM_Period * Out );
		}
		else if (Out<0){
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PCLK1_Freq / PWM_Period * Out*-1);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
		}
		else if(Out == 0){
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,0);
		}
	}
	if(controller ==7){
		_TXData[2] = encoder.totalAngle();
		_TXData[3] = HAL_ADC_GetValue(&hadc1);
	}

	if(controller ==77){
		if(_RxData[3] == 1){
			Pkp = (float)(_RxData[2]/100);
			Position.setTunings(Pkd, Pki, Pkd);
		}
		else if(_RxData[3] == 2){
			Vkp = (float)_RxData[2]/100;
			Position.setTunings(Vkd, Vki, Vkd);
		}
		else if(_RxData[3] == 3){
			Tkp = (float)_RxData[2]/100;
			Position.setTunings(Tkd, Tki, Tkd);
		}


		_TXData[0] = _RxData[0];
		_TXData[1] = _RxData[1];
		_TXData[2] = _RxData[2];
		_TXData[3] = _RxData[3];
		_TXData[4] = _RxData[4];
		controller = 0;
	}

	if(controller == 78){
		if(_RxData[3] == 1){
			Pki = (float)_RxData[2]/100;
			Position.setTunings(Pkd, Pki, Pkd);
		}
		else if(_RxData[3] == 2){
			Vki = (float)_RxData[2]/100;
			Position.setTunings(Vkd, Vki, Vkd);
		}
		else if(_RxData[3] == 3){
			Tki = (float)_RxData[2]/100;
			Position.setTunings(Tkd, Tki, Tkd);
		}


		_TXData[0] = _RxData[0];
		_TXData[1] = _RxData[1];
		_TXData[2] = _RxData[2];
		_TXData[3] = _RxData[3];
		_TXData[4] = _RxData[4];
		controller-=50;
	}
	if(controller == 79){
		if(_RxData[3] == 1){
			Pkd = (float)_RxData[2]/100;
			Position.setTunings(Pkd, Pki, Pkd);
		}
		else if(_RxData[3] == 2){
			Vkd = (float)_RxData[2]/100;
			Position.setTunings(Vkd, Vki, Vkd);
		}
		else if(_RxData[3] == 3){
			Tkd = (float)_RxData[2]/100;
			Position.setTunings(Tkd, Tki, Tkd);
		}


		_TXData[0] = _RxData[0];
		_TXData[1] = _RxData[1];
		_TXData[2] = _RxData[2];
		_TXData[3] = _RxData[3];
		_TXData[4] = _RxData[4];
		controller-=60;
	}
	if(controller == 80){
		if(_RxData[3] == 1){

			Position.setGrav((float)((int16_t)(_RxData[2])/100));
		}
		else if(_RxData[3] == 2){
			Position.setGrav((float)((int16_t)(_RxData[2])/100));
		}
		else if(_RxData[3] == 3){
			Position.setGrav((float)((int16_t)(_RxData[2])/100));
		}


		_TXData[0] = _RxData[0];
		_TXData[1] = _RxData[1];
		_TXData[2] = _RxData[2];
		_TXData[3] = _RxData[3];
		_TXData[4] = _RxData[4];
		controller-=70;
	}
	if(controller == 81){
		if(_RxData[3] == 1){
			Position.setCorr((float)(((int16_t)_RxData[2])/100));
		}
		else if(_RxData[3] == 2){
			Position.setCorr((float)(((int16_t)_RxData[2])/100));
		}
		else if(_RxData[3] == 3){
			Position.setCorr((float)(((int16_t)_RxData[2])/100));
		}

		_TXData[0] = _RxData[0];
		_TXData[1] = _RxData[1];
		_TXData[2] = _RxData[2];
		_TXData[3] = _RxData[3];
		_TXData[4] = _RxData[4];
		controller-=80;
	}


	GPIOA->ODR |= GPIO_PIN_0;

}
static void Init_Controllers(void){
	Position.setInputLimits(0.0, 4096.0);
	Position.setOutputLimits(-1.0, 1.0);
	Position.setTunings(Pkd, Pki, Pkd);
	Position.setBias(Bias);
	Position.setMode(AUTO_MODE);
	Velocity.setInputLimits(-1.660*4096,1.660*4096);//max velocity in rpm* enc tics/rev
	Velocity.setOutputLimits( 1.0, 1.0);
	Velocity.setBias(Bias);
	Velocity.setMode(AUTO_MODE);
	Torque.setInputLimits(0, 4096.0);
	Torque.setOutputLimits(-1.0, 1.0);
	Torque.setBias(Bias);
	Torque.setMode(AUTO_MODE);



}

#ifdef __cplusplus
} // extern "C"
#endif
