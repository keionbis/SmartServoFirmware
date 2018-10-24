#include "main.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_flash_ex.h"
#include "stm32l4xx_it.h"
#include "MA702/MA702.h"
#include "PID/PID.h"
#include "eeprom.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define PWM_Period 50100
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777, 0x8888, 0x9999, 0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE};
uint16_t VarDataTab[NB_OF_VAR] = {0, 0, 0};
uint16_t VarValue = 0;

int x =0;
ADC_HandleTypeDef hadc1;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim7;
PID Position(1, 0, 0, 0.0025);
PID Velocity(1, 0, 0, 0.0025);
PID Torque(1, 0, 0, 0.0025);
MA702 encoder;
uint16_t devID = 0, Pkp = 1, Pki = 0, Pkd = 0, Vkp = 1, Vki = 0, Vkd = 0, Tkp = 1, Tki = 0, Tkd = 0;
uint32_t Data = 0;
float POut = 0, VOut = 0, TOut = 0;
uint16_t angle;

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void SystemClock_Config(void);
static void TIM7_Init(void);
static void GPIO_Init(void);
static void TIM2_Init(void);
static void TIM1_Init(void);
static void SPI1_Init(void);
static void ADC1_Init(void);
static void Init_Controllers(void);
static void MX_DMA_Init(void);
static void MX_NVIC_Init();

int PCLK1_Freq = HAL_RCC_GetPCLK2Freq();

float Output;


int main(void)
{

	HAL_Init();
	SystemClock_Config();
	GPIO_Init();
	TIM2_Init();
	TIM1_Init();
	TIM7_Init();
	SPI1_Init();
	ADC1_Init();
	MX_DMA_Init();

	Init_Controllers();
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start(&htim7);
	MX_NVIC_Init();


	HAL_FLASH_Unlock();

	EE_Init();

	EE_ReadVariable(VirtAddVarTab[1], &Data);
	if(Data == 0){
		EE_WriteVariable(VirtAddVarTab[0], (uint32_t)devID);
		EE_WriteVariable(VirtAddVarTab[1], (uint32_t)Pkp);
		EE_WriteVariable(VirtAddVarTab[2], (uint32_t)Pki);
		EE_WriteVariable(VirtAddVarTab[3], (uint32_t)Pkd);
		EE_WriteVariable(VirtAddVarTab[4], (uint32_t)Vkp);
		EE_WriteVariable(VirtAddVarTab[5], (uint32_t)Vki);
		EE_WriteVariable(VirtAddVarTab[6], (uint32_t)Vki);
		EE_WriteVariable(VirtAddVarTab[7], (uint32_t)Tkp);
		EE_WriteVariable(VirtAddVarTab[8], (uint32_t)Tki);
		EE_WriteVariable(VirtAddVarTab[9], (uint32_t)Tkd);

		//write default PID vals
	}
	else{
		EE_ReadVariable(VirtAddVarTab[0], &Data);
		devID = Data;
		EE_ReadVariable(VirtAddVarTab[1], &Data);
		Pkp = Data;
		EE_ReadVariable(VirtAddVarTab[2], &Data);
		Pki = Data;
		EE_ReadVariable(VirtAddVarTab[3], &Data);
		Pkd = Data;
		EE_ReadVariable(VirtAddVarTab[4], &Data);
		Vkp = Data;
		EE_ReadVariable(VirtAddVarTab[5], &Data);
		Vki = Data;
		EE_ReadVariable(VirtAddVarTab[6], &Data);
		Vkd = Data;
		EE_ReadVariable(VirtAddVarTab[7], &Data);
		Tkp = Data;
		EE_ReadVariable(VirtAddVarTab[8], &Data);
		Tki = Data;
		EE_ReadVariable(VirtAddVarTab[9], &Data);
		Tkd = Data;
	}
	sharingIsCaring(&hspi1, &Position, &Velocity, &Torque, &devID);
	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
	{

	}
	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_TIM_Base_Start(&htim7);
	__HAL_TIM_SET_COUNTER(&htim7, 0);


	__HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);

	__HAL_TIM_SET_AUTORELOAD(&htim2, PCLK1_Freq / PWM_Period);
	__HAL_TIM_SET_AUTORELOAD(&htim1, PCLK1_Freq / PWM_Period);

	GPIOA -> ODR &= ~GPIO_PIN_1;
	GPIOB -> ODR &= ~GPIO_PIN_3;
	GPIOA -> ODR |= GPIO_PIN_12;
	encoder.begin(&hspi1, 0);
	while (1) {

	}
}

void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;


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


	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}


	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);


	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

static void TIM7_Init(void)
{

	TIM_MasterConfigTypeDef sMasterConfig;

	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 50;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 50;
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
	HAL_NVIC_SetPriority(TIM7_IRQn, 0, 1);

	HAL_NVIC_EnableIRQ(TIM7_IRQn);



}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //TIM7_IRQHandler(void)
{
	__disable_irq();
	//run_Controllers();
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	__enable_irq();

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
	GPIO_InitTypeDef GPIO_InitStruct;

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

static void TIM1_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 0;
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

	HAL_TIM_MspPostInit(&htim1);

}

static void TIM2_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 0;
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

	HAL_TIM_MspPostInit(&htim2);

}

static void GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOA, LED_Pin|ENC_CS_Pin|D2_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = LED_Pin|ENC_CS_Pin|D2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = Comms_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(Comms_CS_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = D1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(D1_GPIO_Port, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

static void Init_Controllers(void){

	Position.setInputLimits(-4096.0, 4096.0);
	Position.setOutputLimits(-1.0, 1.0);
	Position.setBias(0.01);
	Position.setMode(1);
	Velocity.setInputLimits(-1.660,1.660);
	Velocity.setOutputLimits( 1.0, 1.0);
	Velocity.setBias(0.01);
	Velocity.setMode(AUTO_MODE);
	Torque.setInputLimits(0, 4096.0);
	Torque.setOutputLimits(-1.0, 1.0);
	Torque.setBias(0.01);
	Torque.setMode(AUTO_MODE);



}
void runControllers(){
__disable_irq();
	Position.setProcessValue(encoder.totalAngle());
	POut = Position.compute();
	Velocity.setProcessValue(Position.getVel());
	Torque.setProcessValue(HAL_ADC_GetValue(&hadc1));
	VOut = Velocity.compute();
	TOut = Torque.compute();
	//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	if(Output<0){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PCLK1_Freq / PWM_Period *POut*-1);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
	}
	else if (Output > 0){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PCLK1_Freq / PWM_Period *POut);
	}
	else if(Output == 0){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,0);
	}
__enable_irq();
}


void _Error_Handler(char *file, int line){
	while(1)
	{
	}
}


static void MX_NVIC_Init(void)
{
	/* TIM7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM7_IRQn);
	/* EXTI4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);


	/* DMA1_Channel2_IRQn interrupt configuration */

}
static void MX_DMA_Init(void)
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

}
#ifdef __cplusplus
} // extern "C"
#endif
