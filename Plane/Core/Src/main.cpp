/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "RcChannel.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern RcChannel thr_rc, elev_rc, ail_rc, rud_rc, switch_rc;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
/*class RcChannel
{
	private:
		uint16_t channel_max_value = 0;
		uint16_t channel_mid_value = 0;
		uint16_t channel_min_value = 0;
		uint32_t IC_Val1 = 0;
		uint32_t IC_Val2 = 0;
		uint32_t Difference = 0;
		uint8_t Is_First_Captured = 0;
		uint32_t tim_channel = 0;
		TIM_HandleTypeDef *htim;
	public:
		void pulseWidthCalc();
		uint32_t getPulseWidthDif();
		uint32_t getPulseWidth();
		uint32_t getChannelMaxWidth();
		uint32_t getChannelMinWidth();
		uint32_t getChannelMidWidth();
		RcChannel(TIM_HandleTypeDef *htim, uint8_t channel_num, uint16_t channel_min_value, uint16_t channel_max_value);
		uint8_t matchMinValue();
		uint8_t matchMaxValue();
		~RcChannel();

};
RcChannel::RcChannel(TIM_HandleTypeDef *htim, uint8_t channel_num, uint16_t channel_min_value, uint16_t channel_max_value)
{
	this->channel_min_value = channel_min_value;
	this->channel_max_value = channel_max_value;
	this->channel_mid_value = channel_min_value + (channel_max_value - channel_min_value)/2;

	this->htim = htim;
	switch(channel_num)
	{
		case 1: tim_channel = TIM_CHANNEL_1; break;
		case 2: tim_channel = TIM_CHANNEL_2; break;
		case 3: tim_channel = TIM_CHANNEL_3; break;
		case 4: tim_channel = TIM_CHANNEL_4; break;
	}
}
RcChannel::~RcChannel()
{

}

uint32_t RcChannel::getPulseWidth()
{
	return Difference;
}
uint32_t RcChannel::getChannelMaxWidth()
{
	return channel_max_value;
}
uint32_t RcChannel::getChannelMinWidth()
{
	return channel_min_value;
}
uint32_t RcChannel::getChannelMidWidth()
{
	return channel_mid_value;
}
void RcChannel::pulseWidthCalc()
{
	if (Is_First_Captured==0) // if the first value is not captured
	{
		IC_Val1 = HAL_TIM_ReadCapturedValue(this->htim, tim_channel); // read the first value
		Is_First_Captured = 1;  // set the first captured as true
		// Now change the polarity to falling edge
		__HAL_TIM_SET_CAPTUREPOLARITY(this->htim, tim_channel, TIM_INPUTCHANNELPOLARITY_FALLING);
	}
	else
		if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(this->htim, tim_channel);  // read second value
			//__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}
			Is_First_Captured = 0; // set it back to false
			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(this->htim, tim_channel, TIM_INPUTCHANNELPOLARITY_RISING);
		}
}
uint32_t RcChannel::getPulseWidthDif()
{
	return  ((int16_t)(this->channel_mid_value) - ((int16_t)(this->Difference) - (int16_t)(this->channel_mid_value)));
}
uint8_t RcChannel::matchMinValue()
{
	if( (this->getPulseWidth() > (this->channel_min_value - 4)) && (this->getPulseWidth() < (this->channel_min_value + 4)) )
	{
		return 1;
	} else
	{
		return 0;
	}
}
uint8_t RcChannel::matchMaxValue()
{
	if( (this->getPulseWidth() > (this->channel_max_value - 4)) && (this->getPulseWidth() < (this->channel_max_value + 4)) )
	{
		return 1;
	} else
	{
		return 0;
	}
}
RcChannel thr_rc(&htim2, 1, 885, 1850), elev_rc(&htim2, 2, 1080, 1863),
		  ail_rc(&htim2, 3, 1083, 1863), rud_rc(&htim2, 4, 1080, 1863),
		  switch_rc(&htim5, 1, 1080, 1863);

void IcHandlerTim2(TIM_HandleTypeDef *htim)
{
	switch ( (uint8_t) htim->Channel )
	{
		case HAL_TIM_ACTIVE_CHANNEL_1:
		{
			thr_rc.pulseWidthCalc();
		} break;
		case HAL_TIM_ACTIVE_CHANNEL_2:
		{
			elev_rc.pulseWidthCalc();
		} break;
		case HAL_TIM_ACTIVE_CHANNEL_3:
		{
			ail_rc.pulseWidthCalc();
		} break;
		case HAL_TIM_ACTIVE_CHANNEL_4:
		{
			rud_rc.pulseWidthCalc();
		} break;
	}
}*/
void IcHandlerTim5(TIM_HandleTypeDef *htim)
{
	switch ( (uint8_t) htim->Channel )
	{
		case HAL_TIM_ACTIVE_CHANNEL_1:
		{
			switch_rc.pulseWidthCalc();
		} break;
		case HAL_TIM_ACTIVE_CHANNEL_2:
		{

		} break;
		case HAL_TIM_ACTIVE_CHANNEL_3:
		{

		} break;
		case HAL_TIM_ACTIVE_CHANNEL_4:
		{

		} break;
	}
}

class Servo
{
	private:
		//значение скважности, при котором серво поворачивается на 180
		uint16_t max_PWM_value;

		//значение скважности, при котором серво поворачивается на 0
		uint16_t min_PWM_value;

		//таймер
		TIM_TypeDef* TIM;

		//канал таймера
		uint8_t channel;

		//максимальный угол поворота сервопривода
		uint16_t max_Angle;

	public:
		Servo(TIM_TypeDef* TIM, uint8_t channel, uint16_t min_PWM_value, uint16_t max_PWM_value, uint16_t max_Angle);
		Servo(TIM_TypeDef* TIM, uint8_t channel, uint16_t min_PWM_value, uint16_t max_PWM_value);
		Servo(TIM_TypeDef* TIM, uint8_t channel);
		//Поварачивает сервопривод на заданный угол
		void Set_Position(uint8_t position);
		void setPositionMicroSeconds(uint32_t position);
};
Servo::Servo(TIM_TypeDef* TIM, uint8_t channel, uint16_t min_PWM_value, uint16_t max_PWM_value, uint16_t max_Angle)
{
	this->TIM = TIM;
	this->channel = channel;
	this->min_PWM_value = min_PWM_value;
	this->max_PWM_value = max_PWM_value;
	this->max_Angle = max_Angle;
}

Servo::Servo(TIM_TypeDef* TIM, uint8_t channel, uint16_t min_PWM_value, uint16_t max_PWM_value)
{
	this->TIM = TIM;
	this->channel = channel;
	this->min_PWM_value = min_PWM_value;
	this->max_PWM_value = max_PWM_value;
	this->max_Angle = 180;
}
Servo::Servo(TIM_TypeDef* TIM, uint8_t channel)
{
	this->TIM = TIM;
	this->channel = channel;
	this->min_PWM_value = 0;
	this->max_PWM_value = 0;
	this->max_Angle = 0;
}
void Servo::setPositionMicroSeconds(uint32_t position)
{
	switch (this->channel)
	{
		case 1:
			this->TIM->CCR1 = position;
			break;
		case 2:
			this->TIM->CCR2 = position;
			break;
		case 3:
			this->TIM->CCR3 = position;
			break;
		case 4:
			this->TIM->CCR4 = position;
			break;
	}
}
void Servo::Set_Position(uint8_t position)
{
  double multiplier = (double)(this->max_PWM_value - this->min_PWM_value)/this->max_Angle;

	if(position > this->max_Angle)
		position = this->max_Angle;

	uint16_t pwm = min_PWM_value + multiplier * position;

	switch (this->channel)
	{
		case 1:
			this->TIM->CCR1 = pwm;
			break;
		case 2:
			this->TIM->CCR2 = pwm;
			break;
		case 3:
			this->TIM->CCR3 = pwm;
			break;
		case 4:
			this->TIM->CCR4 = pwm;
			break;
	}
}

class Beeper
{
	private:
		GPIO_TypeDef * port = 0;
		uint32_t pin = 0;
		const uint16_t LONG_BEEP_TIME_MS = 1000;
		const uint16_t SHORT_BEEP_TIME_MS = 100;

	public:
		Beeper(GPIO_TypeDef * port, uint32_t pin);
		~Beeper();
		void shortBeep();
		void longBeep();
		void seriesBeep();

};
Beeper::Beeper(GPIO_TypeDef * port, uint32_t pin)
{
	this->port = port;
	this->pin = pin;
}
Beeper::~Beeper()
{
}
void Beeper::shortBeep()
{
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
	HAL_Delay(SHORT_BEEP_TIME_MS);
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}
void Beeper::longBeep()
{
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
	HAL_Delay(LONG_BEEP_TIME_MS);
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}
void Beeper::seriesBeep()
{
	shortBeep();
	HAL_Delay(SHORT_BEEP_TIME_MS);
	shortBeep();
	HAL_Delay(SHORT_BEEP_TIME_MS);
	shortBeep();
}

uint8_t Armed(Beeper* beeper)
{
	static uint8_t flag = 0;
	if(thr_rc.matchMinValue() && rud_rc.matchMinValue() && elev_rc.matchMinValue() && ail_rc.matchMaxValue())
	{
		flag = 1;
		beeper->longBeep();
		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	}
	if(thr_rc.matchMinValue() && rud_rc.matchMaxValue() && elev_rc.matchMinValue() && ail_rc.matchMinValue())
	{
		flag = 0;
		beeper->longBeep();
		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
	}
	return flag;
}
uint8_t ERSarming(Beeper* beeper)
{
	static uint8_t flag = 0;
	if(thr_rc.matchMinValue() && rud_rc.matchMinValue() && elev_rc.matchMinValue() && ail_rc.matchMinValue())
	{
		flag = 1;
		beeper->shortBeep();
		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	}
	if(thr_rc.matchMinValue() && rud_rc.matchMaxValue() && elev_rc.matchMinValue() && ail_rc.matchMaxValue())
	{
		flag = 0;
		beeper->shortBeep();
		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
	}
	return flag;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_RegisterCallback(&htim2, HAL_TIM_IC_CAPTURE_CB_ID, IcHandlerTim2);
  	HAL_TIM_RegisterCallback(&htim5, HAL_TIM_IC_CAPTURE_CB_ID, IcHandlerTim5);

  	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);//PA5 thr input
  	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);//PB3 elev input
  	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);//PB10 ail input
  	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);//PB11 rud input
  	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);//PA0 switch input

  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);//PA6 thr output
  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);//PA7 elev servo output
  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);//PB0 ail servo 1 output
  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);//PB1 ail servo 2 output
  	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);//PA3 rud servo 2 output
  	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);//PA2 ers servo 2 output

  	char str[32];
  	Servo 	thr_servo(htim3.Instance, 1), elev_servo(htim3.Instance, 2),
  			ail_servo_1(htim3.Instance, 3), ail_servo_2(htim3.Instance, 4),
			rud_servo(htim5.Instance, 4), ers_servo(htim5.Instance, 3);

  	uint32_t ers_servo_set_up_position = 1600;
  	uint8_t ers_match_counter = 0;
  	ers_servo.setPositionMicroSeconds(ers_servo_set_up_position);

  	Beeper beeper(GPIOD, GPIO_PIN_13);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		while(Armed(&beeper))
		{

			thr_servo.setPositionMicroSeconds(thr_rc.getPulseWidth());
			elev_servo.setPositionMicroSeconds(elev_rc.getPulseWidthDif());
			ail_servo_1.setPositionMicroSeconds(ail_rc.getPulseWidthDif());
			ail_servo_2.setPositionMicroSeconds(ail_rc.getPulseWidthDif());
			rud_servo.setPositionMicroSeconds(rud_rc.getPulseWidth());

			if(switch_rc.getPulseWidth() > 1500)
			{
				thr_servo.setPositionMicroSeconds(thr_rc.getChannelMinWidth());
				HAL_Delay(1000);
				ers_servo.setPositionMicroSeconds(540);
			}
			while(switch_rc.getPulseWidth() > 1500)
			{
				beeper.seriesBeep();
			}
		}
		elev_servo.setPositionMicroSeconds(elev_rc.getPulseWidthDif());
		ail_servo_1.setPositionMicroSeconds(ail_rc.getPulseWidthDif());
		ail_servo_2.setPositionMicroSeconds(ail_rc.getPulseWidthDif());
		rud_servo.setPositionMicroSeconds(rud_rc.getPulseWidth());
		thr_servo.setPositionMicroSeconds(thr_rc.getChannelMinWidth());

		while(ERSarming(&beeper))
		{
			elev_servo.setPositionMicroSeconds(elev_rc.getChannelMidWidth());
			ail_servo_1.setPositionMicroSeconds(ail_rc.getChannelMidWidth());
			ail_servo_2.setPositionMicroSeconds(ail_rc.getChannelMidWidth());
			rud_servo.setPositionMicroSeconds(rud_rc.getChannelMidWidth());
			ers_servo.setPositionMicroSeconds(ers_servo_set_up_position);
			if(elev_rc.matchMaxValue())
			{
				ers_match_counter++;
				switch (ers_match_counter)
				{
					case 1: ers_servo_set_up_position = 1000; break;
					case 2: ers_servo_set_up_position = 1600; break;
				}
				HAL_Delay(500);
			}
			if(elev_rc.matchMinValue() && !rud_rc.matchMaxValue())
			{
				ers_servo_set_up_position = 540;
				ers_match_counter = 0;
				HAL_Delay(500);
			}
			//HAL_UART_Transmit(&huart2, (uint8_t*)str, sprintf(str, "%d ", ers_servo_set_up_position), 1000);
		}

		//#define DEBUG
		/*#if def DEBUG
			ail_servo_1.setPositionMicroSeconds(ail_rc.getPulseWidth());
			ail_servo_2.setPositionMicroSeconds(ail_rc.getPusleWidthDif());
			elev_servo.setPositionMicroSeconds(elev_rc.getPulseWidth());
			rud_servo.setPositionMicroSeconds(rud_rc.getPulseWidth());
			ers_servo.setPositionMicroSeconds(switch_rc.getPulseWidth());
			thr_servo.setPositionMicroSeconds(thr_rc.getPulseWidth());

			HAL_UART_Transmit(&huart2, (uint8_t*)str, sprintf(str, "%d ", thr_rc.getPulseWidth()), 1000);
			HAL_UART_Transmit(&huart2, (uint8_t*)str, sprintf(str, "%d ", elev_rc.getPulseWidth()), 1000);
			HAL_UART_Transmit(&huart2, (uint8_t*)str, sprintf(str, "%d ", ail_rc.getPulseWidth()), 1000);
			HAL_UART_Transmit(&huart2, (uint8_t*)str, sprintf(str, "%d ", rud_rc.getPulseWidth()), 1000);
			HAL_UART_Transmit(&huart2, (uint8_t*)str, sprintf(str, "%d\n", switch_rc.getPulseWidth()), 1000);
		#endif*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65536;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 22000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 48;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 22000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
