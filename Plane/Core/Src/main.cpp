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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <stdio.h>
#include "Servo/Servo.h"
#include "Beeper/Beeper.h"
#include "Baro/MS5611.h"
#include "AirSpeed/MS4525DO.h"
#include "Gyro/bno055.h"
#include "PIReg/PIReg.h"
#include "Beta/P3002.h"
#include "PWMCapturer/PWMCapturer.h"
#include "Gps/Gps.h"
#include "Modes/Nav/Nav.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//--------------------DEBUG DEFINES-----------------------

#define BARO_DEBUG 0
#define GYRO_DEBUG 1
#define AIR_DEBUG 2
#define RADIO_DEBUG 3
#define BETA_DEBUG 4

//#define DEBUG_MODE AIR_DEBUG//раскоментить для отладки. присвоить одно из значений выше

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
enum Channels
{
	THR,
	ELEV,
	AIL1,
	AIL2,
	RUD,
	SWITCHA,
	SLIDER,
	CHANNELS_ARRAY_SIZE,
};
enum Sensors
{
	BARO,
	AIR,
	GYROX,
	GYROY,
	GYROZ,
	BAROVY,
	NZ,
	TETA,
	GAMMA,
	PSI,
	LATITUDE,
	LONGITUDE,
	GPS_SPEED,
	COURSE,
	GPS_VALID,
	SENSOR_ARRAY_SIZE, //этот элемент всегда должен быть последним в енуме
};
enum Modes
{
	PREFLIGHTCHECK,
	DIRECT,
	OMEGA_STAB,
	COMMAND,
	DIRECT_FLAPS,
	OMEGA_STAB_K_TUNE,
	OMEGA_STAB_I_TUNE,
	VY_STAB_K_TUNE,
	NAV,
};
enum Logs
{
	OMEGA_X_ZAD,
	OMEGA_Y_ZAD,
	OMEGA_Z_ZAD,
	K_PR_OMEGA_X,
	K_INT_OMEGA_X,
	K_PR_OMEGA_Y,
	K_INT_OMEGA_Y,
	K_PR_OMEGA_Z,
	K_INT_OMEGA_Z,
	VY_ZAD,
	ALT_FILTERED,
	OMEGA_TURN_ZAD,
	GAMMA_ZAD,
	LOG_ARRAY_SIZE,
};

enum Omega_targets
{
	X,
	Y,
	Z,
	OMEGA_ARRAY_SIZE,
};

//--------------------Threads-----------------------

/* Definitions for defaultTask */
extern osThreadId_t defaultTaskHandle;
extern const osThreadAttr_t defaultTask_attributes;

/* Definitions for sensorsUpdate */
extern osThreadId_t sensorsUpdateHandle;
extern const osThreadAttr_t sensorsUpdate_attributes;

/* Definitions for modeUpdate */
extern osThreadId_t modeUpdateHandle;
extern const osThreadAttr_t modeUpdate_attributes;

/* Definitions for radioInputUpdat */
extern osThreadId_t radioInputUpdatHandle;
extern const osThreadAttr_t radioInputUpdat_attributes;

/* Definitions for actuatorsUpdate */
extern osThreadId_t actuatorsUpdateHandle;
extern const osThreadAttr_t actuatorsUpdate_attributes;

/* Definitions for loggerUpdate */
extern osThreadId_t loggerUpdateHandle;
extern const osThreadAttr_t loggerUpdate_attributes;

/* Definitions for baroUpdate */
extern osThreadId_t baroUpdateHandle;
extern const osThreadAttr_t baroUpdate_attributes;

//-------------------My Global VARs--------------------------
const uint16_t WHAIT_FOR_RADIO_MS = 5000;

const uint8_t TUNE_K_P = 1;
const uint8_t TUNE_K_I = 2;
const uint8_t TUNE_OFF = 0;

uint8_t g_activate_flaps = 0;

double k_pr_omega_x = 7.6;
double k_int_omega_x = 5.3;
double k_pr_omega_z = 7.6;
double k_int_omega_z = 5.3;
double k_pr_omega_y = 8.0;
double k_int_omega_y = 6.0;


double k_pr_Vy = 8.5;
double k_int_Vy = 0.0;

const uint8_t num_of_coeff_ref_points = 5;

double speed_ref_points[num_of_coeff_ref_points] = {5.0, 10.0, 15.0, 20.0, 30.0};

double k_pr_omega_x_points[num_of_coeff_ref_points] = {8.0, 6.0, 5.0, 4.0, 1.5};
double k_int_omega_x_points[num_of_coeff_ref_points] = {7.5, 7.0, 5.5, 4.5, 2.0};

double k_pr_omega_z_points[num_of_coeff_ref_points] = {8.5, 6.0, 5.0, 4.5, 2.0};
double k_int_omega_z_points[num_of_coeff_ref_points] = {7.5, 7.0, 5.5, 4.5, 2.0};

double k_pr_omega_y_points[num_of_coeff_ref_points] = {9.5, 7.0, 5.5, 4.5, 2.0};
double k_int_omega_y_points[num_of_coeff_ref_points] = {7.5, 6.0, 5.5, 4.5, 1.5};
//------------------------RC---------------------------------
//extern RcChannel thr_rc, elev_rc, ail_rc, rud_rc, switch_rc, slider_rc;

PWMCapturer thr_rc(&htim2, 1, 989, 1500, 2012, 5),
			elev_rc(&htim2, 2, 989, 1500, 2012, 5),
			ail_rc(&htim4, 1, 989, 1500, 2012, 5),
			rud_rc(&htim4, 2, 989, 1500, 2012, 5),
			switch_rc(&htim5, 1, 989, 2012, 5),
			slider_rc(&htim5, 2, 989, 1500, 2012, 5);

void IcHandlerTim2(TIM_HandleTypeDef *htim)
{
	switch ( (uint8_t) htim->Channel )
	{
		case HAL_TIM_ACTIVE_CHANNEL_1:
		{
			thr_rc.calculatePulseWidth();
		} break;
		case HAL_TIM_ACTIVE_CHANNEL_2:
		{
			elev_rc.calculatePulseWidth();
		} break;
		case HAL_TIM_ACTIVE_CHANNEL_3:
		{

		} break;
		case HAL_TIM_ACTIVE_CHANNEL_4:
		{

		} break;
	}
}

void IcHandlerTim5(TIM_HandleTypeDef *htim)
{
	switch ( (uint8_t) htim->Channel )
	{
		case HAL_TIM_ACTIVE_CHANNEL_1:
		{
			switch_rc.calculatePulseWidth();
		} break;
		case HAL_TIM_ACTIVE_CHANNEL_2:
		{
			slider_rc.calculatePulseWidth();
		} break;
		case HAL_TIM_ACTIVE_CHANNEL_3:
		{

		} break;
		case HAL_TIM_ACTIVE_CHANNEL_4:
		{

		} break;
	}
}

void IcHandlerTim4(TIM_HandleTypeDef *htim)
{
	switch ( (uint8_t) htim->Channel )
	{
		case HAL_TIM_ACTIVE_CHANNEL_1:
		{
			ail_rc.calculatePulseWidth();
		} break;
		case HAL_TIM_ACTIVE_CHANNEL_2:
		{
			rud_rc.calculatePulseWidth();
		} break;
		case HAL_TIM_ACTIVE_CHANNEL_3:
		{

		} break;
		case HAL_TIM_ACTIVE_CHANNEL_4:
		{

		} break;
	}
}

//GPS-----------------------------------------------------------
Gps gps = Gps(&huart3, GPIOE, GPIO_PIN_7);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart3)
		gps.UartCallback();
}
//-----------------------------------------------------------

Servo 	thr_servo(&htim3, 1),
		elev_servo(&htim3, 2),
		ail_servo_1(&htim3, 3),
		ail_servo_2(&htim3, 4),
		rud_servo(&htim5, 4),
		ers_servo(&htim5, 3);

int g_flaperon_delta = 0;

//----------------------INIT-------------------------------

uint32_t output[5] = {0};
uint32_t rc_input[CHANNELS_ARRAY_SIZE] = {1500};
double data_input[SENSOR_ARRAY_SIZE] = {0.0};
double logger_data[LOG_ARRAY_SIZE] = {0.0};
double omega_target[OMEGA_ARRAY_SIZE] = {0.0};

char str[150] = "\0";

uint8_t current_mode = 0;
uint8_t integral_reset_flag = 0;
//-----------------------------------------------------------
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);
void sensorsUpdateTask(void *argument);
void modeUpdateTask(void *argument);
void radioInputUpdateTask(void *argument);
void loggerUpdateTask(void *argument);
void actuatorsUpdateTask(void *argument);
void baroUpdateTask(void *argument);

/* USER CODE BEGIN PFP */
void flapsUpdate(uint8_t activate_flaps)
{
	int flaperon_delta_limit = 340; // 2/6 from the whole range

	if (activate_flaps) //need to replace with flag from RC
	{
		g_flaperon_delta += 1;
	}
	else
	{
		g_flaperon_delta += -1;
	}

	//limitations
	if (g_flaperon_delta >= flaperon_delta_limit)
	{
		g_flaperon_delta = flaperon_delta_limit;
	}
	else
		if (g_flaperon_delta <= 0)
		{
			g_flaperon_delta = 0;
		}
}
void updateRcInput()
{
	rc_input[THR] = thr_rc.getPulseWidth();
	rc_input[ELEV] = elev_rc.getPulseWidthDif();//dif
	rc_input[AIL1] = ail_rc.getPulseWidth();
	rc_input[AIL2] = ail_rc.getPulseWidth();//dif
	rc_input[RUD] = rud_rc.getPulseWidthDif();
	rc_input[SWITCHA] = switch_rc.getPulseWidth();
	rc_input[SLIDER] = slider_rc.getPulseWidth();
}
void updateActuators()
{
	thr_servo.setPositionMicroSeconds(output[THR]);
	elev_servo.setPositionMicroSeconds(output[ELEV]);
	ail_servo_1.setPositionMicroSeconds(output[AIL1]);
	ail_servo_2.setPositionMicroSeconds(output[AIL2]);
	rud_servo.setPositionMicroSeconds(output[RUD]);
}
void preFlightCheckUpdate()
{
	output[THR] = 989;
	output[ELEV] = rc_input[ELEV];
	output[AIL1] = rc_input[AIL1];
	output[AIL2] = rc_input[AIL2];
	output[RUD] = rc_input[RUD];
}
void directUpdate()
{
	output[THR] = rc_input[THR];
	output[ELEV] = rc_input[ELEV];
	output[AIL1] = rc_input[AIL1]-g_flaperon_delta;
	output[AIL2] = rc_input[AIL2]+g_flaperon_delta;;
	output[RUD] = rc_input[RUD];
}

void stabOmegaTgtCalc(void)
{
	omega_target[X] = -(0.234375*rc_input[AIL2] - 351.5625);
	omega_target[Y] = -(0.234375*rc_input[RUD] - 351.5625);
	omega_target[Z] = (0.234375*rc_input[ELEV] - 350.0625);
}

void stabOmegaUpdate()
{
	//------------------Regulators INIT------------------------
	double int_lim_omega_z = 1000, int_lim_omega_x = 1000, int_lim_omega_y = 1000;
	//double omega_zad_x = 0, omega_zad_y = 0, omega_zad_z = 0;
	static PIReg omega_x_PI_reg(k_pr_omega_x, k_int_omega_x, 0.01, int_lim_omega_x, speed_ref_points, k_pr_omega_x_points, k_int_omega_x_points, num_of_coeff_ref_points);
	static PIReg omega_y_PI_reg(k_pr_omega_y, k_int_omega_y, 0.01, int_lim_omega_y, speed_ref_points, k_pr_omega_y_points, k_int_omega_y_points, num_of_coeff_ref_points);
	static PIReg omega_z_PI_reg(k_pr_omega_z, k_int_omega_z, 0.01, int_lim_omega_z, speed_ref_points, k_pr_omega_z_points, k_int_omega_z_points, num_of_coeff_ref_points);
	//---------------------------------------------------------

	if(integral_reset_flag)
	{
		omega_x_PI_reg.integralReset();
		omega_y_PI_reg.integralReset();
		omega_z_PI_reg.integralReset();
		integral_reset_flag = 0;
	}
	//omega_zad_x = -(0.234375*rc_input[AIL2] - 351.5625);
	//omega_zad_y = (0.234375*rc_input[RUD] - 351.5625);
	//omega_zad_z = (0.234375*rc_input[ELEV] - 350.0625);
	logger_data[OMEGA_X_ZAD] = omega_target[X];
	logger_data[OMEGA_Y_ZAD] = omega_target[Y];
	logger_data[OMEGA_Z_ZAD] = omega_target[Z];


	output[THR] = rc_input[THR];
	output[ELEV] = (int)(1500+0.4*omega_z_PI_reg.getOutput());
	output[AIL1] = (int)(1500-0.4*omega_x_PI_reg.getOutput());
	output[AIL2] = (int)(1500-0.4*omega_x_PI_reg.getOutput());
	output[RUD] = (int)(1500-0.4*omega_y_PI_reg.getOutput());


	omega_x_PI_reg.calcGainParams(data_input[AIR]); //airspeed hardcoded due to sensor fault
	omega_x_PI_reg.setError(omega_target[X] - data_input[GYROX]);
	omega_x_PI_reg.calcOutput();

	omega_y_PI_reg.calcGainParams(data_input[AIR]); //airspeed hardcoded due to sensor fault
	omega_y_PI_reg.setError(omega_target[Y] - data_input[GYROY]);
	omega_y_PI_reg.calcOutput();

	omega_z_PI_reg.calcGainParams(data_input[AIR]); //airspeed hardcoded due to sensor fault
	omega_z_PI_reg.setError(omega_target[Z] - data_input[GYROZ]);
	omega_z_PI_reg.calcOutput();

	logger_data[K_PR_OMEGA_X] = omega_x_PI_reg.getProportGain();
	logger_data[K_INT_OMEGA_X] = omega_x_PI_reg.getIntGain();

	logger_data[K_PR_OMEGA_Y] = omega_y_PI_reg.getProportGain();
	logger_data[K_INT_OMEGA_Y] = omega_y_PI_reg.getIntGain();

	logger_data[K_PR_OMEGA_Z] = omega_z_PI_reg.getProportGain();
	logger_data[K_INT_OMEGA_Z] = omega_z_PI_reg.getIntGain();

}

void commandModeUpdate(double omega_turn_tgt, double vy_tgt)
{
	//------------------Local vars INIT------------------------
	double rad2deg = 57.2958;
	double deg2rad = 1/rad2deg;
	double g = 9.81;

	//double omega_turn_tgt = 0.0;
	double gamma_tgt = 0.0;
	double k_pr_gamma = 1.5;

	double int_lim_Vy = 1000;
	//double vy_tgt = 0.0;

	double omega_x_roll_tgt = 0.0; // component of omega_x target from target roll angle
	double omega_x_turn_tgt = 0.0; // component of omega_x target from coordinated turn

	double omega_y_turn_tgt = 0.0; // component of omega_y target from coordinated turn

	double omega_z_turn_tgt = 0.0; // component of omega_z target from coordinated turn
	double omega_z_vy_tgt = 0.0; // component of omega_z target from vertical speed stab

	//------------------Regulators INIT------------------------
	static PIReg vy_PI_reg(k_pr_Vy, k_int_Vy, 0.01, int_lim_Vy);

	//---------------------------------------------------------

	//---------------Vertical speed stab-----------------------
	if (abs(vy_tgt) < 0.2) //to set zero when the stick is in neutral
	{
		vy_tgt = 0;
	}

	logger_data[VY_ZAD] = vy_tgt;

	vy_PI_reg.setError(vy_tgt - data_input[BAROVY]);
	vy_PI_reg.calcOutput();
	//---------------------------------------------------------

	//--------------Omega and Roll target calc-----------------

	if (abs(omega_turn_tgt) < 1.0) //to set zero when the stick is in neutral
	{
		omega_turn_tgt = 0;
	}
	logger_data[OMEGA_TURN_ZAD] = omega_turn_tgt;

	gamma_tgt = -atan((data_input[AIR]*omega_turn_tgt*deg2rad)/g);
	gamma_tgt = gamma_tgt*rad2deg;
	logger_data[GAMMA_ZAD] = gamma_tgt;
	//---------------------------------------------------------

	//---------------Omega coord turn calc---------------------
	omega_x_roll_tgt = k_pr_gamma*(gamma_tgt - data_input[GAMMA]);
	omega_x_turn_tgt = omega_turn_tgt*sin(data_input[TETA]*deg2rad);
	omega_target[X] = omega_x_roll_tgt + omega_x_turn_tgt; //deg/s

	omega_y_turn_tgt = omega_turn_tgt*cos(data_input[TETA]*deg2rad)*cos(data_input[GAMMA]*deg2rad);
	omega_target[Y] = omega_y_turn_tgt; //deg/s

	omega_z_turn_tgt = omega_turn_tgt*cos(data_input[TETA]*deg2rad)*sin((-data_input[GAMMA]*deg2rad));
	omega_z_vy_tgt = vy_PI_reg.getOutput();
	omega_target[Z] = omega_z_vy_tgt + omega_z_turn_tgt; //deg/s
	//---------------------------------------------------------
}

void setMode()
{
	static uint8_t prev_mode = 0;

	if(prev_mode != current_mode)
	{
		//beeper->longBeep();
		g_activate_flaps = false;
		integral_reset_flag = true;
	}
	prev_mode = current_mode;

	if(switch_rc.matchMinValue() || switch_rc.matchMaxValue())
		current_mode = PREFLIGHTCHECK;
	else
	{
		if(switch_rc.isInRange(1000, 1100))
		{
			current_mode = DIRECT;
		}
		else
		{
			if(switch_rc.isInRange(1100, 1300))
				current_mode = OMEGA_STAB;
			if(switch_rc.isInRange(1400, 1500))
				current_mode = COMMAND;
			if(switch_rc.isInRange(1700, 1800))
				current_mode = NAV;
			if(switch_rc.isInRange(1900, 1950))
				current_mode = DIRECT_FLAPS;
		}
	}
}
void updateModeState()
{
	switch(current_mode)
	{
		case PREFLIGHTCHECK: preFlightCheckUpdate(); break;
		case DIRECT: directUpdate(); break;
		case OMEGA_STAB: {
				stabOmegaTgtCalc();
				stabOmegaUpdate();
			}break;
		case COMMAND: {
				commandModeUpdate( (-(-0.1173*rc_input[AIL1] + 176.0097)), (0.01953125*rc_input[ELEV] - 29.3164062) );
				stabOmegaUpdate();
			}break;
		case DIRECT_FLAPS: {
				g_activate_flaps = true;
				directUpdate();
			}break;
		case NAV: {
				//some code
			}break;
	}
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_ADC2_Init();
  MX_I2C3_Init();
  MX_TIM6_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  	//-------------------Radio PWM IC INIT--------------------------
	HAL_TIM_RegisterCallback(&htim2, HAL_TIM_IC_CAPTURE_CB_ID, IcHandlerTim2);
	HAL_TIM_RegisterCallback(&htim4, HAL_TIM_IC_CAPTURE_CB_ID, IcHandlerTim4);
	HAL_TIM_RegisterCallback(&htim5, HAL_TIM_IC_CAPTURE_CB_ID, IcHandlerTim5);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);//PA5 thr input
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);//PB3 elev input
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);//PD12 ail input
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);//PB7 rud input
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);//PA0 switch input
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);//PA1 slider input

	//-------------------Servo PWM INIT--------------------------
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);//PA6 thr output
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);//PA7 elev servo output
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);//PB0 ail servo 1 output
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);//PB1 ail servo 2 output
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);//PA3 rud servo 2 output
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);//PA2 ers servo 2 output

	//-------------------Sensors INIT--------------------------
	//Beeper beeper(GPIOD, GPIO_PIN_13);
	HAL_Delay(1000);
	//---------------------------------------------------------

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of sensorsUpdate */
  sensorsUpdateHandle = osThreadNew(sensorsUpdateTask, NULL, &sensorsUpdate_attributes);

  /* creation of modeUpdate */
  modeUpdateHandle = osThreadNew(modeUpdateTask, NULL, &modeUpdate_attributes);

  /* creation of radioInputUpdat */
  radioInputUpdatHandle = osThreadNew(radioInputUpdateTask, NULL, &radioInputUpdat_attributes);

  /* creation of loggerUpdate */
  loggerUpdateHandle = osThreadNew(loggerUpdateTask, NULL, &loggerUpdate_attributes);

  /* creation of actuatorsUpdate */
  actuatorsUpdateHandle = osThreadNew(actuatorsUpdateTask, NULL, &actuatorsUpdate_attributes);

  /* creation of baroUpdate */
  baroUpdateHandle = osThreadNew(baroUpdateTask, NULL, &baroUpdate_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 108;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  htim2.Init.Prescaler = 83;
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
  htim3.Init.Prescaler = 83;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  htim5.Init.Prescaler = 83;
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
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 83;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_sensorsUpdateTask */
/**
* @brief Function implementing the sensorsUpdate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sensorsUpdateTask */
void sensorsUpdateTask(void *argument)
{
  /* USER CODE BEGIN sensorsUpdateTask */
	//MPXV7002 mpxv7002(hadc1);

	BNO055 bno055(&hi2c3);
	osDelay(700);
	bno055.setup();
	osDelay(700);
	bno055.setOperationModeNDOF();
	bno055_vector_t omega;
	bno055_vector_t euler;
	bno055_vector_t accel;

	gps.Start();

	//MS4525DO ms4525do(&hi2c2, 0.01);
  /* Infinite loop */
	for(;;)
	{
		omega = bno055.getVectorGyroscopeRemap();
		euler = bno055.getVectorEulerRemap();
		accel = bno055.getVectorAccelerometerRemap();

		data_input[GYROX] = omega.x;
		data_input[GYROY] = omega.y;
		data_input[GYROZ] = omega.z;
		data_input[TETA] = euler.z;
		data_input[GAMMA] = euler.x;
		data_input[PSI] = euler.y;
		data_input[NZ] = accel.z;
		//data_input[AIR] = ms4525do.getAirSpeed();
		//data_input[BETA] = p3002.getAngle();

		data_input[LATITUDE] = (double) minmea_tocoord(&gps.gpsData.latitude);
		data_input[LONGITUDE] = (double) minmea_tocoord(&gps.gpsData.longitude);
		data_input[GPS_SPEED] = (double) minmea_tofloat(&gps.gpsData.speed) * 0.51; //перевод в м/с из узлов
		data_input[COURSE] = (double) minmea_tofloat(&gps.gpsData.course);
		data_input[GPS_VALID] = (double) gps.gpsData.valid;

		data_input[AIR] = data_input[GPS_SPEED];

		osDelay(10);//ещё 5 мС внутри либы airspeed
	}
  /* USER CODE END sensorsUpdateTask */
}

/* USER CODE BEGIN Header_modeUpdateTask */
/**
* @brief Function implementing the modeUpdate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_modeUpdateTask */
void modeUpdateTask(void *argument)
{
  /* USER CODE BEGIN modeUpdateTask */
  /* Infinite loop */
  for(;;)
  {
	  flapsUpdate(g_activate_flaps);
	  setMode();
	  updateModeState();
	  osDelay(10);
  }
  /* USER CODE END modeUpdateTask */
}

/* USER CODE BEGIN Header_radioInputUpdateTask */
/**
* @brief Function implementing the radioInputUpdat thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_radioInputUpdateTask */
void radioInputUpdateTask(void *argument)
{
  /* USER CODE BEGIN radioInputUpdateTask */
  /* Infinite loop */
  for(;;)
  {
	  updateRcInput();
	  osDelay(50);
  }
  /* USER CODE END radioInputUpdateTask */
}

/* USER CODE BEGIN Header_loggerUpdateTask */
/**
* @brief Function implementing the loggerUpdate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_loggerUpdateTask */
void loggerUpdateTask(void *argument)
{
  /* USER CODE BEGIN loggerUpdateTask */

	/* Infinite loop */
	for(;;)
	{
		memset(str, '\0', sizeof(str));
		#ifndef DEBUG_MODE
			sprintf(str, "%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d",\
					(int)HAL_GetTick(), (int)(10*logger_data[OMEGA_X_ZAD]), (int)(data_input[GYROX]*10),\
					(int)(10*logger_data[OMEGA_Y_ZAD]), (int)(data_input[GYROY]*10), (int)(10*logger_data[OMEGA_Z_ZAD]),\
					(int)(data_input[GYROZ]*10), (int)(data_input[BARO]*100), (int)(100*logger_data[VY_ZAD]),\
					(int)(100*data_input[BAROVY]), (int)(100*data_input[AIR]),	(int)(logger_data[K_PR_OMEGA_X]*10),\
					(int)(100*logger_data[K_INT_OMEGA_X]), (int)(10*logger_data[K_PR_OMEGA_Y]), (int)(100*logger_data[K_INT_OMEGA_Y]),\
					(int)(10*logger_data[K_PR_OMEGA_Z]), (int)(100*logger_data[K_INT_OMEGA_Z]), (int)(10*k_pr_Vy),\
					(int)(data_input[TETA]*10), (int)(data_input[GAMMA]*10), (int)(data_input[PSI]*10),\
					(int)(data_input[NZ]*1000), (int)(10*logger_data[OMEGA_TURN_ZAD]), (int)(10*logger_data[GAMMA_ZAD]), (int)(data_input[LONGITUDE]*1000000), (int)(data_input[LATITUDE]*1000000),\
					(int)(data_input[GPS_SPEED]*100), (int)(data_input[COURSE]*10), (int)(data_input[GPS_VALID]), (int)switch_rc.getPulseWidth());
		#else
			#if DEBUG_MODE == BARO_DEBUG
				sprintf(str, "%d %d %d\n", (int)(data_input[BARO]*100), (int)(logger_data[ALT_FILTERED]*100), (int)(100*data_input[BAROVY]));
			#elif DEBUG_MODE == GYRO_DEBUG
				//sprintf(str, "omega_x=%d, omega_y=%d, omega_z=%d\n", (int)(data_input[GYROX]*10), (int)(data_input[GYROY]*10), (int)(data_input[GYROZ]*10));
				sprintf(str, "%d %d\n", (int)(data_input[GYROZ]*10), (int)(logger_data[OMEGA_Z_ZAD]*10));
			#elif DEBUG_MODE == AIR_DEBUG
				sprintf(str, "%d\n", (int)(1000*data_input[AIR]));
			#elif DEBUG_MODE == RADIO_DEBUG
				sprintf(str, "thr=%d, elev=%d, ail1=%d, ail2=%d, rud=%d, switchA=%d, arm=%d\n", rc_input[THR], rc_input[ELEV], rc_input[AIL1], rc_input[AIL2], rc_input[RUD], rc_input[SWITCHA], rc_input[SLIDER]);
			#elif DEBUG_MODE == BETA_DEBUG
				sprintf(str, "beta=%d\n", (int)data_input[BETA]);
			#endif
		#endif
		HAL_UART_Transmit_IT(&huart2, (uint8_t*)str, sizeof(str));
		osDelay(100);
	}
  /* USER CODE END loggerUpdateTask */
}

/* USER CODE BEGIN Header_actuatorsUpdateTask */
/**
* @brief Function implementing the actuatorsUpdate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_actuatorsUpdateTask */
void actuatorsUpdateTask(void *argument)
{
  /* USER CODE BEGIN actuatorsUpdateTask */
  /* Infinite loop */
	osDelay(WHAIT_FOR_RADIO_MS);
  for(;;)
  {
	  updateActuators();
	  osDelay(50);
  }
  /* USER CODE END actuatorsUpdateTask */
}

/* USER CODE BEGIN Header_baroUpdateTask */
/**
* @brief Function implementing the baroUpdate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_baroUpdateTask */
void baroUpdateTask(void *argument)
{
  /* USER CODE BEGIN baroUpdateTask */
  /* Infinite loop */

	MS5611 ms5611(0x77, &hi2c1, 100, 0.02);
	ms5611.updateQFE(); //remembering QFE pressure when creating object

	for(;;)
	{
		ms5611.calcAltitude();
		data_input[BARO] = ms5611.getRawAltitude();

		ms5611.calcVerticalSpeed();
		data_input[BAROVY] = ms5611.getVerticalSpeed();

		osDelay(2);// в функции вычисления высоты 2 задержки по 9 мС -> цикличность вызоыва каждые 20 мС
	}
  /* USER CODE END baroUpdateTask */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	//sprintf(str, "thr=%d, elev=%d, ail1=%d. ail2=%d, rud=%d, switchA=%d, arm=%d\n", rc_input[THR], rc_input[ELEV], rc_input[AIL1], rc_input[AIL2], rc_input[RUD], rc_input[SWITCHA], rc_input[SLIDER]);
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)"fuck", 4);
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
