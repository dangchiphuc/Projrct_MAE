/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*
 * struct includes variables of pid control
 * param:	kp --- propotion
 * 			ki --- integral		kb 		--- anti windup
 * 			kd --- derivative	alpha 	--- lowpass filter
 **/
typedef struct pid
{
	float kp 	;
	float ki 	;
	float kd	;
	float kb 	;
	float alpha ;
	bool  dir	;
}pid;

/*
 * struct includes variables of motor's feedback
 * param:	current_Pos --- current position uint: radian
 * 			real_Pos	--- position uint: meters
 * 			current_Vel --- current velocity uint: rad/s
 * 			RPM			--- roll per minute
 **/

typedef struct motor
{
	float current_Pos	;
	float real_Pos		;
	float current_Vel	;
	float RPM			;
}motor;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define pi 			3.14
#define rad_round	2*pi
#define	rad_pulse 	pi/748
#define r_Wheel		0.0325
#define sample_time	0.005

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint8_t rx_buffer[20], rx_data, rx_index ;
uint8_t prevState_1 =0, prevState_2 = 0  ;
uint16_t pwm1 ,pwm2 , tim4_tick, DesiredPos, DesiredSpeed ;
uint16_t cnt_vel_1=0 , cnt_vel_2=0 ;
int16_t cnt_pos_1 = 0, cnt_pos_2 = 0, round_1 =0, round_2 = 0;
//float current_Pos1=0, current_Pos2=0, vel_Rad1 =0 , vel_Rad2 =0;
bool run = false, flag = false;
motor motor_1;
motor motor_2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
	#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif

PUTCHAR_PROTOTYPE
	{
		HAL_UART_Transmit(&huart1, (uint8_t*)&ch,1,100);
		return ch;
	}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

///-------------PROTOTYPE--------------------///
uint16_t pid_velo(float setpoint, float current, pid pid);
uint16_t pid_pos(float setpoint, float current,  pid pid);
void setRoute(uint8_t state);
void clockWise(motor motor);
void cnterClockwise(motor motor);

/*-----calback function when recieve data via uart-----------
 * Using recieve_IT()
 * */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	uint8_t i;
				if(huart->Instance == USART1) //uart1
				{
						if(rx_index==0) {for (i=0;i<20;i++) rx_buffer[i] = 0;}

				switch(rx_data) {
		            /* dung dong co */
		            case 'e':
		                run =false;
		                break;

		            /* dong co chay */
		            case 'r':
		                run = true;
		                break;
		            case 'k':
						flag = true;
						break;
		            case 'l':
						flag = false;
						break;
		            case 'b':
		//								reset();
										break;
		            case 's':
		                DesiredPos = atoi((const char *)rx_buffer);
		                memset(&rx_buffer, 0, sizeof(rx_buffer));
		                rx_index = 0;
		                break;
		            case 'v':
		                DesiredSpeed = atoi((const char *)rx_buffer);
		                //DesiredVel = DesiredSpeed * (pi/30);
		                memset(&rx_buffer, 0, sizeof(rx_buffer));
		                rx_index = 0;
		                break;

//		            case 'p':
//		            	kp = atof((const char *)rx_buffer);
//		            	memset(&rx_buffer, 0, sizeof(rx_buffer));
//		            	rx_index = 0 ;
//		            	break;
//
//		            case 'i':
//						ki = atof((const char *)rx_buffer);
//						memset(&rx_buffer, 0, sizeof(rx_buffer));
//						rx_index = 0 ;
//						break;
//		            case 'd':
//						kd = atof((const char *)rx_buffer);
//						memset(&rx_buffer, 0, sizeof(rx_buffer));
//						rx_index = 0 ;
//						break;

		            case '0':
		            case '1':
		            case '2':
		            case '3':
		            case '4':
		            case '5':
		            case '6':
		            case '7':
		            case '8':
		            case '9':
		            case '.':
		            case '-':
		                rx_buffer[rx_index++] |= rx_data;
		                break;
		            default:
		                break;
		        }
						HAL_UART_Receive_IT(&huart1,&rx_data,1);
				}
	}


	/*--------------xu li ngat ngoai---------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
  /*
   * Using external interrupt pins to calculate feedback from encoder of motor
   * EXTI in rising and falling mode --- 4 counts for 1 pulse
   * Motor 1: channel A -> EXTI 4 ; channel B -> EXTI 5
   * Motor 2: channel A -> EXTI 6 ; channel B -> EXTI 7*/
  //// encoder motor 1 channel A

  if(GPIO_Pin == M1_A_Pin){
	  uint8_t state =0;
	  state  = HAL_GPIO_ReadPin(GPIOA, M1_A_Pin);
	  state  = (state << 1) | HAL_GPIO_ReadPin(GPIOA, M1_B_Pin);
	  state &= 0x03;
	  switch (state){
	  case 0:
		  if(prevState_1 == 1)
			  cnt_pos_1 ++;
		  else
			  cnt_pos_1 --;
		  break;
	  case 1:
	  		  if(prevState_1 == 3)
	  			  cnt_pos_1 ++;
	  		  else
	  			  cnt_pos_1 --;
	  		break;
	  case 2:
	  		  if(prevState_1 == 0)
	  			  cnt_pos_1 ++;
	  		  else
	  			  cnt_pos_1 --;
	  		break;
	  case 3:
	  		  if(prevState_1 == 2)
	  			  cnt_pos_1 ++;
	  		  else
	  			  cnt_pos_1 --;
	  		break;
	  }
	  cnt_vel_1 ++;
	  prevState_1 = state;
	  if(cnt_pos_1 >1495){
		  round_1 ++;
		  cnt_pos_1 =0;
	  }
	  if(cnt_pos_1 < -1495){
		  round_1 --;
		  cnt_pos_1 =0;
	  }
  }

  //// encoder motor 1 channel B

  if(GPIO_Pin == M1_B_Pin){
  	  uint8_t state =0;
  	  state  = HAL_GPIO_ReadPin(GPIOA, M1_A_Pin);
  	  state  = (state << 1) | HAL_GPIO_ReadPin(GPIOA, M1_B_Pin);
  	  state &= 0x03;
  	  switch (state){
  	  case 0:
  		  if(prevState_1 == 1)
  			  cnt_pos_1 ++;
  		  else
  			  cnt_pos_1 --;
  		  break;
  	  case 1:
  	  		  if(prevState_1 == 3)
  	  			  cnt_pos_1 ++;
  	  		  else
  	  			  cnt_pos_1 --;
  	  		break;
  	  case 2:
  	  		  if(prevState_1 == 0)
  	  			  cnt_pos_1 ++;
  	  		  else
  	  			  cnt_pos_1 --;
  	  		break;
  	  case 3:
  	  		  if(prevState_1 == 2)
  	  			  cnt_pos_1 ++;
  	  		  else
  	  			  cnt_pos_1 --;
  	  		break;
  	  }
  	  cnt_vel_1 ++;
  	  prevState_1 = state;
  	  if(cnt_pos_1 >1495){
  		  round_1 ++;
  		  cnt_pos_1 =0;
  	  }
  	  if(cnt_pos_1 < -1495){
  		  round_1 --;
  		  cnt_pos_1 =0;
  	  }
    }

  //// encoder motor 2 channel A

  if(GPIO_Pin == M2_A_Pin){
  	  uint8_t state =0;
  	  state  = HAL_GPIO_ReadPin(GPIOA, M2_A_Pin);
  	  state  = (state << 1) | HAL_GPIO_ReadPin(GPIOA, M2_B_Pin);
  	  state &= 0x03;
  	  switch (state){
  	  case 0:
  		  if(prevState_2 == 1)
  			  cnt_pos_2 ++;
  		  else
  			  cnt_pos_2 --;
  		  break;
  	  case 1:
  	  		  if(prevState_2 == 3)
  	  			  cnt_pos_2 ++;
  	  		  else
  	  			  cnt_pos_2 --;
  	  		break;
  	  case 2:
  	  		  if(prevState_2 == 0)
  	  			  cnt_pos_2 ++;
  	  		  else
  	  			  cnt_pos_2 --;
  	  		break;
  	  case 3:
  	  		  if(prevState_2 == 2)
  	  			  cnt_pos_2 ++;
  	  		  else
  	  			  cnt_pos_2 --;
  	  		break;
  	  }
  	  cnt_vel_2 ++;
  	  prevState_2 = state;
  	  if(cnt_pos_2 >1495){
  		  round_2 ++;
  		  cnt_pos_2 =0;
  	  }
  	  if(cnt_pos_2 < -1495){
  		  round_2 --;
  		  cnt_pos_2 =0;
  	  }
    }

    //// encoder motor 2 channel B

    if(GPIO_Pin == M2_B_Pin){
    	  uint8_t state =0;
    	  state  = HAL_GPIO_ReadPin(GPIOA, M2_A_Pin);
    	  state  = (state << 1) | HAL_GPIO_ReadPin(GPIOA, M2_B_Pin);
    	  state &= 0x03;
    	  switch (state){
    	  case 0:
    		  if(prevState_2 == 1)
    			  cnt_pos_2 ++;
    		  else
    			  cnt_pos_2 --;
    		  break;
    	  case 1:
    	  		  if(prevState_2 == 3)
    	  			  cnt_pos_2 ++;
    	  		  else
    	  			  cnt_pos_2 --;
    	  		break;
    	  case 2:
    	  		  if(prevState_2 == 0)
    	  			  cnt_pos_2 ++;
    	  		  else
    	  			  cnt_pos_2 --;
    	  		break;
    	  case 3:
    	  		  if(prevState_2 == 2)
    	  			  cnt_pos_2 ++;
    	  		  else
    	  			  cnt_pos_2 --;
    	  		break;
    	  }
    	  cnt_vel_2 ++;
    	  prevState_2 = state;
    	  if(cnt_pos_2 >1495){
    		  round_2 ++;
    		  cnt_pos_2 =0;
    	  }
    	  if(cnt_pos_2 < -1495){
    		  round_2 --;
    		  cnt_pos_2 =0;
    	  }
      }
}

/**
 * Using timer callback function() to calculate position and velocity of 2 motors
 * TIM3: period 5ms --- calculate position and velocity
 * TIM4: period 1ms --- transmit data via protocol to host
 * */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim->Instance == TIM3){

		motor_1.current_Pos = round_1*rad_round + cnt_pos_1*rad_pulse; ///position motor 1: (rad)
		motor_1.real_Pos	= motor_1.current_Pos * r_Wheel; 		   /// L = (rad)*R
		motor_1.RPM 		= cnt_vel_1 * 8.02139;		 /// v = count*(1/1496)/(0.005/60) (rpm)
		cnt_vel_1			= 0;
		motor_1.current_Vel	= motor_1.RPM * (pi/30); 	 /// v (rad/s)

		motor_2.current_Pos = round_2*rad_round + cnt_pos_2*rad_pulse; ///position motor 1: (rad)
		motor_2.real_Pos	= motor_2.current_Pos * r_Wheel; 		   /// L = (rad)*R
		motor_2.RPM 		= cnt_vel_2 * 8.02139;		 /// v = count*(1/1496)/(0.005/60) (rpm)
		cnt_vel_2			= 0;
		motor_2.current_Vel	= motor_2.RPM * (pi/30); 	 /// v (rad/s)

		//if(run == true){
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, 600);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3, 600);

		//}

	}
	if(htim->Instance == TIM4){

//		tim4_tick ++;
//		if(tim4_tick == 6){
//			printf("V%f\r\n", motor_1.RPM);
//			tim4_tick = 0;
//		}


	}

}

/*
 * brief: pid control calculate
 * param: setpoint	--- offset value
 * 		  current	--- current value
 * 		  kp,ki,kd	--- P,I,D ratio
*  return: value for pwm
**/
uint16_t pid_velo(float setpoint, float current, pid pid){

	static float ui_prev =0, err_prev = 0, err_sat = 0, ud_ftr_prev = 0;
	float err,up,ui,ud,pid_term,pid_sat,err_windup, ud_filter;
	uint16_t pwm;
	int16_t HILIM = 1000, LOLIM = 0;

	err 		= setpoint - current						;		/*Error*/
	up			= pid.kp*err								;		/*Propotion*/
	err_windup	= pid.ki*err + pid.kb*err_sat				;		/*Antiwindup*/
	ui			= ui_prev + err_windup*sample_time			;		/*Intergral*/
	ud			= pid.kd*(err-err_prev)/sample_time			;		/*Derivatvie*/
	ud_filter	= ud*pid.alpha + (1 - pid.alpha)*ud_ftr_prev;		/*Lowpass filter*/
	pid_term	= up + ui + ud_filter						;

	////--------SATURATION---------///
	if(pid_term > HILIM)
		pid_sat = HILIM			;
	if(pid_term < LOLIM)
		pid_sat = LOLIM			;
	else
		pid_sat = pid_term		;
	err_sat = pid_sat - pid_term;
	/*-------------------------------*/

	/*Store previous value for next pid calculate*/
	ui_prev = ui			;
	err_prev = err			;
	ud_ftr_prev = ud_filter	;
	/*-------------------------------------------*/

	pwm = (unsigned int)pid_sat	;
	return pwm;
}

uint16_t pid_pos(float setpoint, float current,  pid pid){

	static float ui_prev =0, err_prev = 0, err_sat = 0, ud_ftr_prev = 0;
	float err,up,ui,ud,pid_term,pid_sat,err_windup, ud_filter;
	uint16_t pwm;
	int16_t HILIM = 1000, LOLIM = -1000;

	err 		= setpoint - current						;		/*Error*/
	up			= pid.kp*err								;		/*Propotion*/
	err_windup	= pid.ki*err + pid.kb*err_sat				;		/*Antiwindup*/
	ui			= ui_prev + err_windup*sample_time			;		/*Intergral*/
	ud			= pid.kd*(err-err_prev)/sample_time			;		/*Derivatvie*/
	ud_filter	= ud*pid.alpha + (1 - pid.alpha)*ud_ftr_prev;		/*Lowpass filter*/
	pid_term	= up + ui + ud_filter						;

	////--------SATURATION---------///
	if(pid_term > HILIM)
		pid_sat = HILIM			;
	if(pid_term < LOLIM)
		pid_sat = LOLIM			;
	else
		pid_sat = pid_term		;

	////--------Direction---------///
	if(pid_sat > 0)
		pid.dir = 1;
	if(pid_sat < 0)
		pid.dir = 0;

	/*Store previous value for next pid calculate*/
	ui_prev = ui			;
	err_prev = err			;
	ud_ftr_prev = ud_filter	;
	/*-------------------------------------------*/

	pwm = abs(pid_sat)		;
	return pwm;

}


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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /// ham khoi tao 2 kenh pwm cho 2 dong co
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);

  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,0);
  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);

  /// ngat uart_rx
  HAL_UART_Receive_IT(&huart1, &rx_data,1);

  /// init value of 2 motors
  motor_1.current_Pos = 0;
  motor_1.current_Vel = 0;

  motor_2.current_Pos = 0;
  motor_2.current_Vel = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_GPIO_TogglePin(GPIOC,LED_Pin );
	  HAL_Delay(100);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 12-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 24-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 24-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M1_A_Pin M1_B_Pin M2_A_Pin M2_B_Pin */
  GPIO_InitStruct.Pin = M1_A_Pin|M1_B_Pin|M2_A_Pin|M2_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
