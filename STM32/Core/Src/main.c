/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h" 
#include <stdlib.h>

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*   tim7      */
extern int16_t tim_cnt;

/*   uart2     */
extern uint8_t CamData;
uint16_t x_u16 = 0;
uint16_t y_u16 = 0;
extern PID X,Y;
uint16_t x_dir = 0x00;
uint16_t y_dir = 0x00;
/*   uart3     */
extern float Roll,Pitch,Yaw;
extern int uart_3_cnt ;
extern uint8_t RxState;
extern uint8_t Jy901s_RxData;
uint8_t RX_Data[100] = {0};

/* Angel */
extern PID Angel;

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
  MX_I2C1_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  MX_TIM9_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
	//电机PWM唤醒
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	
	//PWM激光唤�??
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	
	//定时器开�??
  HAL_TIM_Base_Start_IT(&htim7);
	
  OLED_Init();

  uint8_t wake_cmd[1] = {0x55};
  uint8_t send_data_1[8] = {0x01,0xF6,0x01,0x00,0xff,0x00,0x00,0x6B};
  //��ַλ �ٶ�ģʽ���� 0x1->���� 0x4ff->�ٶ� ���ٶ� У��λ�ֽ�
  uint8_t send_data_2[8] = {0x02,0xF6,0x01,0x00,0xFF,0x00,0x00,0x6B};
  //��ַλ �ٶ�ģʽ���� 0x1->���� 0x4ff->�ٶ� ���ٶ� У��λ�ֽ�

  HAL_UART_Transmit(&huart3, wake_cmd, 1, 100);
	HAL_UART_Transmit(&huart2, wake_cmd, 1, 100);
  HAL_UART_Transmit(&huart5, wake_cmd, 1, 100);
  HAL_UART_Receive_IT(&huart3,&Jy901s_RxData,1);
	HAL_UART_Receive_IT(&huart2,&CamData,1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  int Key_1 = 0;
  int Key_2 = 0;

  Angel.target = 0.0;
  PID_Init(&Angel,300,0,0,0,0,9999,-9999);
	
	X.target = 340;
	Y.target = 320;
	PID_Init(&X,1,0,0,1000,-1000,5000,-5000);				//1
	PID_Init(&Y,0,0,0,1000,-1000,5000,-5000);				//1.5- -01
	
	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(HAL_GPIO_ReadPin(Key_1_GPIO_Port,Key_1_Pin) == 0){
      while(HAL_GPIO_ReadPin(Key_1_GPIO_Port,Key_1_Pin) == 0);
      Key_1++;
    }
    if(HAL_GPIO_ReadPin(Key_2_GPIO_Port,Key_2_Pin) == 0){
      while(HAL_GPIO_ReadPin(Key_2_GPIO_Port,Key_2_Pin) == 0);
      Key_2++;
    }
		
//		if (X.OUT > 0) x_dir = 0x01; else x_dir = 0x00;
//		if (Y.OUT > 0) y_dir = 0x01; else y_dir = 0x00;
//		
//		Emm_V5_Pos_Control(0x01, x_dir, 30, 0, abs(X.OUT), 0, 0);
//		HAL_Delay(1);
//		Emm_V5_Pos_Control(0x02, y_dir, 30, 0, abs(Y.OUT), 0, 0);
//		HAL_Delay(1);

//		Emm_V5_Vel_Control(0x01,0x01,32,0,0); //X		�ٶ�  + ����  - ����左-1，右-0
//		HAL_Delay(1);
//		Emm_V5_Vel_Control(0x02,0x01,32,0,0); //Y   �ٶ�  + ����  - ����上-1，下-0
    //Set_4_Speed(-6000,-6000,-6000,-6000);
//		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 60);
		
		
//		Set_4_Speed(-6000,-6000,-6000,-6000);
		
				if(tim_cnt <= 1000)		//左转
				{
					Emm_V5_Vel_Control(0x01,0x01,300,0,0);
					Set_4_Speed(-6000-Angel.OUT,-6000+Angel.OUT,-6000+Angel.OUT,-6000-Angel.OUT);
				}
				else if(tim_cnt > 1000 && tim_cnt <= 2700) //右转
				{
					Emm_V5_Vel_Control(0x01,0x00,300,0,0);
				}
				else if(tim_cnt > 2700 && tim_cnt <= 3900)	//停止 向下
				{
					Set_4_Speed(0,0,0,0);
					Emm_V5_Vel_Control(0x01,0x00,0,0,0);
					HAL_Delay(1);
					Set_4_Speed(0, 0, 0, 0);
					Emm_V5_Vel_Control(0x02,0x00,40,0,0);
				}
				else if(tim_cnt > 3900 && tim_cnt <= 4100)	//激光 停转
				{
						__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 100);
					Emm_V5_Vel_Control(0x02,0x01,0,0,0);
				}
				else if(tim_cnt > 4100 && tim_cnt <= 5300){	//关激光 左转 上转
					__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
					Emm_V5_Vel_Control(0x01,0x01,300,0,0);					
					HAL_Delay(1);
					Emm_V5_Vel_Control(0x02,0x01,40,0,0);
					Set_4_Speed(-6000-Angel.OUT,-6000+Angel.OUT,-6000+Angel.OUT,-6000-Angel.OUT);
				}
				else if(tim_cnt > 5300 && tim_cnt <= 6000){	//停止上转
					Emm_V5_Vel_Control(0x02,0x01,0,0,0);
				}
				else if(tim_cnt > 6000 && tim_cnt <= 7200){	//停止左转 向下转 停车
					Emm_V5_Vel_Control(0x01,0x01,0,0,0);
					HAL_Delay(1);
					Emm_V5_Vel_Control(0x02,0x00,50,0,0);
					Set_4_Speed(0, 0, 0, 0);
				}
				else if(tim_cnt > 7200 && tim_cnt <= 7400){
					__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 100);
					Emm_V5_Vel_Control(0x02,0x00,0,0,0);
				}
				else if(tim_cnt > 7400 && tim_cnt <= 8600){
					__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
					Emm_V5_Vel_Control(0x02,0x01,40,0,0);
					HAL_Delay(1);
					Emm_V5_Vel_Control(0x01,0x00,300,0,0);	
				}	
				else {
					Emm_V5_Vel_Control(0x02,0x01,0,0,0);
					HAL_Delay(1);
					Emm_V5_Vel_Control(0x01,0x00,100,0,0);
					Set_4_Speed(-6000-Angel.OUT,-6000+Angel.OUT,-6000+Angel.OUT,-6000-Angel.OUT);					
				}

		
    OLED_ShowFloatNum(0,0,Yaw,3,2,OLED_6X8);
    OLED_ShowFloatNum(0,8*1,Roll,3,2,OLED_6X8);
    OLED_ShowFloatNum(0,8*2,Pitch,3,2,OLED_6X8);
    OLED_ShowNum(0,8*3,tim_cnt,5,OLED_6X8);
    OLED_ShowSignedNum(0,8*4,Angel.OUT,6,OLED_6X8);
    OLED_Update();
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
#ifdef USE_FULL_ASSERT
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
