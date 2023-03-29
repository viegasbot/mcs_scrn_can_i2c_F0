/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_config.h"
#include "i2c_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
bool var = false;
bool print = false;
bool confirm_button = false;
bool PLUS_button_State;



uint8_t CAN_temp1_value = 0;
uint8_t CAN_temp2_value = 0;
uint8_t CAN_temp3_value = 0;
uint8_t CAN_temp4_value = 0;
uint8_t CAN_temp5_value = 0;


static uint8_t I2Cdata[2] = {0, 0};
static uint16_t I2Csize = 2;
#define slave_addr 0x01

extern uint32_t TxMailbox;
extern CAN_TxHeaderTypeDef TxHeader;
#define MCS_MAIN_ADDR 0x020Au

uint8_t CAN_message_active = 0;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */



uint8_t SW_lock = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(!SW_lock && GPIO_Pin == SW1_Pin)
	{
		SW_lock = 1;
		HAL_TIM_Base_Start_IT(&htim14);
		HAL_GPIO_TogglePin(PCB_LED_GREEN_GPIO_Port, PCB_LED_GREEN_Pin);

		I2Cdata[MSG_ID] = P2D_I2C_ID;
		I2Cdata[MSG_VALUE] = 1;
		HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, I2Cdata, I2Csize);
	}

	else if(!SW_lock && GPIO_Pin == SW2_Pin)
	{
		SW_lock = 1;
		HAL_TIM_Base_Start_IT(&htim14);
		HAL_GPIO_TogglePin(PCB_LED_GREEN_GPIO_Port, PCB_LED_GREEN_Pin);

		I2Cdata[MSG_ID] = TS_I2C_ID;
		I2Cdata[MSG_VALUE] = 1;
		HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, I2Cdata, I2Csize);
	}

	else if(!SW_lock && GPIO_Pin == SW3_Pin)
	{
		SW_lock = 1;
		HAL_TIM_Base_Start_IT(&htim14);
		HAL_GPIO_TogglePin(PCB_LED_GREEN_GPIO_Port, PCB_LED_GREEN_Pin);
	}

	else if(!SW_lock && GPIO_Pin == SW4_Pin)
	{
		SW_lock = 1;
		HAL_TIM_Base_Start_IT(&htim14);
		HAL_GPIO_TogglePin(PCB_LED_GREEN_GPIO_Port, PCB_LED_GREEN_Pin);
	}

	else if(!SW_lock && GPIO_Pin == SW5_Pin)
	{
		SW_lock = 1;
		HAL_TIM_Base_Start_IT(&htim14);
		HAL_GPIO_TogglePin(PCB_LED_GREEN_GPIO_Port, PCB_LED_GREEN_Pin);
	}

	else if(!SW_lock && GPIO_Pin == SW6_Pin)
	{
		SW_lock = 1;
		HAL_TIM_Base_Start_IT(&htim14);
		HAL_GPIO_TogglePin(PCB_LED_GREEN_GPIO_Port, PCB_LED_GREEN_Pin);
	}

	else if(!SW_lock && GPIO_Pin == SW7_Pin)
	{
		SW_lock = 1;
		HAL_TIM_Base_Start_IT(&htim14);
		HAL_GPIO_TogglePin(PCB_LED_GREEN_GPIO_Port, PCB_LED_GREEN_Pin);
	}

	else if(!SW_lock && GPIO_Pin == SW8_Pin)
	{
		SW_lock = 1;
		HAL_TIM_Base_Start_IT(&htim14);
		HAL_GPIO_TogglePin(PCB_LED_GREEN_GPIO_Port, PCB_LED_GREEN_Pin);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(SW_lock == 1 && htim == &htim14
	&& HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin)
	&& HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin)
	&& HAL_GPIO_ReadPin(SW2_GPIO_Port, SW3_Pin)
	&& HAL_GPIO_ReadPin(SW2_GPIO_Port, SW4_Pin)
	&& HAL_GPIO_ReadPin(SW2_GPIO_Port, SW5_Pin)
	&& HAL_GPIO_ReadPin(SW2_GPIO_Port, SW6_Pin)
	&& HAL_GPIO_ReadPin(SW2_GPIO_Port, SW7_Pin)
	&& HAL_GPIO_ReadPin(SW2_GPIO_Port, SW8_Pin))
	{
		SW_lock = 0;
		HAL_TIM_Base_Stop_IT(&htim14);
	}
}
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
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  CAN_Init();

  TxHeader.StdId = MCS_MAIN_ADDR;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //for(uint8_t i = 0; i < 10; i++)
  //{
//	  HAL_GPIO_TogglePin(PCB_LED_RED_GPIO_Port, PCB_LED_RED_Pin);
//	  HAL_Delay(60);
 // }

/*
  HAL_I2C_Master_Transmit(&hi2c1, slave_addr, &I2C_status, data_size, HAL_MAX_DELAY);
  //SLAVE MUSI MIEĆ ZASILANIE W MOMENCIE WYWO�?ANIA TEJ FUNKCJI, INACZEJ PROGRAM PÓJDZIE DALEJ
  HAL_Delay(200);
  HAL_TIM_Base_Start_IT(&htim14);
*/

//  uint8_t CANdata[2] = {0, 0};
//  uint16_t CANsize = 2;
	  HAL_GPIO_WritePin(LED_MF_GREEN_GPIO_Port, LED_MF_GREEN_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LED_MF_RED_GPIO_Port, LED_MF_RED_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LED_MDF_GREEN_GPIO_Port, LED_MDF_GREEN_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LED_MDF_RED_GPIO_Port, LED_MDF_RED_Pin, GPIO_PIN_SET);
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(LED_MF_GREEN_GPIO_Port, LED_MF_GREEN_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LED_MF_RED_GPIO_Port, LED_MF_RED_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LED_MDF_GREEN_GPIO_Port, LED_MDF_GREEN_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LED_MDF_RED_GPIO_Port, LED_MDF_RED_Pin, GPIO_PIN_RESET);

  while (1)
  {

//		  HAL_GPIO_TogglePin(PCB_LED_GREEN_GPIO_Port, PCB_LED_GREEN_Pin);
//		  HAL_Delay(100);

//	  if(var == true)
//	  {
//	  CANdata = 1;
//  	  HAL_I2C_Master_Transmit(&hi2c1, slave_addr, &CANdata, CANsize, HAL_MAX_DELAY);
//  	  var = false;
//	  CANdata = 0;
//	  for(uint8_t i = 0; i < 6; i++)
//		  {
//		  	  HAL_GPIO_TogglePin(PCB_LED_GREEN_GPIO_Port, PCB_LED_GREEN_Pin);
//		  	  HAL_Delay(50);
//		  }
//	  }
//
//	  if(print == true)
//	  {
//	  CANdata = 2;
//  	  HAL_I2C_Master_Transmit(&hi2c1, slave_addr, &CANdata, CANsize, HAL_MAX_DELAY);
//  	  print = false;
//	  CANdata = 0;
//	  for(uint8_t i = 0; i < 6; i++)
//		  {
//		  	  HAL_GPIO_TogglePin(PCB_LED_GREEN_GPIO_Port, PCB_LED_GREEN_Pin);
//		  	  HAL_Delay(50);
//		  }
//	  }







//	  if(map_value_active == true)
//	  {
//		  CANdata[0] = 0x05;
//		  CANdata[1] = can_map_value;
//		  HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, CANdata, CANsize);
//		  map_value_active = false;
//		  HAL_GPIO_TogglePin(PCB_LED_RED_GPIO_Port, PCB_LED_RED_Pin);
//	  }

//	  if(tc_value_active == true)
//	  {
//		  CANdata[0] = 0x04;
//		  CANdata[1] = can_tc_value;
//		  HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, CANdata, CANsize);
//		  tc_value_active = false;
//		  HAL_GPIO_TogglePin(PCB_LED_RED_GPIO_Port, PCB_LED_RED_Pin);
//	  }

//	  if(speed_value_active == true)
// 	  {
//	  	  CANdata[0] = 0x03;
//		  CANdata[1] = can_speed_value;
//	  	  HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, CANdata, CANsize);
//	  	  speed_value_active = false;
//		  HAL_GPIO_TogglePin(PCB_LED_RED_GPIO_Port, PCB_LED_RED_Pin);
//	  }

//	  if(diff_value_active == true)
// 	  {
//	  	  CANdata[0] = 0x06;
//		  CANdata[1] = can_diff_value;
//	  	  HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, CANdata, CANsize);
//	  	  diff_value_active = false;
//		  HAL_GPIO_TogglePin(PCB_LED_RED_GPIO_Port, PCB_LED_RED_Pin);
//	  }



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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
