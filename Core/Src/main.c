/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @author					: Septian
	*	@file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
#include "stdio.h"

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


int hasil = 0;

//void motor1(float pwm){

//	if(pwm>=1000){pwm=1000;}
//	else if(pwm<=-1000){pwm=-1000;}
//	
//	if(pwm>=0){
//		//HAL_GPIO_WritePin(DirectionPort1,DirectionPin1,GPIO_PIN_RESET);
//		
//		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,pwm);  
//	}
//	
//	else {
//		pwm=1000+pwm;
//		//HAL_GPIO_WritePin(DirectionPort1,DirectionPin1,GPIO_PIN_SET);
//		
//		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,pwm); 
//	}
//}

//void motor2(float pwm){

//	if(pwm>=1000){pwm=1000;}
//	else if(pwm<=-1000){pwm=-1000;}
//	
//	if(pwm>=0){
//		//HAL_GPIO_WritePin(DirectionPort1,DirectionPin1,GPIO_PIN_RESET);
//		
//		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,pwm);  
//	}
//	
//	else {
//		pwm=1000+pwm;
//		//HAL_GPIO_WritePin(DirectionPort1,DirectionPin1,GPIO_PIN_SET);
//		
//		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,pwm); 
//	}
//}
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	
	
  SSD1306_Init();
  char snum[5];
	char tampil [50];
	//float pwm = 0;
	
//uint8_t buttonMaju, buttonMundur, buttonPWMTambah, buttonPWMKurang;

//#define buttonMaju			 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)
//#define buttonMundur		 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)
//#define buttonPWMTambah  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)
//#define buttonPWMKurang  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)
	
  //SSD1306_GotoXY (0,0);
  //SSD1306_Puts ("Maju", &Font_11x18, 1);
  SSD1306_GotoXY (45,15);
  SSD1306_Puts ("BTN", &Font_11x18, 1);
  SSD1306_GotoXY (25, 35);
  SSD1306_Puts ("CHECKER", &Font_11x18, 1);

SSD1306_UpdateScreen();
HAL_Delay (1500);
SSD1306_Clear();
	int value = 0;
	int delayne = 100;
//	int value2 = 500;

HAL_GPIO_WritePin(INHBTN1_GPIO_Port, INHBTN1_Pin, 1);
 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		TIM3->CCR1 = value; //btn1
//		TIM1->CCR1 = value2; //btn2
		
//		HAL_GPIO_WritePin(DIRBTN1_GPIO_Port,DIRBTN1_Pin,0);
//		HAL_GPIO_WritePin(DIRBTN2_GPIO_Port,DIRBTN2_Pin,0);
//		HAL_Delay(1000);
//		
//		// CCw
//		HAL_GPIO_WritePin(DIRBTN1_GPIO_Port,DIRBTN1_Pin,1);
//		HAL_GPIO_WritePin(DIRBTN2_GPIO_Port,DIRBTN2_Pin,1);
//		
//		HAL_Delay(1000);
		
		//Tambah PWM
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)==0){
			HAL_Delay(delayne);
			hasil += 10;
			if(hasil>=100){hasil=100;}
			SSD1306_GotoXY (20,30);
			sprintf(tampil, "PWM: %4d", hasil);
			SSD1306_Puts (tampil, &Font_11x18, 1);
			SSD1306_UpdateScreen();
		}
		
		//Kurang PWM
		else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)==0){
			HAL_Delay(delayne);
			hasil -= 10;
			if(hasil<=0){hasil=0;}
			SSD1306_GotoXY (20,30);
			sprintf(tampil, "PWM: %4d", hasil);
			SSD1306_Puts (tampil, &Font_11x18, 1);
			SSD1306_UpdateScreen();
		}
//		
		//Maju
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)==0){
			if(hasil>=0){
				HAL_GPIO_WritePin(DIRBTN1_GPIO_Port, DIRBTN1_Pin, 0);
				HAL_GPIO_WritePin(DIRBTN2_GPIO_Port, DIRBTN2_Pin, 0);
			}
			else{
				hasil = 100+hasil;
				HAL_GPIO_WritePin(DIRBTN1_GPIO_Port,DIRBTN1_Pin,1);
				HAL_GPIO_WritePin(DIRBTN2_GPIO_Port,DIRBTN2_Pin,1);
			}
		//relay==1;
		HAL_GPIO_WritePin(DIRRELAY_GPIO_Port, DIRRELAY_Pin, 1);
			
				SSD1306_GotoXY (20,10);
				SSD1306_Puts ("Maju  ", &Font_11x18, 1);
				SSD1306_UpdateScreen();
		}
		
		//Mundur
		else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0){
//			HAL_GPIO_WritePin(DIRBTN1_GPIO_Port,DIRBTN1_Pin,0);
//			HAL_GPIO_WritePin(DIRBTN2_GPIO_Port,DIRBTN2_Pin,0);
			if(hasil>=0){
				HAL_GPIO_WritePin(DIRBTN1_GPIO_Port, DIRBTN1_Pin, 1);
				HAL_GPIO_WritePin(DIRBTN2_GPIO_Port, DIRBTN2_Pin, 1);
			}
			else{
				hasil = 100+hasil;
				HAL_GPIO_WritePin(DIRBTN1_GPIO_Port,DIRBTN1_Pin,0);
				HAL_GPIO_WritePin(DIRBTN2_GPIO_Port,DIRBTN2_Pin,0);
			}
			//relay==0;
			HAL_GPIO_WritePin(DIRRELAY_GPIO_Port, DIRRELAY_Pin, 0);
			SSD1306_GotoXY (20,10);
			SSD1306_Puts ("Mundur", &Font_11x18, 1);
			SSD1306_UpdateScreen();
		}
		TIM3->CCR1 = hasil*2000/100; //btn1
		TIM1->CCR1 = hasil*2000/100; //btn2
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
