/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BT4_Pin GPIO_PIN_0
#define BT4_GPIO_Port GPIOA
#define BT3_Pin GPIO_PIN_1
#define BT3_GPIO_Port GPIOA
#define BT2_Pin GPIO_PIN_2
#define BT2_GPIO_Port GPIOA
#define BT1_Pin GPIO_PIN_3
#define BT1_GPIO_Port GPIOA
#define INHBTN1_Pin GPIO_PIN_5
#define INHBTN1_GPIO_Port GPIOA
#define IS1BTN1_Pin GPIO_PIN_7
#define IS1BTN1_GPIO_Port GPIOA
#define DIRBTN1_Pin GPIO_PIN_0
#define DIRBTN1_GPIO_Port GPIOB
#define IS2BTN1_Pin GPIO_PIN_1
#define IS2BTN1_GPIO_Port GPIOB
#define DIRBTN2_Pin GPIO_PIN_9
#define DIRBTN2_GPIO_Port GPIOA
#define DIRRELAY_Pin GPIO_PIN_5
#define DIRRELAY_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
