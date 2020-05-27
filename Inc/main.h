/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
void IST8310_get(void);
void BMI088_get(void);
void Dat_send(uint16_t ValueXAccel,uint16_t ValueYAccel,uint16_t ValueZAccel,uint16_t ValueXGyro,uint16_t ValueYGyro,uint16_t ValueZGyro,uint16_t ValueX,uint16_t ValueY,uint16_t ValueZ,uint16_t temp);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define INT_GYRO_Pin GPIO_PIN_0
#define INT_GYRO_GPIO_Port GPIOB
#define INT_ACCEL_Pin GPIO_PIN_1
#define INT_ACCEL_GPIO_Port GPIOB
#define DRDY_Pin GPIO_PIN_12
#define DRDY_GPIO_Port GPIOB
#define RSTN_Pin GPIO_PIN_13
#define RSTN_GPIO_Port GPIOB
#define TX_Pin GPIO_PIN_9
#define TX_GPIO_Port GPIOA
#define RX_Pin GPIO_PIN_10
#define RX_GPIO_Port GPIOA
#define SCL_Pin GPIO_PIN_6
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOB
#define CSB_GYRO_Pin GPIO_PIN_8
#define CSB_GYRO_GPIO_Port GPIOB
#define CSB_ACCEL_Pin GPIO_PIN_9
#define CSB_ACCEL_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
