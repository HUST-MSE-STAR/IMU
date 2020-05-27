/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "jiesuan.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

static float ValueX;
static float ValueY;
static float ValueZ;
static float ValueXGyro;
static float ValueYGyro;
static float ValueZGyro;
static float ValueXAccel;
static float ValueYAccel;
static float ValueZAccel;
static float temp;

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
uint16_t IST8310_Add= 0x0E;
uint8_t ist8310=0x0E;
uint8_t valuex_l=0x03;
uint8_t valuex_h=0x04;
uint8_t valuey_l=0x05;
uint8_t valuey_h=0x06;
uint8_t valuez_l=0x07;
uint8_t valuez_h=0x08;


uint8_t ACC_CHIP_ID=0X00;
uint8_t GYRO_CHIP_ID=0X00;
uint8_t ACC_X_LSB=0X12;
uint8_t ACC_X_MSB=0X13;
uint8_t ACC_Y_LSB=0X14;
uint8_t ACC_Y_MSB=0X15;
uint8_t ACC_Z_LSB=0X16;
uint8_t ACC_Z_MSB=0X17;
uint8_t RATE_X_LSB=0X02;
uint8_t RATE_X_MSB=0X03;
uint8_t RATE_Y_LSB=0X04;
uint8_t RATE_Y_MSB=0X05;
uint8_t RATE_Z_LSB=0X06;
uint8_t RATE_Z_MSB=0X07;
uint8_t TEMP_LSB=0X23;
uint8_t TEMP_MSB=0X22;
uint8_t ACC_SOFTRESET=0X7E;
uint8_t GYRO_SOFTRESET=0X14;
uint8_t Accelerometer_on=0x04;
uint8_t ACC_PWR_CTRL=0x7D;
uint8_t ACC_PWR_CONF=0x7C;
uint8_t Active_mode=0x00;
uint8_t GYRO_LPM1=0x11;
uint8_t normal=0x00;
uint8_t ACC_RANGE=0x41;
uint8_t GYRO_RANGE=0x0F;
uint8_t STAT1=0x02;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId Get_InfHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Get_inf(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Get_Inf */
  osThreadDef(Get_Inf, Get_inf, osPriorityHigh, 0, 128);
  Get_InfHandle = osThreadCreate(osThread(Get_Inf), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Get_inf */
/**
* @brief Function implementing the Get_Inf thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Get_inf */
void Get_inf(void const * argument)
{
  /* USER CODE BEGIN Get_inf */
  /* Infinite loop */
  for(;;)
  {
    osDelay(5);
		IST8310_get();
		BMI088_get();
		accel_jiesuan_fai(ax,ay,az,mx,my,mz);
		update(ax,ay,az,gx,gy,gz,mx,my,mz);
		
  }
  /* USER CODE END Get_inf */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     void IST8310_get(void)
		 {
			 uint8_t Buffer[6];
			 uint8_t stat1;
			 
			 HAL_I2C_Master_Transmit_DMA( &hi2c1, IST8310_Add, &ist8310, 8);
			 HAL_I2C_Master_Transmit_DMA( &hi2c1, IST8310_Add, &STAT1, 8);
			 HAL_I2C_Master_Transmit_DMA( &hi2c1, IST8310_Add, &ist8310, 8);
			 HAL_I2C_Master_Receive_DMA( &hi2c1, IST8310_Add, &stat1, 8);
			 
			 if(stat1%2==1)
			 {
			 HAL_I2C_Master_Transmit_DMA( &hi2c1, IST8310_Add, &ist8310, 8);
			 HAL_I2C_Master_Transmit_DMA( &hi2c1, IST8310_Add, &valuex_l, 8);
			 HAL_I2C_Master_Transmit_DMA( &hi2c1, IST8310_Add, &ist8310, 8);
			 HAL_I2C_Master_Receive_DMA( &hi2c1, IST8310_Add, &Buffer[0], 8);
			 
			 HAL_I2C_Master_Transmit_DMA( &hi2c1, IST8310_Add, &ist8310, 8);
			 HAL_I2C_Master_Transmit_DMA( &hi2c1, IST8310_Add, &valuex_h, 8);
			 HAL_I2C_Master_Transmit_DMA( &hi2c1, IST8310_Add, &ist8310, 8);
			 HAL_I2C_Master_Receive_DMA( &hi2c1, IST8310_Add, &Buffer[1], 8);
			 
			 HAL_I2C_Master_Transmit_DMA( &hi2c1, IST8310_Add, &ist8310, 8);
			 HAL_I2C_Master_Transmit_DMA( &hi2c1, IST8310_Add, &valuey_l, 8);
			 HAL_I2C_Master_Transmit_DMA( &hi2c1, IST8310_Add, &ist8310, 8);
			 HAL_I2C_Master_Receive_DMA( &hi2c1, IST8310_Add, &Buffer[2], 8);
			 
			 HAL_I2C_Master_Transmit_DMA( &hi2c1, IST8310_Add, &ist8310, 8);
			 HAL_I2C_Master_Transmit_DMA( &hi2c1, IST8310_Add, &valuey_h, 8);
			 HAL_I2C_Master_Transmit_DMA( &hi2c1, IST8310_Add, &ist8310, 8);
			 HAL_I2C_Master_Receive_DMA( &hi2c1, IST8310_Add, &Buffer[3], 8);
			 
			 HAL_I2C_Master_Transmit_DMA( &hi2c1, IST8310_Add, &ist8310, 8);
			 HAL_I2C_Master_Transmit_DMA( &hi2c1, IST8310_Add, &valuez_l, 8);
			 HAL_I2C_Master_Transmit_DMA( &hi2c1, IST8310_Add, &ist8310, 8);
			 HAL_I2C_Master_Receive_DMA( &hi2c1, IST8310_Add, &Buffer[4], 8);
			 
			 HAL_I2C_Master_Transmit_DMA( &hi2c1, IST8310_Add, &ist8310, 8);
			 HAL_I2C_Master_Transmit_DMA( &hi2c1, IST8310_Add, &valuez_h, 8);
			 HAL_I2C_Master_Transmit_DMA( &hi2c1, IST8310_Add, &ist8310, 8);
			 HAL_I2C_Master_Receive_DMA( &hi2c1, IST8310_Add, &Buffer[5], 8);
			 
			 ValueX=(((uint16_t)(Buffer[0]))|((uint16_t)(Buffer[1])<<8))/32768*1600;
			 ValueY=(((uint16_t)(Buffer[2]))|((uint16_t)(Buffer[3])<<8))/32768*1600;
			 ValueZ=(((uint16_t)(Buffer[4]))|((uint16_t)(Buffer[5])<<8))/32768*2500;
		   }
		 }
		 
		 void BMI088_get(void)
		 {
			 uint8_t addr;
			 uint8_t Rxdate;
			 uint8_t Dat[14];
			 uint8_t Acc_range;
			 uint8_t Gyro_range;
			 uint16_t Temp_uint11;
			 HAL_GPIO_WritePin(CSB_GYRO_GPIO_Port, CSB_GYRO_Pin, GPIO_PIN_RESET);
			 osDelay(1);
			 HAL_SPI_TransmitReceive_DMA(&hspi1, &GYRO_CHIP_ID, &Rxdate, 0xFFFF);
			 if(Rxdate==0x0F)
			 {
				 HAL_SPI_Transmit_DMA(&hspi1, &GYRO_LPM1, 0xFFFF);
				 HAL_SPI_Transmit_DMA(&hspi1, &normal, 0xFFFF);
				 HAL_SPI_TransmitReceive_DMA(&hspi1, &GYRO_RANGE, &Gyro_range, 8);
				 HAL_SPI_TransmitReceive_DMA(&hspi1, &RATE_X_LSB, &Dat[0], 8);
				 HAL_SPI_TransmitReceive_DMA(&hspi1, &RATE_X_MSB, &Dat[1], 8);
				 HAL_SPI_TransmitReceive_DMA(&hspi1, &RATE_Y_LSB, &Dat[2], 8);
				 HAL_SPI_TransmitReceive_DMA(&hspi1, &RATE_Y_MSB, &Dat[3], 8);
				 HAL_SPI_TransmitReceive_DMA(&hspi1, &RATE_Z_LSB, &Dat[4], 8);
				 HAL_SPI_TransmitReceive_DMA(&hspi1, &RATE_Z_MSB, &Dat[5], 8);
				 osDelay(1);
				 
				 ValueXGyro=(((uint16_t)(Dat[0]))|((uint16_t)(Dat[1])<<8))/32767*2000/2^(Gyro_range);
				 ValueYGyro=(((uint16_t)(Dat[2]))|((uint16_t)(Dat[3])<<8))/32767*2000/2^(Gyro_range);
				 ValueZGyro=(((uint16_t)(Dat[4]))|((uint16_t)(Dat[5])<<8))/32767*2000/2^(Gyro_range);
			 }
			 HAL_GPIO_WritePin(CSB_GYRO_GPIO_Port, CSB_GYRO_Pin, GPIO_PIN_SET);
			 
			 HAL_GPIO_WritePin(CSB_ACCEL_GPIO_Port, CSB_ACCEL_Pin, GPIO_PIN_RESET);
			 osDelay(1);
			 HAL_SPI_TransmitReceive_DMA(&hspi1, &ACC_CHIP_ID, &Rxdate, 0xFFFF);
			 if(Rxdate==0X1E)
			 {
         HAL_SPI_Transmit_DMA(&hspi1, &ACC_PWR_CTRL, 8);
				 HAL_SPI_Transmit_DMA(&hspi1, &Accelerometer_on, 8);
				 HAL_SPI_Transmit_DMA(&hspi1, &ACC_PWR_CONF, 8);
				 HAL_SPI_Transmit_DMA(&hspi1, &Active_mode, 8);
				 HAL_SPI_TransmitReceive_DMA(&hspi1, &ACC_RANGE, &Acc_range, 8);
				 HAL_SPI_TransmitReceive_DMA(&hspi1, &ACC_X_LSB, &Dat[6], 8);
				 HAL_SPI_TransmitReceive_DMA(&hspi1, &ACC_X_MSB, &Dat[7], 8);
				 HAL_SPI_TransmitReceive_DMA(&hspi1, &ACC_Y_LSB, &Dat[8], 8);
				 HAL_SPI_TransmitReceive_DMA(&hspi1, &ACC_Y_MSB, &Dat[9], 8);
				 HAL_SPI_TransmitReceive_DMA(&hspi1, &ACC_Z_LSB, &Dat[10], 8);
				 HAL_SPI_TransmitReceive_DMA(&hspi1, &ACC_Z_MSB, &Dat[11], 8);
				 HAL_SPI_TransmitReceive_DMA(&hspi1, &TEMP_LSB, &Dat[12], 8);
				 HAL_SPI_TransmitReceive_DMA(&hspi1, &TEMP_MSB, &Dat[13], 8);
				 osDelay(1);
				 
				 ValueXAccel=(((uint16_t)(Dat[6]))|((uint16_t)(Dat[7])<<8))/32768*1000*(2^(Acc_range+1))*1.5;
				 ValueYAccel=(((uint16_t)(Dat[8]))|((uint16_t)(Dat[9])<<8))/32768*1000*(2^(Acc_range+1))*1.5;
				 ValueZAccel=(((uint16_t)(Dat[10]))|((uint16_t)(Dat[11])<<8))/32768*1000*(2^(Acc_range+1))*1.5;
				 Temp_uint11 = (Dat[13] * 8) + (Dat[12] / 32);
         if (Temp_uint11 > 1023)
         Temp_uint11 = Temp_uint11 - 2048;
         else 				 
					 Temp_uint11 = Temp_uint11;			
         temp = Temp_uint11 * 0.125/Dat[12] + 23;
				 HAL_GPIO_WritePin(CSB_ACCEL_GPIO_Port, CSB_ACCEL_Pin, GPIO_PIN_SET);
			 }			
       

			 
		 }
		 
	
		 
			 
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
