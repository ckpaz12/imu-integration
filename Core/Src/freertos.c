/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu.h"
#include "cmsis_os.h"
#include <stdio.h>
#include "usart_i2c.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_QUEUE_SIZE 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
union FloatBytes {
    float float_value;
    uint8_t bytes[4];
} FloatBytes;

/* Definitions for blinkled */
osThreadId_t blinkledHandle;
const osThreadAttr_t blinkled_attributes = {
  .name = "blinkled",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for readimu */
osThreadId_t readimuHandle;
const osThreadAttr_t readimu_attributes = {
  .name = "readimu",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for transmitdata */
osThreadId_t transmitdataHandle;
const osThreadAttr_t transmitdata_attributes = {
  .name = "transmitdata",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osMessageQueueId_t imuMessageQueueHandle;
const osMessageQueueAttr_t imuMessageQueue_attributes = {
  .name = "imuMessageQueue"
};


/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void MX_FREERTOS_Init(void);
void StartBlinkLed(void *argument);
void StartReadIMU(void *argument);
void StartTransmitData(void *argument);

void addtoIMUQueue(char* type, char* dimension, uint8_t* data);

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* Initialize IMU */
  initIMU();

  /* Create Queue */
  imuMessageQueueHandle = osMessageQueueNew(IMU_QUEUE_SIZE, sizeof(IMU_msg_t), &imuMessageQueue_attributes);

  /* Create the thread(s) */
  blinkledHandle = osThreadNew(StartBlinkLed, NULL, &blinkled_attributes);
  readimuHandle = osThreadNew(StartReadIMU, NULL, &readimu_attributes);
  transmitdataHandle = osThreadNew(StartTransmitData, NULL, &transmitdata_attributes);
}

/**
  * @brief  Function implementing the blinkled thread.
  * @param  argument: Not used
  * @retval None
  */

void StartBlinkLed(void *argument)
{
  /* Infinite loop */
  while(1)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    osDelay(1000);
  }

  // In case we accidentally exit from task loop
  osThreadTerminate(NULL);
}

/**
* @brief Function implementing the imu thread.
* @param argument: Not used
* @retval None
*/

void StartReadIMU(void *argument)
{
  union FloatBytes gy_x, gy_y, gy_z, ax_x, ax_y, ax_z;
  /* Infinite loop */
  while(1)
  {
    gy_x.float_value = gyro(GYRO_X);
    gy_y.float_value = gyro(GYRO_Y);
    gy_z.float_value = gyro(GYRO_Z);
    ax_x.float_value = accel(ACCEL_X);
    ax_y.float_value = accel(ACCEL_Y);
    ax_z.float_value = accel(ACCEL_Z);


    printf("\e[1;1H\e[38;2;252;186;3mGYRO\e[0m | X : %i | Y : %i | Z : %i \n\r", (int)gy_x.float_value, (int)gy_y.float_value, (int)gy_z.float_value);
    printf("\e[1;1H\e[38;2;252;186;3mACCL\e[0m | X : %i | Y : %i | Z : %i \n\r", (int)ax_x.float_value, (int)ax_y.float_value, (int)ax_z.float_value);

    // IMU DATA: 16 ASCII characters
    addtoIMUQueue("G", "X", gy_x.bytes);
    addtoIMUQueue("G", "Y", gy_y.bytes);
    addtoIMUQueue("G", "Z", gy_z.bytes);
    addtoIMUQueue("A", "X", ax_x.bytes);
    addtoIMUQueue("A", "Y", ax_y.bytes);
    addtoIMUQueue("A", "Z", ax_z.bytes);

    osDelay(1000);

  }

  // In case we accidentally exit from task loop
  osThreadTerminate(NULL);

}

void addtoIMUQueue(char* type, char* dimension, uint8_t* data){
    IMU_msg_t imu_message;

    imu_message.imu_type[1] = type;
    imu_message.dimension[1] = dimension;
    for (int i = 0; i < 4; i++) {
        imu_message.data[i] = data[i];
    }

    osMessageQueuePut(imuMessageQueueHandle, &imu_message, 0U, 0U);
}

void StartTransmitData(void *argument){
  osStatus_t imu_queue_status;
  IMU_msg_t imu_message;

  while(1)
  {
    imu_queue_status = osMessageQueueGet(imuMessageQueueHandle, &imu_message, NULL, osWaitForever);

    if (imu_queue_status != osOK){
      osThreadYield();
    }
    // IMU ID: 1 ASCII characters
    uint8_t imu_id[1] = "@";
    HAL_UART_Transmit(&huart2, imu_id, sizeof(imu_id), 1000);

    transmitData(imu_message.imu_type, sizeof(imu_message.imu_type));
    transmitData(imu_message.dimension, sizeof(imu_message.dimension));
    transmitData(imu_message.data, sizeof(imu_message.data));

    // NEW LINE: 1 ASCII character
    uint8_t newline[1] = "\n";
    HAL_UART_Transmit(&huart2, newline, sizeof(newline), 1000);

    // CARRIAGE RETURN: 1 ASCII character
    uint8_t carriage[1] = "\r";
    HAL_UART_Transmit(&huart2, carriage, sizeof(carriage), 1000);
  }

  // In case we accidentally exit from task loop
  osThreadTerminate(NULL);
}
/* USER CODE END Application */
