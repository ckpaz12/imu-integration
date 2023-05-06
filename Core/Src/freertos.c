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

void addtoIMUQueue(char* type, char* dimension, union FloatBytes data);

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

__NO_RETURN void StartReadIMU(void *argument)
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

    // IMU DATA: 16 ASCII characters
    addtoIMUQueue("G", "X", gy_x);
    addtoIMUQueue("G", "Y", gy_y);
    addtoIMUQueue("G", "Z", gy_z);
    addtoIMUQueue("A", "X", ax_x);
    addtoIMUQueue("A", "Y", ax_y);
    addtoIMUQueue("A", "Z", ax_z);

    osDelay(1000);

  }

  // In case we accidentally exit from task loop
  osThreadTerminate(NULL);

}

void addtoIMUQueue(char* type, char* dimension, union FloatBytes data){
    IMU_msg_t imu_message;

    imu_message.imu_type = type[0];
    imu_message.dimension = dimension[0];
    for (int i = 0; i < 4; i++) {
        imu_message.data[i] = data.bytes[i];
    }

    osMessageQueuePut(imuMessageQueueHandle, &imu_message, 0U, 0U);
}

__NO_RETURN void StartTransmitData(void *argument){
  osStatus_t imu_queue_status;
  IMU_msg_t imu_message;

  while(1)
  {
    // Check if there are messages in the queue
    if (osMessageQueueGetCount(imuMessageQueueHandle) == 0) {
        continue;
    }

    imu_queue_status = osMessageQueueGet(imuMessageQueueHandle, &imu_message, NULL, osWaitForever);

    if (imu_queue_status != osOK){
      osThreadYield();
    }
    uint8_t imu_buffer[9] = {0};

    // IMU ID: 1 ASCII characters
    imu_buffer[0] = '@';

    //Data from queue
    imu_buffer[1] = imu_message.imu_type;
    imu_buffer[2] = imu_message.dimension;
    for (int i = 0; i < 4; i++) {
        imu_buffer[i + 3] = imu_message.data[i];
    }

    // NEW LINE: 1 ASCII character
    imu_buffer[7] = '\n';

    // CARRIAGE RETURN: 1 ASCII character
    imu_buffer[8] = '\r';

    HAL_UART_Transmit(&huart2, imu_buffer, sizeof(imu_buffer), 1000);
  }

  // In case we accidentally exit from task loop
  osThreadTerminate(NULL);
}
/* USER CODE END Application */
