/**
  ******************************************************************************
  * @file    usart_i2c.h
  * @brief   This file contains all the function prototypes for
  *          the usart_i2c.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__


/* Includes ------------------------------------------------------------------*/
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

void MX_USART2_UART_Init(void);
void MX_I2C1_Init(void);
void MX_USART3_UART_Init(void);


#endif /* __USART_H__ */
