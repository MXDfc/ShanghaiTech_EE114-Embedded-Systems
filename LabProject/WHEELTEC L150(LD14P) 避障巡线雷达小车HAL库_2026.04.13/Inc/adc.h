/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Header.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

/* USER CODE BEGIN Private defines */
// ADC 通道宏定义
#define    CAR_ADC_CHANNEL                  ADC_CHANNEL_10
#define    ELE_ADC_L_CHANNEL					 ADC_CHANNEL_4
#define    ELE_ADC_M_CHANNEL					 ADC_CHANNEL_5
#define    ELE_ADC_R_CHANNEL					 ADC_CHANNEL_15
#define    CCD_ADC_CHANNEL					 	 ADC_CHANNEL_15

//单片机最大测量电压3.3V
#define Max_Voltage   				3.3f
//ADC读取最大数值4095
#define Max_Voltage_ADC				4095
//电池电压与读取电压的比例，11:1
#define Ratio 						11
//车型选择ADC电压最大是3.3/2V 
#define Max_Car_ADC					2047
//一共四个车
#define Num_Of_Car					4
/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);

/* USER CODE BEGIN Prototypes */
u16 Get_Adc(u8 ch);
void Robot_Select(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
