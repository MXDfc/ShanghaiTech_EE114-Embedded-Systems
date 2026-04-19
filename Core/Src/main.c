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
#include "mpu6050.h"
#include "inv_mpu.h"
#include "ioi2c.h"
#include "OLED.h"
#include <stdio.h>
#include <string.h>
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
float pitch, roll, yaw;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Retarget printf to USART1 (Keil/ARMCC) */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
  return ch;
}

/* Display a float value on OLED as "+XXX.XX" at given line, starting at column */
static void OLED_ShowFloat(uint8_t Line, uint8_t Column, float val)
{
  char buf[10];
  int32_t int_part, dec_part;

  if (val >= 0) {
    int_part = (int32_t)val;
    dec_part = (int32_t)((val - int_part) * 100);
  } else {
    int_part = (int32_t)(-val);
    dec_part = (int32_t)((-val - int_part) * 100);
  }

  sprintf(buf, "%c%3d.%02d",
          (val >= 0) ? '+' : '-',
          (int)(int_part % 1000),
          (int)(dec_part % 100));
  OLED_ShowString(Line, Column, buf);
}

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
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();

  printf("\r\n=== System Boot ===\r\n");
  printf("[OK] HAL_Init, Clock, GPIO, I2C2, TIM1, USART1\r\n");

  /* USER CODE BEGIN 2 */
  printf("[..] OLED_Init start\r\n");
  OLED_Init();
  printf("[OK] OLED_Init done\r\n");

  printf("[..] OLED_ShowString \"MaoXudong\"\r\n");
  OLED_ShowString(1, 1, "MaoXudong");
  printf("[..] OLED_ShowString \"2023531071\"\r\n");
  OLED_ShowString(2, 1, "2023531071");
  printf("[OK] OLED_ShowString done\r\n");

  
  OLED_ShowString(1, 1, "MPU6050 Init...");
  MPU6050_Init();
  DMP_Init();

  OLED_Clear();
  OLED_ShowString(1, 1, "P:");
  OLED_ShowString(2, 1, "R:");
  OLED_ShowString(3, 1, "Y:");
  OLED_ShowString(4, 1, "  MPU6050 OK");

  /* OLED 软件 I2C 极慢(清屏~3s), 期间 FIFO 已溢出, 进循环前清空 */
  mpu_reset_fifo();
  HAL_Delay(30);
  printf("[OK] FIFO reset, entering main loop\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  {
    uint32_t last_oled_tick = 0;
    while (1)
    {

      /* USER CODE BEGIN 3 */

      /*
       * 1) 快速排空 FIFO: 把所有积压的 DMP 包读完，只保留最新值。
       *    这样即使 OLED 更新慢也不会让 FIFO 溢出。
       */
      {
        int i;
        float p_tmp, r_tmp, y_tmp;
        for (i = 0; i < 50; i++) {
          Read_DMP(&p_tmp, &r_tmp, &y_tmp);
          if (p_tmp != 0.0f || r_tmp != 0.0f || y_tmp != 0.0f) {
            pitch = p_tmp;
            roll  = r_tmp;
            yaw   = y_tmp;
          }
        }
      }

      /*
       * 2) 每 500ms 更新一次 OLED 和串口，
       *    更新完后清空 FIFO 防止下次进来就溢出。
       */
      if (HAL_GetTick() - last_oled_tick >= 500) {
        last_oled_tick = HAL_GetTick();

        printf("Pitch=%.2f\tRoll=%.2f\tYaw=%.2f\r\n", pitch, roll, yaw);
        OLED_ShowFloat(1, 3, pitch);
        OLED_ShowFloat(2, 3, roll);
        OLED_ShowFloat(3, 3, yaw);

        /* OLED 更新耗时长, 完成后清空积压的 FIFO */
        mpu_reset_fifo();
      }

      HAL_Delay(5);
      /* USER CODE END 3 */
    }
  }
  /* USER CODE END WHILE */
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
  /* Force a UART message out before disabling IRQs */
  {
    const char msg[] = "\r\n!!! Error_Handler entered !!!\r\n";
    HAL_UART_Transmit(&huart1, (const uint8_t *)msg, sizeof(msg) - 1, 100);
  }
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
