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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../Task/led.h"
#include "../../Task/key.h"
/* USER CODE END Includes */


/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*
 * 应用层有限状态机（FSM）状态枚举
 *
 * 本 FSM 描述 LED 系统在用户操作下的所有合法状态及其转换关系，
 * 将"当前处于什么状态"与"收到什么事件后该做什么"分开表达，
 * 避免用多层 if-else 嵌套隐式地处理状态逻辑。
 *
 * 状态转换图：
 *
 *              长按
 *   ┌──────────────────────────────────┐
 *   │                                  ▼
 * [APP_STATE_OFF]               [APP_STATE_ON]
 *   ▲    短按：忽略               │   短按：切换下一模式（循环）
 *   │                             │   长按：关闭 LED
 *   └─────────────────────────────┘
 *
 * 扩展说明：
 *   若需新增"锁定"、"亮度调节"等状态，只需在此枚举新增值，
 *   并在 APP_STATE_COUNT 之前插入，switch-case 的 default 分支
 *   会捕获未处理的非法状态，便于调试。
 */
typedef enum {
    APP_STATE_OFF   = 0,  /* LED 关闭：只响应长按（开启 LED） */
    APP_STATE_ON    = 1,  /* LED 开启：短按切换模式，长按关闭 */
    APP_STATE_COUNT = 2,  /* 状态总数，仅用于范围校验，勿作为实际状态使用 */
} AppState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*
 * 应用层 FSM 状态变量
 *
 * 放在文件作用域（而非 main 函数局部），原因：
 *   1. 未来若需在中断或其他模块中读取当前状态，可通过 extern 直接访问，
 *      无需修改 main 函数签名或引入全局 getter。
 *   2. 静态初始化为 APP_STATE_OFF，与 LED_Init 关闭 LED 的初始状态保持一致，
 *      保证系统上电后 FSM 状态与外设状态同步。
 */
static AppState_t s_app_state = APP_STATE_OFF;
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
  MX_I2C2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  KEY_Init();
  LED_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /*
   * 主循环设计原则：
   *   - 不使用 HAL_Delay，避免阻塞，确保 LED_Process 能以 ~1 ms 粒度被调用
   *   - 按键扫描以 10 ms 定时执行，满足消抖要求
   *   - 所有状态转换集中在应用层 FSM（s_app_state）中，逻辑一目了然
   */
  uint32_t last_key_tick = 0;
  while (1)
  {

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* ----------------------------------------------------------------
     * 【LED 效果驱动】每次主循环迭代均调用
     * LED_Process 内部会根据当前模式和 tick 决定 GPIO 输出，
     * 调用频率越高，呼吸灯模式的软件 PWM 越平滑。
     * ---------------------------------------------------------------- */
    LED_Process(now);

    /* ----------------------------------------------------------------
     * 【按键扫描 + 应用层 FSM】每 10 ms 执行一次
     *
     * FSM 转换规则（见 USER CODE BEGIN PTD 中的状态转换图）：
     *
     *   APP_STATE_OFF
     *     收到 KEY_EVENT_LONG  → 开启 LED，切换至 APP_STATE_ON
     *     收到 KEY_EVENT_SHORT → 忽略（关闭状态下短按无意义）
     *     收到 KEY_EVENT_NONE  → 无动作
     *
     *   APP_STATE_ON
     *     收到 KEY_EVENT_SHORT → 切换 LED 下一显示模式，状态不变
     *     收到 KEY_EVENT_LONG  → 关闭 LED，切换至 APP_STATE_OFF
     *     收到 KEY_EVENT_NONE  → 无动作
     *
     * 所有状态-事件组合均被显式列举，default 分支捕获非法状态，
     * 防止因内存异常等原因导致状态变量越界时系统行为不可预期。
     * ---------------------------------------------------------------- */
    if (now - last_key_tick >= 10U)
    {
      last_key_tick = now;

      KeyEvent_t evt = KEY_Scan();

      switch (s_app_state)
      {
        /* ──────────────────────────────────────────────────────────
         * 状态：APP_STATE_OFF（LED 处于关闭状态）
         * 仅响应长按事件，短按事件在此状态下被静默丢弃。
         * 丢弃短按而非报错，是因为用户误触短按属于正常操作，
         * 不应引发任何视觉反馈以免造成困惑。
         * ────────────────────────────────────────────────────────── */
        case APP_STATE_OFF:
          if (evt == KEY_EVENT_LONG)
          {
            LED_Enable();              /* 恢复上次记忆的显示模式 */
            s_app_state = APP_STATE_ON;
          }
          /* KEY_EVENT_SHORT / KEY_EVENT_NONE：不做任何处理 */
          break;

        /* ──────────────────────────────────────────────────────────
         * 状态：APP_STATE_ON（LED 处于开启状态）
         * 短按切换显示模式，长按关闭 LED。
         * 两种事件均不共存（KEY_Scan 每次最多返回一个事件），
         * 因此无需考虑优先级问题。
         * ────────────────────────────────────────────────────────── */
        case APP_STATE_ON:
          if (evt == KEY_EVENT_SHORT)
          {
            LED_NextMode();            /* 循环切换：常亮→慢闪→快闪→呼吸→常亮→… */
          }
          else if (evt == KEY_EVENT_LONG)
          {
            LED_Disable();             /* 立即熄灭 */
            s_app_state = APP_STATE_OFF;
          }
          /* KEY_EVENT_NONE：不做任何处理 */
          break;

        /* ──────────────────────────────────────────────────────────
         * 防御性 default：状态值越界时强制复位到安全状态
         * 触发条件：内存异常、栈溢出等导致 s_app_state 被意外篡改
         * ────────────────────────────────────────────────────────── */
        default:
          LED_Disable();
          s_app_state = APP_STATE_OFF;
          break;
      }
    }
    /* USER CODE END 3 */
  }
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
