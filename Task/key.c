#include "key.h"

/*
 * key.c — 按键事件检测（消抖 + 短按/长按状态机）
 *
 * 硬件连接：
 *   PB14 — 按键 — GND
 *   PB14 上拉输入（CubeMX 已配置），按下时读到低电平
 *
 * 状态机设计：
 *   IDLE  ──(检测到低电平)──▶  PRESS（记录按下时刻）
 *   PRESS ──(持续按下 ≥ KEY_LONG_PRESS_MS)──▶  触发长按，进入 LONG_HELD
 *   PRESS ──(释放，且持续 ≥ KEY_DEBOUNCE_MS)──▶  触发短按，回到 IDLE
 *   PRESS ──(释放，持续 < KEY_DEBOUNCE_MS)──▶  视为抖动，回到 IDLE
 *   LONG_HELD ──(释放)──▶  静默回到 IDLE（长按已触发，不再补发短按）
 */

/* =========================================================================
 * 硬件适配层：更换按键引脚时仅修改此处
 * ========================================================================= */
#define KEY_PORT          GPIOB
#define KEY_PIN           GPIO_PIN_14

/* 按键按下时为低电平（上拉接法） */
#define KEY_IS_DOWN()     (HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN) == GPIO_PIN_RESET)

/* =========================================================================
 * 时序参数（单位：ms；均为 KEY_Scan 调用周期的整数倍）
 * ========================================================================= */

/* 消抖最短按压时长：按下后至少保持此时长才视为有效按键，过滤机械抖动 */
#define KEY_DEBOUNCE_MS     20U

/* 长按判定阈值：按下并持续超过此时长即触发长按事件 */
#define KEY_LONG_PRESS_MS   800U

/* =========================================================================
 * 内部状态机状态定义
 * ========================================================================= */
typedef enum {
    KEY_STATE_IDLE      = 0,  /* 空闲：按键未被按下 */
    KEY_STATE_PRESS     = 1,  /* 已按下：等待释放或达到长按阈值 */
    KEY_STATE_LONG_HELD = 2,  /* 长按已触发：等待释放，期间静默 */
} KeyState_t;

/* =========================================================================
 * 公开接口实现
 * ========================================================================= */

void KEY_Init(void)
{
    /*
     * PB14 已由 CubeMX（MX_GPIO_Init）配置为 GPIO_Input / Pull-up，
     * 此处无需额外 HAL 调用，保留此函数作为统一初始化入口。
     */
}

KeyEvent_t KEY_Scan(void)
{
    /*
     * 注意：所有 static 变量在首次调用前均为零值（C 标准保证），
     * 等价于 s_state = KEY_STATE_IDLE，s_press_tick = 0。
     */
    static KeyState_t s_state      = KEY_STATE_IDLE;
    static uint32_t   s_press_tick = 0;

    KeyEvent_t event = KEY_EVENT_NONE;

    switch (s_state)
    {
        /* -----------------------------------------------------------------
         * 空闲状态：等待按键按下
         * ----------------------------------------------------------------- */
        case KEY_STATE_IDLE:
            if (KEY_IS_DOWN())
            {
                /* 检测到按下：记录时刻，切换至 PRESS 状态 */
                s_press_tick = HAL_GetTick();
                s_state      = KEY_STATE_PRESS;
            }
            break;

        /* -----------------------------------------------------------------
         * 按下状态：同时监测释放（短按）和长按超时
         * ----------------------------------------------------------------- */
        case KEY_STATE_PRESS:
            if (KEY_IS_DOWN())
            {
                /* 按键仍处于按下状态：检查是否达到长按阈值 */
                if (HAL_GetTick() - s_press_tick >= KEY_LONG_PRESS_MS)
                {
                    /*
                     * 达到长按阈值：立即上报长按事件，
                     * 切换至 LONG_HELD 状态以屏蔽后续重复触发
                     */
                    event   = KEY_EVENT_LONG;
                    s_state = KEY_STATE_LONG_HELD;
                }
                /* 未达到阈值：继续等待，本次调用返回 KEY_EVENT_NONE */
            }
            else
            {
                /* 检测到释放：判断按压时长是否满足消抖要求 */
                if (HAL_GetTick() - s_press_tick >= KEY_DEBOUNCE_MS)
                {
                    /* 有效短按：上报短按事件 */
                    event = KEY_EVENT_SHORT;
                }
                /* 否则视为机械抖动，静默丢弃 */
                s_state = KEY_STATE_IDLE;
            }
            break;

        /* -----------------------------------------------------------------
         * 长按已触发状态：等待松手，期间不产生任何事件
         * ----------------------------------------------------------------- */
        case KEY_STATE_LONG_HELD:
            if (!KEY_IS_DOWN())
            {
                /* 检测到释放：回到空闲状态，不再触发短按 */
                s_state = KEY_STATE_IDLE;
            }
            break;

        default:
            s_state = KEY_STATE_IDLE;
            break;
    }

    return event;
}
