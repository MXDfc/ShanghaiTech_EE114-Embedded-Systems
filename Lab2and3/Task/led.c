#include "led.h"

/*
 * led.c — LED 控制模块实现
 *
 * 整体分为三层：
 *   1. 硬件适配层（文件顶部宏 + LED_SetPin）：封装 GPIO 操作
 *   2. 状态管理层（s_enabled / s_mode）：记录 LED 的开关和当前模式
 *   3. 效果驱动层（LED_Process）：根据时钟实现各模式的视觉效果
 *
 * 更换 LED 所连接的外设（如换到其他 GPIO 端口或使用其他驱动方式），
 * 只需修改下方"硬件适配层"部分，上层状态与逻辑代码无需改动。
 */

/* =========================================================================
 * 硬件适配层
 *
 * LED 连接引脚：PB13（同时也是 TIM1_CH1N，LED_Init 会将其重配为 GPIO 输出）
 *
 * 极性配置（根据实际电路二选一）：
 *   共阴极（阴极接 GND）：高电平点亮
 *     LED_LEVEL_ON  = GPIO_PIN_SET
 *     LED_LEVEL_OFF = GPIO_PIN_RESET
 *   共阳极（阳极接 VCC，本开发板实际接法）：低电平点亮
 *     LED_LEVEL_ON  = GPIO_PIN_RESET
 *     LED_LEVEL_OFF = GPIO_PIN_SET
 * ========================================================================= */
#define LED_PORT        GPIOB
#define LED_PIN         GPIO_PIN_13
#define LED_LEVEL_ON    GPIO_PIN_RESET
#define LED_LEVEL_OFF   GPIO_PIN_SET

/*
 * LED_SetPin — 最底层引脚写入（不含亮度调制）
 *   on != 0 → 点亮；on == 0 → 熄灭
 *   所有对 GPIO 的写入均通过此函数，便于集中替换驱动方式。
 *   调用方需自行决定是否经过亮度调制；直接调用此函数会绕过亮度设置。
 */
static void LED_SetPin(uint8_t on)
{
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, on ? LED_LEVEL_ON : LED_LEVEL_OFF);
}

/* =========================================================================
 * 内部状态
 * ========================================================================= */
static uint8_t   s_enabled    = 0;               /* 0 = 关闭，1 = 开启 */
static LedMode_t s_mode       = LED_MODE_STEADY; /* 当前显示模式（关闭时也保留，再次开启时恢复）*/
static uint8_t   s_brightness = 100U;            /* 亮度级别，0 ~ 100，默认全亮 */

/* =========================================================================
 * 效果参数（可根据需要调整）
 * ========================================================================= */

/* 慢闪：半周期（亮/灭各持续此时长，单位 ms） */
#define SLOW_BLINK_HALF_MS   500U

/* 快闪：半周期 */
#define FAST_BLINK_HALF_MS   125U

/*
 * 呼吸灯：
 *   BREATH_PERIOD_MS — 完整呼吸周期（一次从全灭→全亮→全灭），单位 ms
 *   BREATH_PWM_MS    — 软件 PWM 周期，决定呼吸渐变的亮度分辨率
 *                      值越小越平滑，但对 LED_Process 调用频率要求越高
 *                      推荐：调用粒度 ≤ BREATH_PWM_MS / 10
 */
#define BREATH_PERIOD_MS     2000U
#define BREATH_PWM_MS        20U

/*
 * 亮度软件 PWM 周期（单位 ms）
 *
 * 用于常亮/慢闪/快闪模式在"点亮"阶段内进行占空比调制，模拟中间亮度。
 * 与呼吸灯的 BREATH_PWM_MS 刻意取不同的值，避免二者周期相同时
 * 在呼吸模式切换到其他模式时出现相位相关的视觉跳变。
 *
 * 取值原则：
 *   - 必须小于 FAST_BLINK_HALF_MS（125 ms），否则快闪"亮"段内
 *     亮度 PWM 还未完成一个完整周期，导致实际占空比偏离设定值
 *   - 推荐 ≤ 20 ms（人眼临界闪烁频率约 50 Hz），避免可见频闪
 *   - 调用粒度必须 ≤ BRIGHT_PWM_MS，否则亮度等级 < 100 时
 *     每个 PWM 周期只有 0 或 1 个采样点，输出退化为全亮或全灭
 */
#define BRIGHT_PWM_MS        10U

/*
 * LED_ApplyBrightness — 带亮度调制的输出辅助函数（常亮/慢闪/快闪模式专用）
 *
 * 参数：
 *   want_on — 当前模式逻辑希望 LED 处于点亮（1）还是熄灭（0）状态
 *   tick    — 当前系统毫秒计数，与 LED_Process 传入的 tick 一致
 *
 * 工作原理：
 *   - want_on == 0（模式逻辑要求熄灭）：直接熄灭，亮度不影响熄灭阶段
 *   - want_on == 1 且 s_brightness == 100：直接点亮，不引入 PWM 开销
 *   - want_on == 1 且 s_brightness == 0  ：直接熄灭
 *   - want_on == 1 且 0 < s_brightness < 100：
 *       在 BRIGHT_PWM_MS 周期内，前 (brightness * BRIGHT_PWM_MS / 100) ms
 *       点亮，其余时间熄灭，形成等效亮度占空比
 *
 * 注意：
 *   - 此函数不适用于呼吸灯模式；呼吸灯通过直接缩放 duty 值实现亮度控制
 *   - BRIGHT_PWM_MS 必须远小于各闪烁模式的半周期，否则"熄灭"阶段前
 *     会出现不完整的 PWM 周期导致亮度误差
 */
static void LED_ApplyBrightness(uint8_t want_on, uint32_t tick)
{
    if (!want_on)
    {
        /* 模式逻辑要求熄灭：直接关闭，亮度设置不作用于熄灭阶段 */
        LED_SetPin(0);
        return;
    }

    if (s_brightness >= 100U)
    {
        /* 全亮：跳过 PWM 计算，避免不必要的取模运算 */
        LED_SetPin(1);
        return;
    }

    if (s_brightness == 0U)
    {
        /* 亮度为零：等效熄灭，但不改变 s_enabled 状态 */
        LED_SetPin(0);
        return;
    }

    /*
     * 中间亮度：在 BRIGHT_PWM_MS 周期内按比例点亮
     * threshold 表示每个 PWM 周期内"点亮"所占的 ms 数
     * 例如 s_brightness=75，BRIGHT_PWM_MS=10 → threshold=7
     * 即每 10 ms 内前 7 ms 点亮，后 3 ms 熄灭，等效占空比 75%
     */
    uint32_t threshold = (uint32_t)s_brightness * BRIGHT_PWM_MS / 100U;
    LED_SetPin((tick % BRIGHT_PWM_MS) < threshold);
}

/* =========================================================================
 * 公开接口实现
 * ========================================================================= */

void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*
     * 先写熄灭电平，再配置 GPIO 方向。
     * 原因：HAL_GPIO_Init 执行期间引脚方向切换瞬间电平不确定，
     * 预先写入目标电平可防止 LED 在初始化时出现短暂随机闪亮。
     */
    LED_SetPin(0);

    GPIO_InitStruct.Pin   = LED_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP; /* 推挽输出，覆盖 CubeMX 的 AF 配置 */
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    s_enabled    = 0;
    s_mode       = LED_MODE_STEADY;
    s_brightness = 100U; /* 默认全亮；复位后亮度不依赖上电前的随机值 */
}

void LED_Enable(void)
{
    /*
     * 仅设置标志位；实际点亮由 LED_Process 在下次调用时执行。
     * 这样可确保 LED 的第一次输出状态与模式逻辑完全一致。
     */
    s_enabled = 1;
}

void LED_Disable(void)
{
    s_enabled = 0;
    LED_SetPin(0); /* 立即熄灭，不等待 LED_Process */
}

void LED_TogglePower(void)
{
    /*
     * 切换开关状态：开→关，关→开。
     * 封装判断逻辑，使调用方（如按键事件处理）代码更简洁。
     */
    if (s_enabled)
    {
        LED_Disable();
    }
    else
    {
        LED_Enable();
    }
}

void LED_NextMode(void)
{
    if (!s_enabled)
    {
        /* 关闭状态下切换模式无意义，直接返回 */
        return;
    }

    /*
     * 模式循环切换：利用 LED_MODE_COUNT 自动计算边界，
     * 新增或删除模式时只需修改枚举，此处代码无需改动。
     */
    s_mode = (LedMode_t)((s_mode + 1) % LED_MODE_COUNT);
}

void LED_Process(uint32_t tick)
{
    if (!s_enabled)
    {
        /*
         * 关闭状态：LED_Disable 已执行熄灭，此处直接返回。
         * 不重复调用 LED_SetPin，避免不必要的总线操作。
         */
        return;
    }

    switch (s_mode)
    {
        /* -----------------------------------------------------------------
         * 模式 0：常亮
         *   want_on 始终为 1，LED_ApplyBrightness 负责根据亮度决定是否
         *   引入 PWM；全亮时直接写高电平，无 PWM 开销。
         * ----------------------------------------------------------------- */
        case LED_MODE_STEADY:
            LED_ApplyBrightness(1, tick);
            break;

        /* -----------------------------------------------------------------
         * 模式 1：慢闪（约 1 Hz）
         *   先由闪烁逻辑判断当前是"亮"段还是"灭"段，
         *   再将结果传入 LED_ApplyBrightness：
         *     "灭"段 → 直接熄灭，亮度设置不影响熄灭阶段
         *     "亮"段 → 按亮度占空比输出
         *   所需调用粒度：≤ SLOW_BLINK_HALF_MS（500 ms），10 ms 满足。
         * ----------------------------------------------------------------- */
        case LED_MODE_SLOW_BLINK:
        {
            uint32_t period  = SLOW_BLINK_HALF_MS * 2U;
            uint8_t  want_on = (tick % period) < SLOW_BLINK_HALF_MS;
            LED_ApplyBrightness(want_on, tick);
            break;
        }

        /* -----------------------------------------------------------------
         * 模式 2：快闪（约 4 Hz）
         *   算法与慢闪完全相同，仅半周期参数不同。
         *   所需调用粒度：≤ FAST_BLINK_HALF_MS（125 ms），10 ms 满足。
         * ----------------------------------------------------------------- */
        case LED_MODE_FAST_BLINK:
        {
            uint32_t period  = FAST_BLINK_HALF_MS * 2U;
            uint8_t  want_on = (tick % period) < FAST_BLINK_HALF_MS;
            LED_ApplyBrightness(want_on, tick);
            break;
        }

        /* -----------------------------------------------------------------
         * 模式 3：呼吸灯（软件 PWM 模拟亮度线性渐变）
         *
         * 亮度控制方式：
         *   呼吸灯本身已是 PWM 调制，若再叠加一层亮度 PWM 会产生双层调制，
         *   导致实际占空比为两层 PWM 占空比的乘积，难以预测。
         *   因此改为将 s_brightness 作为线性缩放系数直接乘入 duty 值：
         *     scaled_duty = duty * s_brightness / 100
         *   效果：呼吸曲线形状不变，仅峰值亮度按比例压低。
         *   例如 s_brightness=50，峰值 duty 从 BREATH_PWM_MS 缩减至一半，
         *   视觉上整个呼吸周期的最亮点约为全亮的 50%。
         *
         * 所需调用粒度：
         *   1 ms 粒度可充分利用 BREATH_PWM_MS=20 的分辨率；
         *   10 ms 粒度时视觉有颗粒感，但节奏正确。
         * ----------------------------------------------------------------- */
        case LED_MODE_BREATH:
        {
            /* 在呼吸周期内的相位（0 ~ BREATH_PERIOD_MS-1） */
            uint32_t phase = tick % BREATH_PERIOD_MS;

            /* 计算呼吸曲线的原始 duty（0 ~ BREATH_PWM_MS） */
            uint32_t duty;
            if (phase < BREATH_PERIOD_MS / 2U)
            {
                duty = phase * BREATH_PWM_MS / (BREATH_PERIOD_MS / 2U);
            }
            else
            {
                duty = (BREATH_PERIOD_MS - phase) * BREATH_PWM_MS / (BREATH_PERIOD_MS / 2U);
            }

            /* 将亮度系数乘入 duty，压低峰值亮度 */
            uint32_t scaled_duty = duty * s_brightness / 100U;

            /* 根据缩放后的 duty 决定当前采样点的亮灭 */
            LED_SetPin((tick % BREATH_PWM_MS) < scaled_duty);
            break;
        }

        default:
            /* 防御性处理：状态异常时熄灭并复位为常亮 */
            s_mode = LED_MODE_STEADY;
            LED_SetPin(0);
            break;
    }
}

void LED_SetBrightness(uint8_t level)
{
    /*
     * 限幅：超出 [0, 100] 范围的值钳位到边界，防止调用方传入非法值
     * 导致 LED_ApplyBrightness 中 threshold > BRIGHT_PWM_MS 的溢出情况。
     */
    s_brightness = (level > 100U) ? 100U : level;
}

uint8_t LED_GetBrightness(void)
{
    return s_brightness;
}
