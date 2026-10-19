#include "voice_rec.h"
#include "i2c.h"

/*
 * 硬件地址与寄存器定义（与语音播报模块共用同一 I2C 设备）
 *
 *   ASR_DEV_ADDR    : 语音模块 I2C 7-bit 地址，HAL 要求传入 8-bit（左移 1 位）
 *   ASR_REG_SPEAK   : 播报命令寄存器，写入 [cmd, idNum] 触发播报
 *   ASR_REG_RESULT  : 识别结果寄存器，读取返回当前识别到的词条 ID
 *
 * 参考例程协议逆向（软件 IIC → HAL I2C）：
 *
 *   识别结果读取（Asr_Result）：
 *     写阶段：START → 写地址(0x34) → 寄存器(0x64) → STOP
 *     读阶段：START → 读地址(0x34) → 读 1 字节(NACK) → STOP
 *     HAL 等价：HAL_I2C_Mem_Read，内部自动完成两段事务
 *
 *   播报命令（Asr_Speak）：
 *     START → 写地址(0x34) → 寄存器(0x6E) → cmd → idNum → STOP
 *     HAL 等价：HAL_I2C_Mem_Write
 */
#define ASR_DEV_ADDR   (0x34U << 1)
#define ASR_REG_SPEAK  0x6EU
#define ASR_REG_RESULT 0x64U
#define ASR_I2C_TIMEOUT 20U

void ASR_Init(void)
{
    /* 等待语音模块上电稳定，参考例程使用 200 ms */
    HAL_Delay(200);
}

uint8_t ASR_GetResult(void)
{
    HAL_StatusTypeDef ret;
    uint8_t result = 0;

    ret = HAL_I2C_Mem_Read(&hi2c2,
                           ASR_DEV_ADDR,
                           ASR_REG_RESULT,
                           I2C_MEMADD_SIZE_8BIT,
                           &result,
                           1,
                           ASR_I2C_TIMEOUT);

    if (ret != HAL_OK)
    {
        return 0;
    }

    return result;
}

HAL_StatusTypeDef ASR_Speak(uint8_t cmd, uint8_t idNum)
{
    HAL_StatusTypeDef ret;
    uint8_t buf[2] = {cmd, idNum};

    ret = HAL_I2C_Mem_Write(&hi2c2,
                            ASR_DEV_ADDR,
                            ASR_REG_SPEAK,
                            I2C_MEMADD_SIZE_8BIT,
                            buf,
                            sizeof(buf),
                            ASR_I2C_TIMEOUT);

    HAL_Delay(20);

    return ret;
}
