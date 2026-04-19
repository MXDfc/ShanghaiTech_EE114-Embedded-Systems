#include "main.h"
#include "stdint.h"
#include "OLED.h"
#include "OLED_Font.h"
#include <stdio.h>

/*引脚配置: OLED使用PC13(SCL)/PC14(SDA)软件I2C*/
#define OLED_I2C_PORT     GPIOC
#define OLED_SCL_PIN      GPIO_PIN_13
#define OLED_SDA_PIN      GPIO_PIN_14

#define I2C_SCL_HIGH      HAL_GPIO_WritePin(OLED_I2C_PORT, OLED_SCL_PIN, GPIO_PIN_SET)
#define I2C_SCL_LOW       HAL_GPIO_WritePin(OLED_I2C_PORT, OLED_SCL_PIN, GPIO_PIN_RESET)
#define I2C_SDA_HIGH      HAL_GPIO_WritePin(OLED_I2C_PORT, OLED_SDA_PIN, GPIO_PIN_SET)
#define I2C_SDA_LOW       HAL_GPIO_WritePin(OLED_I2C_PORT, OLED_SDA_PIN, GPIO_PIN_RESET)
/*引脚初始化*/
static void I2C_Delay(void)
{
    volatile uint32_t i = 1000; 
    while(i--);
    // 如果屏幕不显示或花屏，可以适当增大这里的初始值
}

/**
  * @brief  初始化 I2C 引脚
  * @note   需要在 MX_GPIO_Init() 之后调用，或者在此处重新配置引脚
  */
void OLED_I2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 开启 GPIOC 时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // 配置 PC13 (SCL) 和 PC14 (SDA)
    // 注意：STM32F103 的 PC13/PC14 驱动能力有限，使用推挽+低速
    GPIO_InitStruct.Pin = OLED_SCL_PIN | OLED_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(OLED_I2C_PORT, &GPIO_InitStruct);

    // 初始状态：SCL 和 SDA 都为高电平
    I2C_SCL_HIGH;
    I2C_SDA_HIGH;

    printf("  OLED: PC13/PC14 configured as PP, LOW speed\r\n");
}

/**
  * @brief  产生 I2C 起始信号
  */
void OLED_I2C_Start(void)
{
    I2C_SDA_HIGH;
    I2C_SCL_HIGH;
    I2C_Delay();
    I2C_SDA_LOW;  // SCL 高电平时，SDA 由高变低 -> 起始信号
    I2C_Delay();
    I2C_SCL_LOW;  // 钳住总线，准备发送数据
    I2C_Delay();
}

/**
  * @brief  产生 I2C 停止信号
  */
void OLED_I2C_Stop(void)
{
    I2C_SDA_LOW;
    I2C_SCL_LOW;
    I2C_Delay();
    I2C_SCL_HIGH;
    I2C_Delay();
    I2C_SDA_HIGH; // SCL 高电平时，SDA 由低变高 -> 停止信号
    I2C_Delay();
}

/**
  * @brief  I2C 发送一个字节 (核心函数)
  * @param  Byte: 要发送的数据
  * @note   此函数仅负责发送 8 位数据，不负责等待 ACK
  */
void OLED_I2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        if (Byte & 0x80)
        {
            I2C_SDA_HIGH;
        }
        else
        {
            I2C_SDA_LOW;
        }
        Byte <<= 1;

        I2C_SCL_HIGH;
        I2C_Delay();
        I2C_SCL_LOW;
        I2C_Delay();
    }

    // 第 9 个时钟：ACK 周期，释放 SDA 让从机拉低
    I2C_SDA_HIGH;       // 释放 SDA
    I2C_SCL_HIGH;       // 第 9 个 SCL 上升沿，从机在此拉低 SDA 表示 ACK
    I2C_Delay();
    I2C_SCL_LOW;        // 第 9 个 SCL 下降沿
    I2C_Delay();
}


/**
 * @brief  OLED写命令
 * @param  Command 要写入的命令
 * @retval 无
 */
static uint32_t oled_cmd_count = 0;

void OLED_WriteCommand(uint8_t Command)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78); //从机地址
	OLED_I2C_SendByte(0x00); //写命令
	OLED_I2C_SendByte(Command);
	OLED_I2C_Stop();
	oled_cmd_count++;
	if (oled_cmd_count <= 3)
	{
		printf("  OLED_CMD[%lu]: 0x%02X  SCL=%d SDA=%d\r\n",
		       (unsigned long)oled_cmd_count, Command,
		       (int)HAL_GPIO_ReadPin(OLED_I2C_PORT, OLED_SCL_PIN),
		       (int)HAL_GPIO_ReadPin(OLED_I2C_PORT, OLED_SDA_PIN));
	}
}

/**
 * @brief  OLED写数据
 * @param  Data 要写入的数据
 * @retval 无
 */
void OLED_WriteData(uint8_t Data)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78); //从机地址
	OLED_I2C_SendByte(0x40); //写数据
	OLED_I2C_SendByte(Data);
	OLED_I2C_Stop();
}

/**
 * @brief  OLED设置光标位置
 * @param  Y 以左上角为原点，向下方向的坐标，范围：0~7
 * @param  X 以左上角为原点，向右方向的坐标，范围：0~127
 * @retval 无
 */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	OLED_WriteCommand(0xB0 | Y);				 //设置Y位置
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4)); //设置X位置低4位
	OLED_WriteCommand(0x00 | (X & 0x0F));		 //设置X位置高4位
}

/**
 * @brief  OLED清屏
 * @param  无
 * @retval 无
 */
void OLED_Clear(void)
{
	uint8_t i, j;
	for (j = 0; j < 8; j++)
	{
		OLED_SetCursor(j, 0);
		for (i = 0; i < 128; i++)
		{
			OLED_WriteData(0x00);
		}
	}
}

/**
 * @brief  OLED部分清屏
 * @param  Line 行位置，范围：1~4
 * @param  start 列开始位置，范围：1~16
 * @param  end 列开始位置，范围：1~16
 * @retval 无
 */
void OLED_Clear_Part(uint8_t Line, uint8_t start, uint8_t end)
{
	uint8_t i, Column;
	for (Column = start; Column <= end; Column++)
	{
		OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8); //设置光标位置在上半部分
		for (i = 0; i < 8; i++)
		{
			OLED_WriteData(0x00); //显示上半部分内容
		}
		OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8); //设置光标位置在下半部分
		for (i = 0; i < 8; i++)
		{
			OLED_WriteData(0x00); //显示下半部分内容
		}
	}
}

/**
 * @brief  OLED显示一个字符
 * @param  Line 行位置，范围：1~4
 * @param  Column 列位置，范围：1~16
 * @param  Char 要显示的一个字符，范围：ASCII可见字符
 * @retval 无
 */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
	uint8_t i;
	OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8); //设置光标位置在上半部分
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i]); //显示上半部分内容
	}
	OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8); //设置光标位置在下半部分
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]); //显示下半部分内容
	}
}

/**
 * @brief  OLED显示字符串
 * @param  Line 起始行位置，范围：1~4
 * @param  Column 起始列位置，范围：1~16
 * @param  String 要显示的字符串，范围：ASCII可见字符
 * @retval 无
 */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowChar(Line, Column + i, String[i]);
	}
}



/**
 * @brief  OLED次方函数
 * @retval 返回值等于X的Y次方
 */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;
	}
	return Result;
}

/**
 * @brief  OLED显示数字（十进制，正数）
 * @param  Line 起始行位置，范围：1~4
 * @param  Column 起始列位置，范围：1~16
 * @param  Number 要显示的数字，范围：0~4294967295
 * @param  Length 要显示数字的长度，范围：1~10
 * @retval 无
 */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
 * @brief  OLED显示数字（十进制，带符号数）
 * @param  Line 起始行位置，范围：1~4
 * @param  Column 起始列位置，范围：1~16
 * @param  Number 要显示的数字，范围：-2147483648~2147483647
 * @param  Length 要显示数字的长度，范围：1~10
 * @retval 无
 */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;
	if (Number >= 0)
	{
		OLED_ShowChar(Line, Column, '+');
		Number1 = Number;
	}
	else
	{
		OLED_ShowChar(Line, Column, '-');
		Number1 = -Number;
	}
	for (i = 0; i < Length; i++)
	{
		OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
 * @brief  OLED显示数字（十六进制，正数）
 * @param  Line 起始行位置，范围：1~4
 * @param  Column 起始列位置，范围：1~16
 * @param  Number 要显示的数字，范围：0~0xFFFFFFFF
 * @param  Length 要显示数字的长度，范围：1~8
 * @retval 无
 */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)
	{
		SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
		if (SingleNumber < 10)
		{
			OLED_ShowChar(Line, Column + i, SingleNumber + '0');
		}
		else
		{
			OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
		}
	}
}

/**
 * @brief  OLED显示数字（二进制，正数）
 * @param  Line 起始行位置，范围：1~4
 * @param  Column 起始列位置，范围：1~16
 * @param  Number 要显示的数字，范围：0~1111 1111 1111 1111
 * @param  Length 要显示数字的长度，范围：1~16
 * @retval 无
 */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
	}
}

/**
 * @brief  OLED初始化
 * @param  无
 * @retval 无
 */
void OLED_Init(void)
{
	uint32_t i, j;

	printf("  OLED: power-on delay...\r\n");
	for (i = 0; i < 1000; i++) //上电延时
	{
		for (j = 0; j < 1000; j++)
			;
	}

	printf("  OLED: I2C pin init (PC13=SCL, PC14=SDA)...\r\n");
	OLED_I2C_Init(); //端口初始化

	/* --- GPIO readback test --- */
	{
		GPIO_PinState scl, sda;
		I2C_SCL_HIGH; I2C_SDA_HIGH;
		scl = HAL_GPIO_ReadPin(OLED_I2C_PORT, OLED_SCL_PIN);
		sda = HAL_GPIO_ReadPin(OLED_I2C_PORT, OLED_SDA_PIN);
		printf("  OLED: set HIGH -> SCL=%d SDA=%d\r\n", (int)scl, (int)sda);

		I2C_SCL_LOW; I2C_SDA_LOW;
		scl = HAL_GPIO_ReadPin(OLED_I2C_PORT, OLED_SCL_PIN);
		sda = HAL_GPIO_ReadPin(OLED_I2C_PORT, OLED_SDA_PIN);
		printf("  OLED: set LOW  -> SCL=%d SDA=%d\r\n", (int)scl, (int)sda);

		I2C_SCL_HIGH; I2C_SDA_HIGH;
		printf("  OLED: restored HIGH\r\n");
	}

	printf("  OLED: sending SSD1306 command sequence...\r\n");
	OLED_WriteCommand(0xAE); //关闭显示

	OLED_WriteCommand(0xD5); //设置显示时钟分频比/振荡器频率
	OLED_WriteCommand(0x80);

	OLED_WriteCommand(0xA8); //设置多路复用率
	OLED_WriteCommand(0x3F);

	OLED_WriteCommand(0xD3); //设置显示偏移
	OLED_WriteCommand(0x00);

	OLED_WriteCommand(0x40); //设置显示开始行

	OLED_WriteCommand(0xA1); //设置左右方向，0xA1正常 0xA0左右反置

	OLED_WriteCommand(0xC8); //设置上下方向，0xC8正常 0xC0上下反置

	OLED_WriteCommand(0xDA); //设置COM引脚硬件配置
	OLED_WriteCommand(0x12);

	OLED_WriteCommand(0x81); //设置对比度控制
	OLED_WriteCommand(0xCF);

	OLED_WriteCommand(0xD9); //设置预充电周期
	OLED_WriteCommand(0xF1);

	OLED_WriteCommand(0xDB); //设置VCOMH取消选择级别
	OLED_WriteCommand(0x30);

	OLED_WriteCommand(0xA4); //设置整个显示打开/关闭

	OLED_WriteCommand(0xA6); //设置正常/倒转显示

	OLED_WriteCommand(0x8D); //设置充电泵
	OLED_WriteCommand(0x14);

	OLED_WriteCommand(0xAF); //开启显示
	printf("  OLED: commands sent, clearing screen...\r\n");

	OLED_Clear(); // OLED清屏
	printf("  OLED: clear done\r\n");
}
