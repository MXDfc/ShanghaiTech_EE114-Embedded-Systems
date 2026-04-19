#include "include.h"
#include "IIC.h"
#include <string.h>

/******************************************************
//	Company:深圳幻尔科技有限公司					 //
//	作者：CuZn                                       //
//	我们的店铺:lobot-zone.taobao.com                 //
//	我们的官网:https://www.hiwonder.com/             //
******************************************************/
//hiwonder 语音识别模块例程

/* 语音识别功能例程 */

int main(void)
{
	SystemInit();
	InitDelay(72);																		//初始化延时函数
	Usart1_Init(); //串口初始化
	IIC_Init();		//IIC初始化
  DelayMs(200);
	printf("start");
	while (1)
	{
        Asr_Speak(ASR_CMDMAND , 0x01); // 命令词播报语 播报：正在前进
        DelayMs(5000);
        Asr_Speak(ASR_CMDMAND , 0x03); // 命令词播报语 播报：正在左转
        DelayMs(5000);
        Asr_Speak(ASR_ANNOUNCER , 0x01); // 播报语 播报：可回收物
        DelayMs(5000);
        Asr_Speak(ASR_ANNOUNCER , 0x03); // 播报语 播报：有害垃圾
        DelayMs(5000);
	}
}
