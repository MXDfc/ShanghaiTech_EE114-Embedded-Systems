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
	InitDelay(72); //初始化延时函数
	Usart1_Init(); //串口初始化
	IIC_Init();		//IIC初始化
    DelayMs(200);
	printf("start");
	while (1)
	{		
		u8 result;
		result = Asr_Result(); //返回识别结果，即识别到的词条编号
		if(result != 0)
        {
            if(result == 0x01)
            {
                printf("go");
            }else if(result == 0x02)
            {
                printf("back");
            }else if(result == 0x03)
            {
                printf("left");
            }else if(result == 0x04)
            {
                printf("right");
            }else if(result == 0x09)
            {
                printf("stop");
            }
        }
		DelayMs(50);
	}
}
