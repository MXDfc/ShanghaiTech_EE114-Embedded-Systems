# include <stdio.h>
#include "stm32f10x.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"

extern u32 Distance; 

 /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/


//------------接线说明---------------
//HC-SR04丝印标识--------STM32F1主板引脚

//    VCC     -----------    3.3V   
//    Trig    -----------    B1   
//    Echo    -----------    B0  
//    GND     -----------    GND  

//------------接线说明---------------


 int main(void)
 {	
	SystemInit(); 																				//配置系统时钟为72M   
	 delay_init();	    	 																//延时函数初始化	 
	 uart_init(9600);   														      //串口初始化
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  	  // 设置中断优先级分组2
	 TIM3_Cap_Init(0XFFFF,72-1);//TIM_CH3输入捕获
  while(1)
	{
	  Read_Distane(); //读超声波来回的时间  和  当前距离
		printf("当前距离=%d  mm\r\n",Distance);//串口打印输出测得距离
	}
 }

