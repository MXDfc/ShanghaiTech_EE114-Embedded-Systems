/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2023-03-02

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2023-03-02

All rights reserved
***********************************************/
#include "Header.h"
volatile u8 delay_50,delay_flag; 		//延时变量			
u16 Voltage;							//电压变量，放大100倍储存
u8 Flag_Stop;							//电机启停标志位
u8 PS2_ON_Flag = 0,APP_ON_Flag = 0,ROS_ON_Flag = 0;		//默认所有方式不控制
u8 Car_Num;								//车型号码选择
u8 Flag_Show = 1;						//显示标志位，默认开启，长按切换到上位机模式，此时关闭
float Perimeter; 						//轮子的周长
float Wheelspacing; 					//轮子的轮距
u16 DISTANCE=0,ANGLE=0;
u8 one_lap_data_success_flag=0;         //雷达数据完成一圈的接收标志位
int lap_count=0;//当前雷达这一圈数据有多少个点
int PointDataProcess_count=0,test_once_flag=0,Dividing_point=0;//雷达接收数据点的计算值、接收到一圈数据最后一帧数据的标志位、需要切割数据的数据数
/**************************************************************************  
Function: Main function
Input   : none
Output  : none
函数功能：主函数
入口参数: 无 
返回  值：无
**************************************************************************/	 	
int main(void)
{	

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//中断优先级分组
	OLED_Init();						//OLED初始化
	DEBUG_USART_Init();					//调试串口设置，串口1，波特率115200
	IIC_Init();                     //IIC初始化
	MPU6050_Init();										//MPU6050初始化
	DMP_Init();                     //初始化DMP 
	while(1)
	{
	        Read_DMP();
			Show();												//显示屏      	
	}
}


/***********************************END OF FILE********************************/

