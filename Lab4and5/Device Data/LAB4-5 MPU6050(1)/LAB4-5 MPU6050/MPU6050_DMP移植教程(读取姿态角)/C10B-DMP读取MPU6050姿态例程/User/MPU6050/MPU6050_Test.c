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

#include "MPU6050.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "Header.h"
#include "ioi2c.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "inv_mpu.h"
#include "filter.h"


#define INT PBin(9)




//简单的测试程序，进行测试的时候需要注释掉原来的main函数，然后把测试程序中注释掉的恢复
//接上mup6050模块可以看到角度变化


///**************************************************************************
//Function: Get angle
//Input   : way：The algorithm of getting angle 1：DMP  2：kalman  3：Complementary filtering
//Output  : none
//函数功能：获取角度	
//入口参数：way：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
//返回  值：无
//**************************************************************************/	
//void Get_Angle(u8 way)
//{ 
//	float Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
//	if(way==1)                           //DMP的读取在数据采集中断读取，严格遵循时序要求
//	{	
//		Read_DMP();                      //读取加速度、角速度、倾角
//	}			
//	else
//	{
//		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取X轴陀螺仪
//		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪
//		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
//		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度计
//		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度计
//		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计
//		if(Gyro_X>32768)  Gyro_X-=65536;                 //数据类型转换  也可通过short强制类型转换
//		if(Gyro_Y>32768)  Gyro_Y-=65536;                 //数据类型转换  也可通过short强制类型转换
//		if(Gyro_Z>32768)  Gyro_Z-=65536;                 //数据类型转换
//		if(Accel_X>32768) Accel_X-=65536;                //数据类型转换
//		if(Accel_Y>32768) Accel_Y-=65536;                //数据类型转换
//		if(Accel_Z>32768) Accel_Z-=65536;                //数据类型转换
//		Accel_Angle_x=atan2(Accel_Y,Accel_Z)*180/Pi;     //计算倾角，转换单位为度	
//		Accel_Angle_y=atan2(Accel_X,Accel_Z)*180/Pi;     //计算倾角，转换单位为度
//		Gyro_X=Gyro_X/16.4;                              //陀螺仪量程转换，量程±2000°/s对应灵敏度16.4，可查手册
//		Gyro_Y=Gyro_Y/16.4;                              //陀螺仪量程转换	
//		if(way==2)		  	
//		{
//			Roll= -Kalman_Filter_x(Accel_Angle_x,Gyro_X);//卡尔曼滤波
//			Pitch = -Kalman_Filter_y(Accel_Angle_y,Gyro_Y);
//		}
//		else if(way==3) 
//		{  
//			Roll = -Complementary_Filter_x(Accel_Angle_x,Gyro_X);//互补滤波
//			Pitch= -Complementary_Filter_y(Accel_Angle_y,Gyro_Y);
//		}
//	}
//}
/***********以下是测试程序，需注释main.c中的主函数******************/

///**************************************************************************
//Function: main
//Input   : none
//Output  : none
//函数功能：测试程序主程序	
//入口参数：无
//返回  值：无
//**************************************************************************/	
//int main(void)
//{
//	
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//中断优先级分组
//	DEBUG_USART_Init();					//调试串口设置，串口1，波特率115200
//	LED_GPIO_Config();					//LED初始化，PC2和PC3
//	OLED_Init();						//OLED初始化
//	MPU6050_Init();						//MPU6050初始化
//	while(1)
//	{
//		/****************显示屏显示角度*******************/
//		OLED_ShowString(0,0,"Pitch:");
//		if(Pitch<0)		
//		{
//			OLED_ShowString(70,0,"-");	
//			OLED_ShowNumber(80,0,-(int)Pitch,3,12);
//		}
//		else
//		{
//			OLED_ShowString(70,0,"+");	
//			OLED_ShowNumber(80,0,(int)Pitch,3,12);
//		}	
//		
//		OLED_ShowString(0,10,"Roll:");
//		if(Pitch<0)		
//		{
//			OLED_ShowString(70,10,"-");	
//			OLED_ShowNumber(80,10,-(int)Roll,3,12);
//		}
//		else
//		{
//			OLED_ShowString(70,10,"+");	
//			OLED_ShowNumber(80,10,(int)Roll,3,12);
//		}			
//		
//		
//		OLED_ShowString(0,20,"Yaw:");
//		if(Pitch<0)		
//		{
//			OLED_ShowString(70,20,"-");	
//			OLED_ShowNumber(80,20,-(int)Yaw,3,12);
//		}
//		else
//		{
//			OLED_ShowString(70,20,"+");	
//			OLED_ShowNumber(80,20,(int)Yaw,3,12);

//		}			
//		OLED_Refresh_Gram();	
//		/****************显示屏显示角度*******************/
//	}
	
	
//}



/**************************************************************************
Function: EXTI9_5_IRQHandler
Input   : none
Output  : none
函数功能：中断读取数据，外部中断	
入口参数：无
返回  值：无
**************************************************************************/	
int EXTI9_5_IRQHandler(void) 
{
	if(INT==0)		
	{
		EXTI->PR=1<<9;    //清除中断        
		Get_Angle(2);     //获取角度，1.DMP 2.卡尔曼滤波 3.互补滤波 									

	}

	return 0;	  

}


