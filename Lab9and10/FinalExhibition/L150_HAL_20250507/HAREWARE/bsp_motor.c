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

#include "bsp_motor.h"  


/**************************************************************************
Function: Drive_Motor
Input   : motor_a,motor_b:Indicates the left/right motor PWM duty cycle
Output  : none
函数功能：驱动电机
入口参数: motor_a和motor_b数值指示左右电机PWM占空比，值越大速度越快
返回  值：无
**************************************************************************/	 	
void Set_Pwm(int motor_a,int motor_b)
{
	
	if(Car_Num==Akm_Car)//阿克曼车需要控制舵机
		SERVO_TIM_SetCompareX_FUN(&htim1,TIM_CHANNEL_1,Servo_PWM);
	if(motor_a<0)		PWMA_IN1=7200,PWMA_IN2=7200+motor_a;
	else 	            PWMA_IN2=7200,PWMA_IN1=7200-motor_a;

	if(motor_b<0)		PWMB_IN1=7200,PWMB_IN2=7200+motor_b;
	else 	            PWMB_IN2=7200,PWMB_IN1=7200-motor_b;
}


