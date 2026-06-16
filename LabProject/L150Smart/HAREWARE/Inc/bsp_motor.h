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

#ifndef __MOTOR_H
#define	__MOTOR_H


#include "Header.h"

#define    	SERVO_TIM                 TIM1
#define 	SERVO_TIM_SetCompareX_FUN   __HAL_TIM_SET_COMPARE

//这里是电机输出的PWM
#define PWMA_IN1 TIM3->CCR1
#define PWMA_IN2 TIM3->CCR2
#define PWMB_IN1 TIM3->CCR3
#define PWMB_IN2 TIM3->CCR4


void Set_Pwm(int motor_a,int motor_b);

#endif
