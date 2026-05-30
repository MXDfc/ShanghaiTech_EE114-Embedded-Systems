#include "car_ctrl.h"

void car_init(){
	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_4, GPIO_Mode_Out_PP);//初始化与LED连接的硬件接口
	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_5, GPIO_Mode_Out_PP);//初始化与LED连接的硬件接口
	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_6, GPIO_Mode_Out_PP);//初始化与LED连接的硬件接口
	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_7, GPIO_Mode_Out_PP);//初始化与LED连接的硬件接口
	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_8, GPIO_Mode_Out_PP);//初始化与LED连接的硬件接口
	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_9, GPIO_Mode_Out_PP);//初始化与LED连接的硬件接口
	
	MOTOR_B_PWM = 0;
	MOTOR_B_L = 0;
	MOTOR_B_R = 0;

	MOTOR_A_PWM = 0;
	MOTOR_A_L = 0;
	MOTOR_A_R = 0;
}

void car_forward(){
	MOTOR_B_PWM = 1;
	MOTOR_B_L = 1;
	MOTOR_B_R = 0;

	MOTOR_A_PWM = 1;
	MOTOR_A_L = 1;
	MOTOR_A_R = 0;
}

void car_back(void){
	MOTOR_B_PWM = 1;
	MOTOR_B_L = 0;
	MOTOR_B_R = 1;

	MOTOR_A_PWM = 1;
	MOTOR_A_L = 0;
	MOTOR_A_R = 1;
}

void car_brake(){
	MOTOR_B_PWM = 0;
	MOTOR_B_L = 0;
	MOTOR_B_R = 0;

	MOTOR_A_PWM = 0;
	MOTOR_A_L = 0;
	MOTOR_A_R = 0;
}

void car_turn_left_place(void){
	MOTOR_B_PWM = 1;
	MOTOR_B_L = 0;
	MOTOR_B_R = 1;

	MOTOR_A_PWM = 1;
	MOTOR_A_L = 1;
	MOTOR_A_R = 0;
}

void car_turn_right_place(void){
	MOTOR_B_PWM = 1;
	MOTOR_B_L = 1;
	MOTOR_B_R = 0;

	MOTOR_A_PWM = 1;
	MOTOR_A_L = 0;
	MOTOR_A_R = 1;
}

void car_turn_left(void){
	MOTOR_B_PWM = 0;
	MOTOR_B_L = 0;
	MOTOR_B_R = 0;

	MOTOR_A_PWM = 1;
	MOTOR_A_L = 1;
	MOTOR_A_R = 0;
}

void car_turn_right(void){
	MOTOR_B_PWM = 1;
	MOTOR_B_L = 1;
	MOTOR_B_R = 0;

	MOTOR_A_PWM = 0;
	MOTOR_A_L = 0;
	MOTOR_A_R = 0;
}

