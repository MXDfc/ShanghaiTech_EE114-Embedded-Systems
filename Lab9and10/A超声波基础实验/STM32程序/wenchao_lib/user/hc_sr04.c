#include "hc_sr04.h"

u8 flag_hc_sr04_outtime = 0;            //超时标志位

void HC_SR04_timer(){
	if (TIM_GetITStatus(HC_SR04_TIMERX, TIM_IT_Update) != RESET){  //检查TIM3更新中断发生与否
		  
		  flag_hc_sr04_outtime  = 1;        //超时
		
	  	TIM_ClearITPendingBit(HC_SR04_TIMERX, TIM_IT_Update);  //清除TIMx更新中断标志 
	}
}

/**
 * @return[out]  超声波初始化函数---如果更改超声波端口，请更改这里的GPIO配置
 */
void hc_sr04_init(void){
	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_15, GPIO_Mode_Out_PP);//tric
	 
	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_12, GPIO_Mode_IPU);//echo
	
	TRIG = 0;
	
	TIM_us_Init_org(HC_SR04_TIMERX, 40000, 72);	       //定时40ms，最多测试680cm
	TIM_Cmd(HC_SR04_TIMERX, DISABLE);  
	
	if(HC_SR04_TIMERX == TIM2) 	TIMER_CallbackInstall(HW_TIMER2, HC_SR04_timer);	 
	else if(HC_SR04_TIMERX == TIM3) 	TIMER_CallbackInstall(HW_TIMER3, HC_SR04_timer);	 
	else if(HC_SR04_TIMERX == TIM4) 	TIMER_CallbackInstall(HW_TIMER4, HC_SR04_timer);	 
	
}

/**
 * @return[out]  返回测得的超声波的距离  单位mm
 */
u16 get_hc_sr04_value(void){
	flag_hc_sr04_outtime = 0;
	TRIG = 1;
	SYSTICK_DelayUs(15);
	TRIG = 0;
	
	TIM_Cmd(HC_SR04_TIMERX, ENABLE);  
	while(!ECHO && flag_hc_sr04_outtime == 0);
	TIM_SetCounter(HC_SR04_TIMERX,0);
	TIM_Cmd(HC_SR04_TIMERX, ENABLE); 
	
	while(ECHO && flag_hc_sr04_outtime == 0); 
	TIM_Cmd(HC_SR04_TIMERX,DISABLE);
	
	if(flag_hc_sr04_outtime == 1){
		flag_hc_sr04_outtime = 0;
		return 65535;
	}
	
	return TIM_GetCounter(HC_SR04_TIMERX)*340/200;   //返回测的距离
}
