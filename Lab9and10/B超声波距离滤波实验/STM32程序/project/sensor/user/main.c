#include "sysinc.h"
#include "car_ctrl.h"
#include "hc_sr04.h"

#define KEY_CTRL PAin(8)

void timer2()
{	
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){  //检查TIM3更新中断发生与否
		  PCout(13)= !PCout(13);
	  	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除TIMx更新中断标志 
	}
}

int main(void){
	int distance = 0;
	 
	NVIC_init();
	SYSTICK_DelayInit();	    	 //延时函数初始化	  	
	
	TIM_ms_Init(TIM2, 500, TIM_IT_Update,2, ENABLE);	//初始化定时器2，定是时间500ms
 	TIMER_CallbackInstall(HW_TIMER2, timer2);	 
	
	UART_QuickInit(HW_UART1, 9600, 2, 2, ENABLE);  //初始化串口1，波特率9600
	 
	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_13, GPIO_Mode_Out_PP);//初始化与LED连接的硬件接口
	 
	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_8, GPIO_Mode_IPU);//初始化与按键相连的引脚
	
	hc_sr04_init();       //超声波初始化
	
	car_init();          //小车驱动初始化
	
	while(1){  

		distance = get_hc_sr04_value_by_median_filter();
		SYSTICK_DelayMs(200);
		printf("distance = %d\n", distance);
		
	}
}
 

