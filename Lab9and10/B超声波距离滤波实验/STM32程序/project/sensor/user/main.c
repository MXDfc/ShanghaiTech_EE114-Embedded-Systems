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
	/* 原调试计数保留，正常运行不再使用
	int debug_cycle = 0;
	*/

	NVIC_init();
	SYSTICK_DelayInit();	     //延时函数初始化

	/* TIM2 now used by HC-SR04 PA1 input capture.
	TIM_ms_Init(TIM2, 500, TIM_IT_Update,2, ENABLE);	//初始化定时器2，定是时间500ms
	TIMER_CallbackInstall(HW_TIMER2, timer2);
	*/

	UART_QuickInit(HW_UART1, 9600, 2, 2, ENABLE);  //初始化串口1，波特率9600
	/* printf("[MAIN] UART1 ready: 9600 8N1\r\n"); */

	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_13, GPIO_Mode_Out_PP);//初始化与LED连接的硬件接口

	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_8, GPIO_Mode_IPU);//初始化与按键相连的引脚

	/* 开路自检打印保留但默认关闭
	printf("[MAIN] HC_SR04 open-circuit self-test: remove module and leave 4-pin header open\r\n");
	hc_sr04_open_circuit_self_test();
	*/

	hc_sr04_init();       //超声波初始化

	car_init();          //小车驱动初始化

	while(1){
		/* printf("[MAIN] cycle=%d start measure\r\n", debug_cycle++); */
		distance = get_hc_sr04_value_by_median_filter();
		SYSTICK_DelayMs(200);
		printf("distance = %d\r\n", distance);
		/*
		if(distance == 65535){
			printf("[MAIN] distance timeout value received, check HC_SR04 stage logs above\r\n");
		}
		*/
	}
}