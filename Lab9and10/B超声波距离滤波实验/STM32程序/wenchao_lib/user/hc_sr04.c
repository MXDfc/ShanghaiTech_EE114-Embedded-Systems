#include "hc_sr04.h"

#if 0
//u8 flag_hc_sr04_outtime = 0;            //超时标志位
volatile u8 flag_hc_sr04_outtime = 0;   //超时标志位，中断中修改，主循环读取

static volatile u8 hc_sr04_debug_wait_stage = 0;
static volatile u16 hc_sr04_debug_timeout_irq_count = 0;
static volatile u16 hc_sr04_debug_timeout_irq_counter = 0;
static volatile u8 hc_sr04_debug_timeout_irq_echo = 0;
static volatile u8 hc_sr04_debug_timeout_irq_trig_odr = 0;
static volatile u8 hc_sr04_debug_timeout_irq_trig_idr = 0;
static volatile u8 hc_sr04_debug_timeout_irq_stage = 0;
static volatile u16 hc_sr04_debug_timeout_irq_cr1 = 0;
static volatile u16 hc_sr04_debug_timeout_irq_dier = 0;
static volatile u16 hc_sr04_debug_timeout_irq_sr = 0;
static volatile u16 hc_sr04_debug_timeout_irq_psc = 0;
static volatile u16 hc_sr04_debug_timeout_irq_arr = 0;

static const char *hc_sr04_debug_stage_name(u8 stage)
{
	switch(stage){
		case 0: return "idle";
		case 1: return "wait echo low before trigger";
		case 2: return "wait rising edge";
		case 3: return "wait falling edge";
		default: return "unknown";
	}
}

static void hc_sr04_debug_print_timer_regs(const char *stage)
{
	printf("[HC_SR04] timer regs[%s]: CR1=0x%04X, DIER=0x%04X, SR=0x%04X, CNT=%u, PSC=%u, ARR=%u\r\n",
	       stage,
	       (unsigned int)HC_SR04_TIMERX->CR1,
	       (unsigned int)HC_SR04_TIMERX->DIER,
	       (unsigned int)HC_SR04_TIMERX->SR,
	       (unsigned int)HC_SR04_TIMERX->CNT,
	       (unsigned int)HC_SR04_TIMERX->PSC,
	       (unsigned int)HC_SR04_TIMERX->ARR);
}

static void hc_sr04_debug_print_state(const char *stage, u16 counter)
{
	printf("[HC_SR04] %s, counter=%u, echo=%u, trig_odr=%u, trig_idr=%u, wait_stage=%u(%s), timeout=%u\r\n",
	       stage,
	       (unsigned int)counter,
	       (unsigned int)ECHO,
	       (unsigned int)TRIG,
	       (unsigned int)TRIG_READ,
	       (unsigned int)hc_sr04_debug_wait_stage,
	       hc_sr04_debug_stage_name(hc_sr04_debug_wait_stage),
	       (unsigned int)flag_hc_sr04_outtime);
}

static void hc_sr04_debug_print_timeout_latch(const char *reason)
{
	printf("[HC_SR04] timeout detail[%s]: irq_count=%u, irq_counter=%u, irq_echo=%u, irq_trig_odr=%u, irq_trig_idr=%u, irq_stage=%u(%s), irq_CR1=0x%04X, irq_DIER=0x%04X, irq_SR=0x%04X, irq_PSC=%u, irq_ARR=%u\r\n",
	       reason,
	       (unsigned int)hc_sr04_debug_timeout_irq_count,
	       (unsigned int)hc_sr04_debug_timeout_irq_counter,
	       (unsigned int)hc_sr04_debug_timeout_irq_echo,
	       (unsigned int)hc_sr04_debug_timeout_irq_trig_odr,
	       (unsigned int)hc_sr04_debug_timeout_irq_trig_idr,
	       (unsigned int)hc_sr04_debug_timeout_irq_stage,
	       hc_sr04_debug_stage_name(hc_sr04_debug_timeout_irq_stage),
	       (unsigned int)hc_sr04_debug_timeout_irq_cr1,
	       (unsigned int)hc_sr04_debug_timeout_irq_dier,
	       (unsigned int)hc_sr04_debug_timeout_irq_sr,
	       (unsigned int)hc_sr04_debug_timeout_irq_psc,
	       (unsigned int)hc_sr04_debug_timeout_irq_arr);
}

static void hc_sr04_debug_reset_timeout_latch(void)
{
	hc_sr04_debug_timeout_irq_count = 0;
	hc_sr04_debug_timeout_irq_counter = 0;
	hc_sr04_debug_timeout_irq_echo = 0;
	hc_sr04_debug_timeout_irq_trig_odr = 0;
	hc_sr04_debug_timeout_irq_trig_idr = 0;
	hc_sr04_debug_timeout_irq_stage = 0;
	hc_sr04_debug_timeout_irq_cr1 = 0;
	hc_sr04_debug_timeout_irq_dier = 0;
	hc_sr04_debug_timeout_irq_sr = 0;
	hc_sr04_debug_timeout_irq_psc = 0;
	hc_sr04_debug_timeout_irq_arr = 0;
}

static void hc_sr04_debug_probe_echo_line(void)
{
	u8 echo_float = 0;
	u8 echo_pulldown = 0;
	u8 echo_pullup = 0;
	u8 echo_restore = 0;

	//GPIO_QuickInit(HW_GPIOA, GPIO_Pin_12, GPIO_Mode_IN_FLOATING);
	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_1, GPIO_Mode_IN_FLOATING);
	SYSTICK_DelayMs(2);
	echo_float = ECHO;

	//GPIO_QuickInit(HW_GPIOA, GPIO_Pin_12, GPIO_Mode_IPD);
	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_1, GPIO_Mode_IPD);
	SYSTICK_DelayMs(2);
	echo_pulldown = ECHO;

	//GPIO_QuickInit(HW_GPIOA, GPIO_Pin_12, GPIO_Mode_IPU);
	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_1, GPIO_Mode_IPU);
	SYSTICK_DelayMs(2);
	echo_pullup = ECHO;

	//GPIO_QuickInit(HW_GPIOA, GPIO_Pin_12, GPIO_Mode_IN_FLOATING);
	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_1, GPIO_Mode_IN_FLOATING);
	SYSTICK_DelayMs(2);
	echo_restore = ECHO;

	printf("[HC_SR04] echo probe: floating=%u, pulldown=%u, pullup=%u, restore=%u\r\n",
	       (unsigned int)echo_float, (unsigned int)echo_pulldown,
	       (unsigned int)echo_pullup, (unsigned int)echo_restore);

	if(echo_float == 1 && echo_pulldown == 1 && echo_pullup == 1){
		printf("[HC_SR04] echo probe result: PA1 is externally driven or strongly pulled high\r\n");
	}else if(echo_float == 0 && echo_pulldown == 0 && echo_pullup == 0){
		printf("[HC_SR04] echo probe result: PA1 is held low even with pull-up; Echo/module may be driving low or line may be shorted to GND\r\n");
	}else if(echo_pulldown == 0 && echo_pullup == 1){
		printf("[HC_SR04] echo probe result: PA1 looks floating or weakly driven; check Echo wiring\r\n");
	}else{
		printf("[HC_SR04] echo probe result: PA1 level changes with input mode; inspect wiring and module state\r\n");
}
}

static void hc_sr04_debug_probe_trig_line(void)
{
	u8 trig_float = 0;
	u8 trig_pulldown = 0;
	u8 trig_pullup = 0;
	u8 trig_out_low = 0;
	u8 trig_out_high = 0;

	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_15, GPIO_Mode_IN_FLOATING);
	SYSTICK_DelayMs(2);
	trig_float = TRIG_READ;

	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_15, GPIO_Mode_IPD);
	SYSTICK_DelayMs(2);
	trig_pulldown = TRIG_READ;

	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_15, GPIO_Mode_IPU);
	SYSTICK_DelayMs(2);
	trig_pullup = TRIG_READ;

	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_15, GPIO_Mode_Out_PP);
	TRIG = 0;
	SYSTICK_DelayMs(2);
	trig_out_low = TRIG_READ;
	TRIG = 1;
	SYSTICK_DelayMs(2);
	trig_out_high = TRIG_READ;
	TRIG = 0;

	printf("[HC_SR04] trig probe: floating=%u, pulldown=%u, pullup=%u, out_low=%u, out_high=%u\r\n",
	       (unsigned int)trig_float, (unsigned int)trig_pulldown,
	       (unsigned int)trig_pullup, (unsigned int)trig_out_low,
	       (unsigned int)trig_out_high);

	if(trig_pulldown == 0 && trig_pullup == 1 && trig_out_low == 0 && trig_out_high == 1){
		printf("[HC_SR04] trig probe result: PC15 responds normally with module disconnected\r\n");
	}else{
		printf("[HC_SR04] trig probe result: PC15 abnormal under open-circuit test; inspect board trace, LSE circuit, or short\r\n");
	}

	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_15, GPIO_Mode_Out_PP);
	TRIG = 0;
}

static void hc_sr04_debug_probe_open_cross_short(void)
{
	u8 echo_ipd_trig_low = 0;
	u8 echo_ipd_trig_high = 0;
	u8 echo_ipu_trig_low = 0;
	u8 echo_ipu_trig_high = 0;

	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_1, GPIO_Mode_IPD);
	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_15, GPIO_Mode_Out_PP);
	TRIG = 0;
	SYSTICK_DelayMs(2);
	echo_ipd_trig_low = ECHO;
	TRIG = 1;
	SYSTICK_DelayMs(2);
	echo_ipd_trig_high = ECHO;

	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_1, GPIO_Mode_IPU);
	TRIG = 0;
	SYSTICK_DelayMs(2);
	echo_ipu_trig_low = ECHO;
	TRIG = 1;
	SYSTICK_DelayMs(2);
	echo_ipu_trig_high = ECHO;
	TRIG = 0;

	printf("[HC_SR04] open short test: echo_ipd(trig0=%u,trig1=%u), echo_ipu(trig0=%u,trig1=%u)\r\n",
	       (unsigned int)echo_ipd_trig_low,
	       (unsigned int)echo_ipd_trig_high,
	       (unsigned int)echo_ipu_trig_low,
	       (unsigned int)echo_ipu_trig_high);

	if(echo_ipd_trig_low == 0 && echo_ipd_trig_high == 0 &&
	   echo_ipu_trig_low == 1 && echo_ipu_trig_high == 1){
		printf("[HC_SR04] open short result: PA1 does not follow PC15, Echo/Trig short is unlikely\r\n");
	}else{
		printf("[HC_SR04] open short result: PA1 changes when PC15 toggles; check Echo-Trig short or header solder bridge\r\n");
	}
}
void hc_sr04_open_circuit_self_test(void)
{
	printf("[HC_SR04] open-circuit self-test begin: remove HC-SR04, keep VCC/TRIG/ECHO/GND header open\r\n");
	printf("[HC_SR04] open-circuit self-test note: software can test PA1(Echo) and PC15(Trig); VCC/GND still need meter check\r\n");
	printf("[HC_SR04] open-circuit expected: Echo PA1 pulldown=0 pullup=1; Trig PC15 pulldown=0 pullup=1 out_low=0 out_high=1\r\n");
	hc_sr04_debug_probe_echo_line();
	hc_sr04_debug_probe_trig_line();
	hc_sr04_debug_probe_open_cross_short();
	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_1, GPIO_Mode_IN_FLOATING);
	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_15, GPIO_Mode_Out_PP);
	TRIG = 0;
	printf("[HC_SR04] open-circuit self-test end\r\n");
}

void HC_SR04_timer(void)
{
	if (TIM_GetITStatus(HC_SR04_TIMERX, TIM_IT_Update) != RESET){  //检查TIM3更新中断发生与否
		flag_hc_sr04_outtime = 1;        //超时
		hc_sr04_debug_timeout_irq_count++;
		hc_sr04_debug_timeout_irq_counter = TIM_GetCounter(HC_SR04_TIMERX);
		hc_sr04_debug_timeout_irq_echo = ECHO;
		hc_sr04_debug_timeout_irq_trig_odr = TRIG;
		hc_sr04_debug_timeout_irq_trig_idr = TRIG_READ;
		hc_sr04_debug_timeout_irq_stage = hc_sr04_debug_wait_stage;
		hc_sr04_debug_timeout_irq_cr1 = HC_SR04_TIMERX->CR1;
		hc_sr04_debug_timeout_irq_dier = HC_SR04_TIMERX->DIER;
		hc_sr04_debug_timeout_irq_sr = HC_SR04_TIMERX->SR;
		hc_sr04_debug_timeout_irq_psc = HC_SR04_TIMERX->PSC;
		hc_sr04_debug_timeout_irq_arr = HC_SR04_TIMERX->ARR;
		TIM_Cmd(HC_SR04_TIMERX, DISABLE);
		TIM_ClearITPendingBit(HC_SR04_TIMERX, TIM_IT_Update);  //清除TIMx更新中断标志
	}
}

/**
 * @return[out]  超声波初始化函数---如果更改超声波端口，请更改这里的GPIO配置
 */
void hc_sr04_init(void)
{
	//GPIO_QuickInit(HW_GPIOA, GPIO_Pin_15, GPIO_Mode_Out_PP);//tric
	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_15, GPIO_Mode_Out_PP);//trig

	//GPIO_QuickInit(HW_GPIOA, GPIO_Pin_12, GPIO_Mode_IPU);//echo
	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_1, GPIO_Mode_IN_FLOATING);//echo
	hc_sr04_debug_probe_echo_line();

	TRIG = 0;
	hc_sr04_debug_wait_stage = 0;
	SYSTICK_DelayMs(2);
	printf("[HC_SR04] init pins ready: trig=PC15(out), echo=PA1(in float), trig_odr=%u, trig_idr=%u, echo=%u\r\n",
	       (unsigned int)TRIG, (unsigned int)TRIG_READ, (unsigned int)ECHO);

	printf("[HC_SR04] init system clock=%u\r\n", (unsigned int)SystemCoreClock);
	printf("[HC_SR04] timer init request: ARR=40000, PSC=72, expect about 40ms timeout\r\n");
	TIM_us_Init_org(HC_SR04_TIMERX, 40000, 72);       //定时40ms，最多测试680cm
	hc_sr04_debug_print_timer_regs("after TIM_us_Init_org");
	TIM_Cmd(HC_SR04_TIMERX, DISABLE);
	TIM_SetCounter(HC_SR04_TIMERX, 0);
	TIM_ClearITPendingBit(HC_SR04_TIMERX, TIM_IT_Update);
	HC_SR04_TIMERX->SR = 0;
	hc_sr04_debug_print_timer_regs("after raw SR clear");
	printf("[HC_SR04] init done, trig=PC15, echo=PA1, timer=TIM3\r\n");

	if(HC_SR04_TIMERX == TIM2) 	TIMER_CallbackInstall(HW_TIMER2, HC_SR04_timer);
	else if(HC_SR04_TIMERX == TIM3) 	TIMER_CallbackInstall(HW_TIMER3, HC_SR04_timer);
	else if(HC_SR04_TIMERX == TIM4) 	TIMER_CallbackInstall(HW_TIMER4, HC_SR04_timer);
}

/**
 * @return[out]  返回测得的超声波的距离  单位mm
 * @node  当返回值为65535，代表超时
 */
u16 get_hc_sr04_value(void)
{
	u8 timeout_stage = 0;
	u8 trig_odr_before = 0;
	u8 trig_idr_before = 0;
	u8 trig_odr_high = 0;
	u8 trig_idr_high = 0;
	u8 trig_odr_after = 0;
	u8 trig_idr_after = 0;
	u8 echo_during_trig = 0;

	flag_hc_sr04_outtime = 0;
	hc_sr04_debug_reset_timeout_latch();
	hc_sr04_debug_wait_stage = 0;
	TIM_Cmd(HC_SR04_TIMERX, DISABLE);
	TIM_SetCounter(HC_SR04_TIMERX, 0);
	TIM_ClearITPendingBit(HC_SR04_TIMERX, TIM_IT_Update);
	HC_SR04_TIMERX->SR = 0;
	hc_sr04_debug_print_timer_regs("measure after raw SR clear");
	hc_sr04_debug_print_state("measure start", TIM_GetCounter(HC_SR04_TIMERX));

	if(ECHO){
		printf("[HC_SR04] echo high before trigger, waiting for idle low\r\n");
		hc_sr04_debug_wait_stage = 1;
		TIM_Cmd(HC_SR04_TIMERX, ENABLE);
		while(ECHO && flag_hc_sr04_outtime == 0);
		TIM_Cmd(HC_SR04_TIMERX, DISABLE);
	if(flag_hc_sr04_outtime == 1){
		timeout_stage = 1;
		TIM_Cmd(HC_SR04_TIMERX, DISABLE);
		printf("[HC_SR04] timeout while waiting rising edge, echo=%u, counter=%u\r\n", (unsigned int)ECHO, (unsigned int)TIM_GetCounter(HC_SR04_TIMERX));
		hc_sr04_debug_print_timeout_latch("wait rising edge");
		hc_sr04_debug_print_timer_regs("after wait rising timeout disable");
		hc_sr04_debug_wait_stage = 0;
		flag_hc_sr04_outtime = 0;
		return 65535;
	}else{
		hc_sr04_debug_print_state("rising edge detected", TIM_GetCounter(HC_SR04_TIMERX));
	}
	}

	hc_sr04_debug_print_state("before trigger", TIM_GetCounter(HC_SR04_TIMERX));
	trig_odr_before = TRIG;
	trig_idr_before = TRIG_READ;
	TRIG = 1;
	SYSTICK_DelayUs(15);
	trig_odr_high = TRIG;
	trig_idr_high = TRIG_READ;
	echo_during_trig = ECHO;
	TRIG = 0;
	trig_odr_after = TRIG;
	trig_idr_after = TRIG_READ;
	printf("[HC_SR04] trigger pulse: before(odr=%u,idr=%u), high(odr=%u,idr=%u,echo=%u), after(odr=%u,idr=%u)\r\n",
	       (unsigned int)trig_odr_before,
	       (unsigned int)trig_idr_before,
	       (unsigned int)trig_odr_high,
	       (unsigned int)trig_idr_high,
	       (unsigned int)echo_during_trig,
	       (unsigned int)trig_odr_after,
	       (unsigned int)trig_idr_after);
	if(trig_odr_high != 1 || trig_idr_high != 1){
		printf("[HC_SR04] trigger warning: PC15 did not read high cleanly during pulse, check PC15 wiring or pin conflict\r\n");
	}
	if(trig_odr_after != 0 || trig_idr_after != 0){
		printf("[HC_SR04] trigger warning: PC15 did not return low after pulse, current level may be abnormal\r\n");
	}

	hc_sr04_debug_wait_stage = 2;
	TIM_SetCounter(HC_SR04_TIMERX, 0);
	TIM_ClearITPendingBit(HC_SR04_TIMERX, TIM_IT_Update);
	HC_SR04_TIMERX->SR = 0;
	hc_sr04_debug_print_timer_regs("before wait rising enable");
	hc_sr04_debug_print_state("wait rising begin", TIM_GetCounter(HC_SR04_TIMERX));
	TIM_Cmd(HC_SR04_TIMERX, ENABLE);
	hc_sr04_debug_print_timer_regs("after wait rising enable");
	while(!ECHO && flag_hc_sr04_outtime == 0);
	if(flag_hc_sr04_outtime == 1){
		timeout_stage = 1;
		TIM_Cmd(HC_SR04_TIMERX, DISABLE);
		printf("[HC_SR04] timeout while waiting rising edge, echo=%u, counter=%u\r\n", (unsigned int)ECHO, (unsigned int)TIM_GetCounter(HC_SR04_TIMERX));
		hc_sr04_debug_print_timeout_latch("wait rising edge");
		hc_sr04_debug_print_timer_regs("after wait rising timeout disable");
		hc_sr04_debug_wait_stage = 0;
		flag_hc_sr04_outtime = 0;
		return 65535;
	}else{
		hc_sr04_debug_print_state("rising edge detected", TIM_GetCounter(HC_SR04_TIMERX));
	}

	hc_sr04_debug_wait_stage = 3;
	TIM_SetCounter(HC_SR04_TIMERX, 0);
	TIM_ClearITPendingBit(HC_SR04_TIMERX, TIM_IT_Update);
	HC_SR04_TIMERX->SR = 0;
	hc_sr04_debug_print_state("counter reset", TIM_GetCounter(HC_SR04_TIMERX));
	TIM_Cmd(HC_SR04_TIMERX, ENABLE);

	while(ECHO && flag_hc_sr04_outtime == 0);
	TIM_Cmd(HC_SR04_TIMERX, DISABLE);

	if(flag_hc_sr04_outtime == 1){
		if(timeout_stage == 0){
			timeout_stage = 2;
		}
		printf("[HC_SR04] timeout stage=%u, echo=%u, counter=%u\r\n", (unsigned int)timeout_stage, (unsigned int)ECHO, (unsigned int)TIM_GetCounter(HC_SR04_TIMERX));
		hc_sr04_debug_print_timeout_latch("wait falling edge");
		hc_sr04_debug_wait_stage = 0;
		flag_hc_sr04_outtime = 0;
		return 65535;
	}

	hc_sr04_debug_print_state("echo low", TIM_GetCounter(HC_SR04_TIMERX));
	hc_sr04_debug_wait_stage = 0;
	/* 原始返回逻辑保留：
	return TIM_GetCounter(HC_SR04_TIMERX)*340/200;   //返回测的距离
	*/
	{
		u16 raw_counter = TIM_GetCounter(HC_SR04_TIMERX);
		u16 distance = raw_counter*340/200;
		printf("[HC_SR04] raw_counter=%u, distance=%u\r\n", (unsigned int)raw_counter, (unsigned int)distance);
		return distance;   //返回测的距离
	}
}

#define MEDIAN_FILTER_LEN 3

u16 get_hc_sr04_value_by_median_filter(void)  //中值滤波后得到超声波测得距离
{
	u16 dis[MEDIAN_FILTER_LEN] = {0};
	u16 tmp = 0;
	for(u8 i=0; i<MEDIAN_FILTER_LEN; i++){
		dis[i] = get_hc_sr04_value();
		printf("[HC_SR04] sample[%u]=%u\r\n", (unsigned int)i, (unsigned int)dis[i]);
	}

	for(u8 i=0; i<MEDIAN_FILTER_LEN-1; i++){
		for(u8 j=0; j<MEDIAN_FILTER_LEN-1-i; j++){
			if(dis[j] > dis[j+1]){
				tmp = dis[j];
				dis[j] = dis[j+1];
				dis[j+1] = tmp;
			}
		}
	}
	printf("[HC_SR04] sorted samples: %u, %u, %u\r\n", (unsigned int)dis[0], (unsigned int)dis[1], (unsigned int)dis[2]);
	return dis[MEDIAN_FILTER_LEN/2];
}
#endif

#define HC_SR04_CAPTURE_TIMER TIM2
#define HC_SR04_CAPTURE_CH CAP_CH2
#define HC_SR04_CAPTURE_DONE 0x80
#define HC_SR04_CAPTURE_RISING 0x40
#define HC_SR04_CAPTURE_OVF_MASK 0x3F
#define HC_SR04_TIMEOUT_VALUE 65535
#define HC_SR04_WAIT_TIMEOUT_MS 60

volatile u8 flag_hc_sr04_outtime = 0;

static volatile u8 hc_sr04_capture_sta = 0;
static volatile u16 hc_sr04_capture_val = 0;

static void hc_sr04_capture_reset(void)
{
	flag_hc_sr04_outtime = 0;
	hc_sr04_capture_sta = 0;
	hc_sr04_capture_val = 0;
	TIM_SetCounter(HC_SR04_CAPTURE_TIMER, 0);
	TIM_OC2PolarityConfig(HC_SR04_CAPTURE_TIMER, TIM_ICPolarity_Rising);
	TIM_ClearITPendingBit(HC_SR04_CAPTURE_TIMER, TIM_IT_CC2 | TIM_IT_Update);
}

void HC_SR04_timer(void)
{
	if((hc_sr04_capture_sta & HC_SR04_CAPTURE_DONE) == 0){
		if(TIM_GetITStatus(HC_SR04_CAPTURE_TIMER, TIM_IT_Update) != RESET){
			if(hc_sr04_capture_sta & HC_SR04_CAPTURE_RISING){
				if((hc_sr04_capture_sta & HC_SR04_CAPTURE_OVF_MASK) == HC_SR04_CAPTURE_OVF_MASK){
					hc_sr04_capture_sta |= HC_SR04_CAPTURE_DONE;
					hc_sr04_capture_val = 0xFFFF;
					flag_hc_sr04_outtime = 1;
				}else{
					hc_sr04_capture_sta++;
				}
			}
		}

		if(TIM_GetITStatus(HC_SR04_CAPTURE_TIMER, TIM_IT_CC2) != RESET){
			if(hc_sr04_capture_sta & HC_SR04_CAPTURE_RISING){
				hc_sr04_capture_sta |= HC_SR04_CAPTURE_DONE;
				hc_sr04_capture_val = TIM_GetCapture2(HC_SR04_CAPTURE_TIMER);
				TIM_OC2PolarityConfig(HC_SR04_CAPTURE_TIMER, TIM_ICPolarity_Rising);
			}else{
				hc_sr04_capture_sta = 0;
				hc_sr04_capture_val = 0;
				TIM_SetCounter(HC_SR04_CAPTURE_TIMER, 0);
				hc_sr04_capture_sta |= HC_SR04_CAPTURE_RISING;
				TIM_OC2PolarityConfig(HC_SR04_CAPTURE_TIMER, TIM_ICPolarity_Falling);
			}
		}
	}

	TIM_ClearITPendingBit(HC_SR04_CAPTURE_TIMER, TIM_IT_CC2 | TIM_IT_Update);
}

void hc_sr04_init(void)
{
	//GPIO_QuickInit(HW_GPIOA, GPIO_Pin_15, GPIO_Mode_Out_PP);//tric
	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_15, GPIO_Mode_Out_PP);//trig
	TRIG = 0;

	//GPIO_QuickInit(HW_GPIOA, GPIO_Pin_12, GPIO_Mode_IPU);//echo
	TIM_Cap_Init(HC_SR04_CAPTURE_TIMER, HC_SR04_CAPTURE_CH, 0xFFFF, 72 - 1, 2, 0);
	TIMER_CallbackInstall(HW_TIMER2, HC_SR04_timer);
	hc_sr04_capture_reset();
}

u16 get_hc_sr04_value(void)
{
	u8 wait_ms = 0;
	u32 pulse_us = 0;
	u32 distance = 0;

	hc_sr04_capture_reset();

	TRIG = 1;
	SYSTICK_DelayUs(15);
	TRIG = 0;

	while((hc_sr04_capture_sta & HC_SR04_CAPTURE_DONE) == 0 && wait_ms < HC_SR04_WAIT_TIMEOUT_MS){
		SYSTICK_DelayMs(1);
		wait_ms++;
	}

	if((hc_sr04_capture_sta & HC_SR04_CAPTURE_DONE) == 0 || hc_sr04_capture_val == 0xFFFF){
		hc_sr04_capture_sta = 0;
		return HC_SR04_TIMEOUT_VALUE;
	}

	pulse_us = (u32)(hc_sr04_capture_sta & HC_SR04_CAPTURE_OVF_MASK) * 65536u + hc_sr04_capture_val;
	if(pulse_us == 0){
		return HC_SR04_TIMEOUT_VALUE;
	}

	distance = pulse_us * 340u / 2000u;
	if(distance > HC_SR04_TIMEOUT_VALUE){
		return HC_SR04_TIMEOUT_VALUE;
	}
	return (u16)distance;
}

#define MEDIAN_FILTER_LEN 3

u16 get_hc_sr04_value_by_median_filter(void)
{
	u8 i = 0;
	u8 j = 0;
	u16 dis[MEDIAN_FILTER_LEN] = {0};
	u16 tmp = 0;

	for(i = 0; i < MEDIAN_FILTER_LEN; i++){
		dis[i] = get_hc_sr04_value();
		SYSTICK_DelayMs(60);
	}

	for(i = 0; i < MEDIAN_FILTER_LEN - 1; i++){
		for(j = 0; j < MEDIAN_FILTER_LEN - 1 - i; j++){
			if(dis[j] > dis[j + 1]){
				tmp = dis[j];
				dis[j] = dis[j + 1];
				dis[j + 1] = tmp;
			}
		}
	}

	return dis[MEDIAN_FILTER_LEN / 2];
}

void hc_sr04_open_circuit_self_test(void)
{
	/* Debug self-test disabled in normal capture driver. Old version is kept above. */
}