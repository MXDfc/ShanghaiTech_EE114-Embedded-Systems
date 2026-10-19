#ifndef _HC_SR04_H_
#define _HC_SR04_H_

#include "sysinc.h"

//#define HC_SR04_TIMERX TIM3
#define HC_SR04_TIMERX TIM2

//#define TRIG PAout(15)     //需要在初始化函数中修改GPIO初始化
#define TRIG PCout(15)     //现用TRIG接到PC15
#define TRIG_READ PCin(15) //回读PC15实际电平，用于调试TRIG是否真的输出
// #define ECHO PAin(12)
#define ECHO PAin(1)       //现用ECHO接到PA1

//获取超声波距离可能最长会阻塞程序50ms

void hc_sr04_init(void);
u16 get_hc_sr04_value(void);
u16 get_hc_sr04_value_by_median_filter(void);  //中值滤波后得到超声波测得距离
void hc_sr04_open_circuit_self_test(void);  //模块拔掉时的裸板自检

#endif
