#ifndef _HC_SR04_H_
#define _HC_SR04_H_

#include "sysinc.h"

#define HC_SR04_TIMERX TIM3

#define TRIG PAout(15)     //需要在初始化函数中修改GPIO初始化
#define ECHO PAin(12)

//获取超声波距离可能最长会阻塞程序50ms

void hc_sr04_init(void);
u16 get_hc_sr04_value(void);

#endif
