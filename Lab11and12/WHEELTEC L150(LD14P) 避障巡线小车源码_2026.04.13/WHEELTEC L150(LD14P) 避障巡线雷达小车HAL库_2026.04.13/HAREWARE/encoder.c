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

#include "encoder.h"

/**************************************************************************
Function: Read_Encoder
Input   : TIMX:Timer Number
Output  : Encoder data
函数功能：读取编码器
入口参数: TIMX: 编码器序号
返回  值：编码器读数
**************************************************************************/	 	
int Read_Encoder(u8 TIMX)
{
	int Encoder_TIM;    
	switch(TIMX)
	{
		case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0; break;
		case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0; break;	
		case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0; break;	
		case 8:  Encoder_TIM= (short)TIM8 -> CNT;  TIM8 -> CNT=0; break;	
		default:  Encoder_TIM=0;
	}
	return Encoder_TIM;
}
