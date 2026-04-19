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

#include "show.h"

/**************************************************************************
Function: OLED_Show
Input   : none
Output  : none
函数功能：显示屏显示函数
入口参数: 无 
返回  值：无
**************************************************************************/	 	
void Show(void)
{
	memset(OLED_GRAM,0, 128*8*sizeof(u8));	//GRAM清零但不立即刷新，防止花屏
	 OLED_ShowString(0,0,"Roll ");
	if(Roll>0) OLED_ShowString(55,0,"+");
	else
		OLED_ShowString(55,0,"-");
	OLED_ShowNumber(75,0,myabs(Roll),5,12);
	    
	
	    OLED_ShowString(0,15,"Pitch ");
	if(Pitch>0) OLED_ShowString(55,15,"+");
	else
		OLED_ShowString(55,15,"-");
	OLED_ShowNumber(75,15,myabs(Pitch),5,12);
	   
	    OLED_ShowString(0,30,"Yaw ");
	if(Yaw>0) OLED_ShowString(55,30,"+");
	else
		OLED_ShowString(55,30,"-");
	OLED_ShowNumber(75,30,myabs(Yaw),5,12);
	//==================刷新==================//
	OLED_Refresh_Gram();	
}

/**************************************************************************
Function: OLED_Show_CCD
Input   : none
Output  : none
函数功能：CCD模式显示函数，画点
入口参数: 无 
返回  值：无
**************************************************************************/	 	

void OLED_DrawPoint_Shu(u8 x,u8 y,u8 t)
{ 
	u8 i=0;
	OLED_DrawPoint(x,y,t);
	OLED_DrawPoint(x,y,t);
	for(i = 0;i<8; i++)
	{
		OLED_DrawPoint(x,y+i,t);
	}
}

void OLED_Show_CCD(void)
{ 
	u8 i,t;
	for(i = 0;i<128; i++)
	{
		if(CCD_ADV[i]<CCD_Threshold) t=1; else t=0;
		OLED_DrawPoint_Shu(i,0,t);
	}
}

/**************************************************************************
Function: Car_Select_Show
Input   : none
Output  : none
函数功能：车型选择显示
入口参数: 无 
返回  值：无
**************************************************************************/	 	
void Car_Select_Show(void)
{						  
	OLED_ShowString(0,00,"Rotate Resistor");
	OLED_ShowString(0,10,"To Select Car");
	OLED_ShowString(0,20,"Current Car Is");
	if(Car_Num==Diff_Car)         			OLED_ShowString(24,30," Diff Car ");//差速小车
	if(Car_Num==Akm_Car)        			OLED_ShowString(24,30,"  Akm Car ");//阿克曼
	if(Car_Num==Small_Tank_Car)				OLED_ShowString(24,30,"S_Tank Car");//小履带车
	if(Car_Num==Big_Tank_Car)				OLED_ShowString(24,30,"B_Tank Car");//大履带车
	
	OLED_ShowString(0,40,"Press User Key");
	OLED_ShowString(0,50,"To End Selection");
	
	OLED_Refresh_Gram();	
}

/**************************************************************************
Function: Send data to APP
Input   : none
Output  : none
函数功能：向APP发送数据
入口参数：无
返回  值：无
**************************************************************************/
void APP_Show(void)
{    
	static u8 flag;
	int Velocity_Left_Show,Velocity_Right_Show,Voltage_Show;
	Voltage_Show=(Voltage-1000)*2/3;			if(Voltage_Show<0)Voltage_Show=0;if(Voltage_Show>100) Voltage_Show=100;   //对电压数据进行处理
	Velocity_Right_Show=MotorB.Velocity*1.1; 	if(Velocity_Right_Show<0) Velocity_Right_Show=-Velocity_Right_Show;			  //对编码器数据就行数据处理便于图形化
	Velocity_Left_Show= MotorA.Velocity*1.1;  	if(Velocity_Left_Show<0) Velocity_Left_Show=-Velocity_Left_Show;
	flag=!flag;
	if(PID_Send==1)			//发送PID参数,在APP调参界面显示
	{
		 printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",(int)RC_Velocity,(int)CCD_Move_X,(int)0,(int)0,(int)0,(int)0,(int)0,0,0);//打印到APP上面
		 PID_Send=0;		
  }
   else	if(flag==0)		// 发送电池电压，速度，角度等参数，在APP首页显示
	   printf("{A%d:%d}$",(int)Voltage_Show,(int)0); //打印到APP上面
	 else				//发送小车姿态角，在波形界面显示
	    printf("{B%d:%d}$",(int)MotorA.Current_Encoder,(int)MotorB.Current_Encoder); //显示左右编码器的速度																	   //可按格式自行增加显示波形，最多可显示五个
}


//上位机示波器
/**************************************************************************
函数功能：虚拟示波器往上位机发送数据 关闭显示屏
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void DataScope(void)
{   
	u8 i;			//计数变量
	u8 Send_Count;	//串口需要发送的数据个数
	float Vol;		//电压变量
	Vol=(float)Voltage/100;
	DataScope_Get_Channel_Data( MotorA.Velocity, 1 );     	//显示左轮速度，单位mm/s
	DataScope_Get_Channel_Data( MotorB.Velocity, 2 );    	//显示右轮速度
	DataScope_Get_Channel_Data( Vol, 3 );               	//显示电池电压 单位：V
//	DataScope_Get_Channel_Data(0, 5 ); //用您要显示的数据替换0就行了
//	DataScope_Get_Channel_Data(0 , 6 );//用您要显示的数据替换0就行了
//	DataScope_Get_Channel_Data(0, 7 );
//	DataScope_Get_Channel_Data( 0, 8 ); 
//	DataScope_Get_Channel_Data(0, 9 );  
//	DataScope_Get_Channel_Data( 0 , 10);
	Send_Count = DataScope_Data_Generate(CHANNEL_NUMBER);//CHANNEL_NUMBER可改变通道数量，目前是4
	for( i = 0 ; i < Send_Count; i++) 
	{
		while((USART1->SR&0X40)==0);  
		USART1->DR = DataScope_OutPut_Buffer[i]; 
	}
}

/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：uint
作    者：平衡小车之家
**************************************************************************/
u16 myabs(int Input)
{
	int Output;
	if(Input > 0)
		Output = Input;
	else
		Output = -Input;
	return Output;
}

/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：uint
作    者：平衡小车之家
**************************************************************************/
float myabs_float(float Input)
{
	float Output;
	if(Input > 0)
		Output = Input;
	else
		Output = -Input;
	return Output;
}



/**************************************************************************
Function: According to the potentiometer switch needs to control the car type
Input   : none
Output  : none
函数功能：根据电位器切换需要控制的小车类型
入口参数：无
返回  值：无
**************************************************************************/
void Robot_Select(void)
{
	u8 Car_Select_Count = 0;
	u32 Car_Select_Sum = 0;				//车型选择ADC相关变量
	//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models, CAR_NUMBER=6
  //ADC值分段变量，取决于小车型号数量
	//车型选择电位计，adc每10次取一次平均
		for(Car_Select_Count=0;Car_Select_Count<2;Car_Select_Count++)
		{
			Car_Select_Sum += Get_Adc(CAR_ADC_CHANNEL);
			//delay_ms(1);
		}
		Car_Num = Car_Select_Sum/2/(Max_Car_ADC/Num_Of_Car);	//算出车型的号码
		Car_Select_Sum = 0;
	
}

