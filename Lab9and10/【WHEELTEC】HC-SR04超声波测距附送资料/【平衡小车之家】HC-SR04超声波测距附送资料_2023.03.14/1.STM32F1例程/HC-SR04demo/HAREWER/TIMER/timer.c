#include "timer.h"
#include "delay.h"
#include "usart.h"
u32 Distance;                               //超声波测距

TIM_ICInitTypeDef  ICInitStructure;
/**************************************************************************
函数功能：定时器3通道3输入捕获初始化
入口参数：入口参数：arr：自动重装值  psc：时钟预分频数 
返回  值：无
**************************************************************************/

void TIM3_Cap_Init(u16 arr,u16 psc)
{
  GPIO_InitTypeDef GPIO_InitStructure;            //定义结构体GPIO_InitStructure
	NVIC_InitTypeDef NVIC_InitStructure;            //定义结构NVIC_InitStructure
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;   //定义结构TIM_TimeBaseStructure
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE); //开启TIM3B时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//开启GPIOB时钟
	
	//GPIOB.0
  GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_0;	        //PB0 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;     	//下拉输入  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //IO口速度
	GPIO_Init(GPIOB, &GPIO_InitStructure);              //GBIOB初始化  
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);						        //PA0 下拉
	
	//GPIOB.1
	GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_1;	        //PB1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   	//推挽输出 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //IO口速度
	GPIO_Init(GPIOB, &GPIO_InitStructure);             //GBIOB初始化  
	
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM3输入捕获参数
	ICInitStructure.TIM_Channel = TIM_Channel_3; //CC3S=01 	选择输入端 IC3映射到TI3上
  ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI3上
  ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  ICInitStructure.TIM_ICFilter = 0x00;//IC3F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM3, &ICInitStructure);
	
	 //TIM3 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;         //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC3,ENABLE);//允许更新中断 ,允许CC3IE捕获中断	
	
  TIM_Cmd(TIM3,ENABLE ); 	//使能定时器3
	
}

u8  TIM3CH3_CAPTURE_STA=0;	//输入捕获状态		    				
u16	TIM3CH3_CAPTURE_VAL;	//输入捕获值

/**************************************************************************
函数功能：超声波接收回波函数
入口参数：无
返回  值：无
**************************************************************************/
/*距离distance = v*t 
音速v=340m/s=340mm/ms =0.340mm/us，1ms=1000us
距离=（时间（微秒）* 340/1000）/2
2是因为t是来回时间
*/
void Read_Distane(void)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_1);	 // 高电平	 
	 delay_us(15);  
	GPIO_ResetBits(GPIOB, GPIO_Pin_1);	 // 低电平
	if(TIM3CH3_CAPTURE_STA&0X80)//成功捕获到了一次高电平
		{
			Distance=TIM3CH3_CAPTURE_STA&0X3F;
			Distance*=65536;					        //溢出时间总和
			Distance+=TIM3CH3_CAPTURE_VAL;		//得到总的高电平时间
			printf("超声波探测来回时间=%d us\r\n",Distance);
			Distance=Distance*340/1000/2;
			TIM3CH3_CAPTURE_STA=0;			//开启下一次捕获
		}				
}

/**************************************************************************
函数功能：超声波回波脉宽读取中断
入口参数：无
返回  值：无
**************************************************************************/

void TIM3_IRQHandler(void)
{ 		    		  			    
	
	if((TIM3CH3_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{	  
		if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
		 
		{	    
			if(TIM3CH3_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((TIM3CH3_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					TIM3CH3_CAPTURE_STA|=0X80;//标记成功捕获了一次
					TIM3CH3_CAPTURE_VAL=0XFFFF;
				}else TIM3CH3_CAPTURE_STA++;     //定时器溢出TIM3CH3_CAPTURE_STA加1计数
			}	 
		}
	if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)//捕获3发生捕获事件
		{	
			if(TIM3CH3_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
			{	  			
				TIM3CH3_CAPTURE_STA|=0X80;		//标记成功捕获到一次上升沿
				TIM3CH3_CAPTURE_VAL=TIM_GetCapture3(TIM3); //获取定时器3通道3的低电平信号
		   		TIM_OC3PolarityConfig(TIM3,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			}else  								//还未开始,第一次捕获上升沿
			{
				TIM3CH3_CAPTURE_STA=0;			//清空
				TIM3CH3_CAPTURE_VAL=0;
	 			TIM_SetCounter(TIM3,0);
				TIM3CH3_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
		   		TIM_OC3PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			}		    
		}			     	    					   
 	}
 
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC3|TIM_IT_Update); //清除中断标志位  
}

