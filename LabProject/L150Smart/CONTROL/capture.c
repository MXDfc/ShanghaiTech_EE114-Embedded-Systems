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

//这个模块默认不使用
//4路超声波模块
//或 4路航模遥控
#include "capture.h"
#include "tim.h"

//extern TIM_HandleTypeDef htim2;
//Variables related to remote control acquisition of model aircraft
//航模遥控采集相关变量
int Remoter_Ch1=1500,Remoter_Ch2=1500;
//Model aircraft remote control receiver variable
//航模遥控接收变量
int L_Remoter_Ch1=1500,L_Remoter_Ch2=1500;  

u16 Distance1,Distance2,Distance3,Distance4;	//4个超声波测距距离变量

// 定时器输入捕获用户自定义变量结构体定义
TIM_ICUserValueTypeDef Distance_TIM2_CH2_ICUserValueStructure = {0,0,0,0};//超声波模块1捕获相应的变量
TIM_ICUserValueTypeDef Distance_TIM2_CH3_ICUserValueStructure = {0,0,0,0};//超声波模块2捕获相应的变量
TIM_ICUserValueTypeDef Distance_TIM2_CH4_ICUserValueStructure = {0,0,0,0};//超声波模块3捕获相应的变量
TIM_ICUserValueTypeDef Distance_TIM1_CH4_ICUserValueStructure = {0,0,0,0};//超声波模块4捕获相应的变量


TIM_ICUserValueTypeDef PWM_TIM2_CH4_ICUserValueStructure = {0,0,0,0};//航模遥控第一路
TIM_ICUserValueTypeDef PWM_TIM2_CH3_ICUserValueStructure = {0,0,0,0};//航模遥控第二路
TIM_ICUserValueTypeDef PWM_TIM1_CH4_ICUserValueStructure = {0,0,0,0};//航模遥控第三路
TIM_ICUserValueTypeDef PWM_TIM1_CH1_ICUserValueStructure = {0,0,0,0};//航模遥控第四路

/* TIM2 init function */
void MX_TIM2_Init(u16 arr, u16 psc)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = psc;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = arr;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_4);
  /* USER CODE END TIM2_Init 2 */

}


void TIM1_Init(u16 arr,u16 psc) //超声波初始化通道四
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = arr;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = psc;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
    HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_4);//开启定时器1通道4的输入捕获中断

   __HAL_TIM_ENABLE_IT(&htim1,TIM_IT_UPDATE);//更新中断用于溢出计数
  /* USER CODE END TIM1_Init 2 */

}
//void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
//{

//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  if(tim_baseHandle->Instance==TIM2)
//  {
// /* USER CODE BEGIN TIM2_MspInit 0 */

//  /* USER CODE END TIM2_MspInit 0 */
//    /* TIM2 clock enable */
//    __HAL_RCC_TIM2_CLK_ENABLE();

//    __HAL_RCC_GPIOA_CLK_ENABLE();
//    /**TIM2 GPIO Configuration
//    PA2     ------> TIM2_CH3
//	PA3     ------> TIM2_CH4
//    */
//    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
//    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//    /* TIM2 interrupt Init */
//    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 2);
//    HAL_NVIC_EnableIRQ(TIM2_IRQn);
//  /* USER CODE BEGIN TIM2_MspInit 1 */

//  /* USER CODE END TIM2_MspInit 1 */
//  }
//}


//void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
//{

//  if(tim_baseHandle->Instance==TIM2)
//  {
//  /* USER CODE BEGIN TIM2_MspDeInit 0 */

//  /* USER CODE END TIM2_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_TIM2_CLK_DISABLE();

//    /**TIM2 GPIO Configuration
//    PA2     ------> TIM2_CH3
//	  PA2     ------> TIM2_CH4
//    */
//    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

//    /* TIM2 interrupt Deinit */
//    HAL_NVIC_DisableIRQ(TIM2_IRQn);
//  /* USER CODE BEGIN TIM2_MspDeInit 1 */

//  /* USER CODE END TIM2_MspDeInit 1 */
//  }
//}
/**************************************************************************
Function: Distance_Cap_Init
Input   : TIM_Period,TIM_Prescaler
Output  : none
函数功能：超声波初始化
入口参数: 预装载值和预分频器 
返回  值：无
**************************************************************************/	 	
void Distance_Cap_Init(u16 arr,u16 psc)
{
	MX_TIM2_Init(arr,psc);
	TIM1_Init(arr,psc);
}

/**************************************************************************
Function: PWM_Capture_Mode_Config
Input   : TIM_Period,TIM_Prescaler
Output  : none
函数功能：航模遥控捕获PWM高电平初始化
入口参数: 预装载值和预分频器 
返回  值：无
**************************************************************************/	 	
void PWM_Cap_Init(u16 arr,u16 psc) //航模的和超声波的一致，只是航模不需要输出引脚，但是在gpio一并初始化了，不需要理会
{
	MX_TIM2_Init(arr,psc);
}


/**************************************************************************
Function: Read_Distane
Input   : none
Output  : none
函数功能：获取超声波测距距离
入口参数: 无
返回  值：无
**************************************************************************/	 	
//在5ms定时中断中调用
void Read_Distane(void)        
{   
	
	static u16 cnt = 0;	 //计数，超声波触发不要太频繁，会对距离的获取有一定的影响
	u32 temp_distance;
	cnt++;
	if(cnt == 20)		 //调节触发的速度
	{
		TRIG_HIGH1;      //超声波模块1触发   
		delay_us(15);  
		TRIG_LOW1;	
		
		TRIG_HIGH2;      //超声波模块2触发   
		delay_us(15);  
		TRIG_LOW2;	

	}
	if(cnt == 40)
	{
		cnt =0;
		TRIG_HIGH3;      //超声波模块3触发   
		delay_us(15);  
		TRIG_LOW3;	
		
		TRIG_HIGH4;      //超声波模块4触发   
		delay_us(15);  
		TRIG_LOW4;	
	}

	//超声波模块1
	if(Distance_TIM2_CH2_ICUserValueStructure.Capture_FinishFlag)//成功捕获到了一次高电平
	{
		temp_distance=Distance_TIM2_CH2_ICUserValueStructure.Capture_CcrValue+Distance_TIM2_CH2_ICUserValueStructure.Capture_Period*65536; //一共计数多少次
		Distance1=temp_distance*Light_Speed/2/1000;  						//时间*声速/2（来回） 一个计数0.001ms
		Distance_TIM2_CH2_ICUserValueStructure.Capture_FinishFlag = 0;			//开启下一次捕获
	}
	//超声波模块2
	if(Distance_TIM2_CH3_ICUserValueStructure.Capture_FinishFlag)//成功捕获到了一次高电平
	{
		temp_distance=Distance_TIM2_CH3_ICUserValueStructure.Capture_CcrValue+Distance_TIM2_CH3_ICUserValueStructure.Capture_Period*65536; //一共计数多少次
		Distance2=temp_distance*Light_Speed/2/1000;  						//时间*声速/2（来回） 一个计数0.001ms
		Distance_TIM2_CH3_ICUserValueStructure.Capture_FinishFlag = 0;			//开启下一次捕获
	}
	//超声波模块3
	if(Distance_TIM2_CH4_ICUserValueStructure.Capture_FinishFlag)//成功捕获到了一次高电平
	{
		temp_distance=Distance_TIM2_CH4_ICUserValueStructure.Capture_CcrValue+Distance_TIM2_CH4_ICUserValueStructure.Capture_Period*65536; //一共计数多少次
		Distance3=temp_distance*Light_Speed/2/1000;  						//时间*声速/2（来回） 一个计数0.001ms
		Distance_TIM2_CH4_ICUserValueStructure.Capture_FinishFlag = 0;			//开启下一次捕获
	}
	//超声波模块4
	if(Distance_TIM1_CH4_ICUserValueStructure.Capture_FinishFlag)//成功捕获到了一次高电平
	{
		temp_distance=Distance_TIM1_CH4_ICUserValueStructure.Capture_CcrValue+Distance_TIM1_CH4_ICUserValueStructure.Capture_Period*65536; //一共计数多少次
		Distance4=temp_distance*Light_Speed/2/1000;  						//时间*声速/2（来回） 一个计数0.001ms
		Distance_TIM1_CH4_ICUserValueStructure.Capture_FinishFlag = 0;			//开启下一次捕获
	}

}


//使用超声波
#ifdef Distance_Capture


/**************************************************************************
Function: HAL_TIM_IC_CaptureCallback
Input   : none
Output  : none
函数功能：高电平捕获中断回调函数
入口参数: 无
返回  值：无
**************************************************************************/	 	
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{ 	
	// 当要被捕获的信号的周期大于定时器的最长定时时，定时器就会溢出，产生更新中断
	// 这个时候我们需要把这个最长的定时周期加到捕获信号的时间里面去
	if(htim == &htim2)
	{
	/*************************************通道2*******************************************/
	  if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2)
		{
			if(Distance_TIM2_CH2_ICUserValueStructure.Capture_FinishFlag == 0)//没有完成一次的时候捕获才能进去，防止溢出次数错误
	    {
				Distance_TIM2_CH2_ICUserValueStructure.Capture_Period ++;
					if( Distance_TIM2_CH2_ICUserValueStructure.Capture_StartFlag == 0)// 第一次捕获
				{
					Distance_TIM2_CH2_ICUserValueStructure.Capture_CcrValue = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_2);//第一次捕获时，把捕获值储存起来
					Distance_TIM2_CH2_ICUserValueStructure.Capture_Period =0;// 自动重装载寄存器更新标志清0
					__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_2,TIM_ICPOLARITY_FALLING);// 当第一次捕获到上升沿之后，就把捕获边沿配置为下降沿
									// 开始捕获标志位置1			
					Distance_TIM2_CH2_ICUserValueStructure.Capture_StartFlag = 1;	
				}
				else// 下降沿捕获中断,第二次捕获
				{
					// 获取捕获比较寄存器的值，这个值就是捕获到的高电平的时间的值
					Distance_TIM2_CH2_ICUserValueStructure.Capture_CcrValue = 
					HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_2)-Distance_TIM2_CH2_ICUserValueStructure.Capture_CcrValue;
					// 当第二次捕获到下降沿之后，就把捕获边沿配置为上升沿，好开启新的一轮捕获
					__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_2,TIM_ICPOLARITY_RISING);
					// 开始捕获标志清0		
					Distance_TIM2_CH2_ICUserValueStructure.Capture_StartFlag = 0;
				}
			}
		}
	/*************************************通道3*******************************************/
	 if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_3)
		{
			if(Distance_TIM2_CH3_ICUserValueStructure.Capture_FinishFlag == 0)//没有完成一次的时候捕获才能进去，防止溢出次数错误
	    {
				 Distance_TIM2_CH3_ICUserValueStructure.Capture_Period ++;
					if(Distance_TIM2_CH3_ICUserValueStructure.Capture_StartFlag==0)// 第一次捕获
				{
					Distance_TIM2_CH3_ICUserValueStructure.Capture_CcrValue = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_3);//第一次捕获时，把捕获值储存起来
					Distance_TIM2_CH3_ICUserValueStructure.Capture_Period =0;
					__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_3,TIM_ICPOLARITY_FALLING);// 当第一次捕获到上升沿之后，就把捕获边沿配置为下降沿
					Distance_TIM2_CH3_ICUserValueStructure.Capture_StartFlag = 1; // 开始捕获标志位置1		
				}
				else
				{
					// 获取捕获比较寄存器的值，这个值就是捕获到的高电平的时间的值
					Distance_TIM2_CH3_ICUserValueStructure.Capture_CcrValue=
					HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_3)-Distance_TIM2_CH3_ICUserValueStructure.Capture_CcrValue;
					__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_3,TIM_ICPOLARITY_RISING);// 当第二次捕获到下降沿之后，就把捕获边沿配置为上升沿，好开启新的一轮捕获
					Distance_TIM2_CH3_ICUserValueStructure.Capture_StartFlag = 0;// 开始捕获标志清0		
				}
			}
		}
	/*************************************通道4*******************************************/
   if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_4)
	 {
		 if(Distance_TIM2_CH4_ICUserValueStructure.Capture_FinishFlag == 0)//没有完成一次的时候捕获才能进去，防止溢出次数错误
	   {
			  Distance_TIM2_CH4_ICUserValueStructure.Capture_Period ++;
				if(Distance_TIM2_CH4_ICUserValueStructure.Capture_StartFlag==0)// 第一次捕获
				{
					Distance_TIM2_CH4_ICUserValueStructure.Capture_CcrValue = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_4);//第一次捕获时，把捕获值储存起来
					Distance_TIM2_CH4_ICUserValueStructure.Capture_Period=0;
					__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_4,TIM_ICPOLARITY_FALLING);// 当第一次捕获到上升沿之后，就把捕获边沿配置为下降沿
					Distance_TIM2_CH4_ICUserValueStructure.Capture_StartFlag = 1; // 开始捕获标志位置1		
				}
				else
				{
					// 获取捕获比较寄存器的值，这个值就是捕获到的高电平的时间的值
					Distance_TIM2_CH4_ICUserValueStructure.Capture_CcrValue=
					HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_4)-Distance_TIM2_CH4_ICUserValueStructure.Capture_CcrValue;
					__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_4,TIM_ICPOLARITY_RISING);// 当第二次捕获到下降沿之后，就把捕获边沿配置为上升沿，好开启新的一轮捕获
					Distance_TIM2_CH4_ICUserValueStructure.Capture_StartFlag = 0;// 开始捕获标志清0		
				}
			}
		
	 }
  }
	else if(htim==&htim1)
	{
		 if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_4)
	 {
		 if(Distance_TIM1_CH4_ICUserValueStructure.Capture_FinishFlag == 0)//没有完成一次的时候捕获才能进去，防止溢出次数错误
	   {
			 if(Distance_TIM1_CH4_ICUserValueStructure.Capture_StartFlag==0)// 第一次捕获
			{
        Distance_TIM1_CH4_ICUserValueStructure.Capture_CcrValue = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_4);//第一次捕获时，把捕获值储存起来
				Distance_TIM1_CH4_ICUserValueStructure.Capture_Period =0;
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_4,TIM_ICPOLARITY_FALLING);// 当第一次捕获到上升沿之后，就把捕获边沿配置为下降沿
				Distance_TIM1_CH4_ICUserValueStructure.Capture_StartFlag = 1; // 开始捕获标志位置1		
			}
			else
			{
				// 获取捕获比较寄存器的值，这个值就是捕获到的高电平的时间的值
				Distance_TIM1_CH4_ICUserValueStructure.Capture_CcrValue=
				HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_4)-Distance_TIM1_CH4_ICUserValueStructure.Capture_CcrValue;
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_4,TIM_ICPOLARITY_RISING);// 当第二次捕获到下降沿之后，就把捕获边沿配置为上升沿，好开启新的一轮捕获
				Distance_TIM1_CH4_ICUserValueStructure.Capture_StartFlag = 0;// 开始捕获标志清0		
			}
		 }
		 
	 }
	}
}
#endif


//使用航模遥控
#ifdef PWM_Capture
/**************************************************************************
Function: HAL_TIM_IC_CaptureCallback
Input   : none
Output  : none
函数功能：高电平捕获中断回调函数
入口参数: 无
返回  值：无
**************************************************************************/	
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{ 	
	static u8 ch1_filter_times=0,ch2_filter_times=0;
	//连接航模遥遥控器后，需要推下前进杆，才可以正式航模控制小车
	//After connecting the remote controller of the model aircraft, 
	//you need to push down the forward lever to officially control the car of the model aircraft
  if(Remoter_Ch2>1600&&Remote_ON_Flag==RC_OFF)
  {
		//Model aircraft remote control mark position 1, other marks position 0
		//航模遥控标志位置1，其它标志位置0
		Remote_ON_Flag=RC_ON;
	  APP_ON_Flag=RC_OFF;
		PS2_ON_Flag=RC_OFF;
	  ROS_ON_Flag=RC_OFF;
	}
	// 当要被捕获的信号的周期大于定时器的最长定时时，定时器就会溢出，产生更新中断
	// 这个时候我们需要把这个最长的定时周期加到捕获信号的时间里面去
	if(htim == &htim2)
	{
	/*************************************通道3*******************************************/
	  if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_3)
		{
			if( PWM_TIM2_CH3_ICUserValueStructure.Capture_StartFlag == 0)// 第一次捕获
			{
			  PWM_TIM2_CH3_ICUserValueStructure.Capture_CcrValue = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_3);//第一次捕获时，把捕获值储存起来
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_3,TIM_ICPOLARITY_FALLING);// 当第一次捕获到上升沿之后，就把捕获边沿配置为下降沿									
				PWM_TIM2_CH3_ICUserValueStructure.Capture_StartFlag = 1;	// 开始捕获标志位置1		
			}
			else// 下降沿捕获中断,第二次捕获
			{
				// 获取捕获比较寄存器的值，这个值就是捕获到的高电平的时间的值
				Remoter_Ch2 = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_3)-PWM_TIM2_CH3_ICUserValueStructure.Capture_CcrValue;
				if(abs(Remoter_Ch2-L_Remoter_Ch2)>500)
				{
					ch1_filter_times++;
					if(ch1_filter_times<=5) Remoter_Ch2 = L_Remoter_Ch2;
					else ch1_filter_times = 0;
				}
				else
					ch1_filter_times=0;
				L_Remoter_Ch2 = Remoter_Ch2;
			  // 当第二次捕获到下降沿之后，就把捕获边沿配置为上升沿，好开启新的一轮捕获
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_3,TIM_ICPOLARITY_RISING);
				// 开始捕获标志清0		
				PWM_TIM2_CH3_ICUserValueStructure.Capture_StartFlag = 0;
			}
		}
	/*************************************通道4*******************************************/
	 if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_4)
		{
			if(PWM_TIM2_CH4_ICUserValueStructure.Capture_StartFlag==0)// 第一次捕获
			{
        PWM_TIM2_CH4_ICUserValueStructure.Capture_CcrValue = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_4);//第一次捕获时，把捕获值储存起来
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_4,TIM_ICPOLARITY_FALLING);// 当第一次捕获到上升沿之后，就把捕获边沿配置为下降沿
				PWM_TIM2_CH4_ICUserValueStructure.Capture_StartFlag = 1; // 开始捕获标志位置1		
			}
			else
			{
				// 获取捕获比较寄存器的值，这个值就是捕获到的高电平的时间的值
				Remoter_Ch1=HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_4)-PWM_TIM2_CH4_ICUserValueStructure.Capture_CcrValue;
				if(abs(Remoter_Ch1-L_Remoter_Ch1)>500)
				{
					ch2_filter_times++;
					if(ch2_filter_times<=5) Remoter_Ch1 = L_Remoter_Ch1;
					else ch2_filter_times = 0;
				}
				else
					ch2_filter_times=0;
				L_Remoter_Ch1 = Remoter_Ch1;
				__HAL_TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_4,TIM_ICPOLARITY_RISING);// 当第二次捕获到下降沿之后，就把捕获边沿配置为上升沿，好开启新的一轮捕获
				PWM_TIM2_CH4_ICUserValueStructure.Capture_StartFlag = 0;// 开始捕获标志清0		
			}
		}
	}
}

#endif




