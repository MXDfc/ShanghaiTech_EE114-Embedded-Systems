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

#include "bluetooth.h"
#include "Lidar.h"
SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
int Time_count;
u16 receive_cnt;
u8 data_process_flag,data_flag = 0;

static const u8 CrcTable[256] =
{
 0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
 0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
 0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
 0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
 0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
 0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
 0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
 0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
 0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
 0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
 0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};//用于crc校验的数组

u8 Usart1_Receive_buf[1];          //串口1接收中断数据存放的缓冲区
u8 Usart3_Receive_buf[1];          //串口3接收中断数据存放的缓冲区
u8 Uart5_Receive_buf[1];          //串口5接收中断数据存放的缓冲区
u8 PID_Send;			//用于手机app获取参数界面的变量
u8 Flag_Direction;		//用于手机app的方向变量，顺时针一圈共8个方向，数值是1--8,停止时数值为0
#define APP_CMD_AUTO_UPPER      0x41  /* 'A': smart cruise before manual control is active */
#define APP_CMD_MANUAL_UPPER    0x42  /* 'B': APP manual before manual control is active */
#define APP_CMD_AUTO            0x61  /* 'a': smart cruise at any time */
#define APP_CMD_MANUAL          0x62  /* 'b': APP manual at any time */
#define APP_CMD_MAPPING         0x53  /* 'S': local mapping navigation */

static void APP_Enter_Smart_Cruise_Mode(void)
{
	Mode = Smart_Cruise_Mode;
	APP_ON_Flag = RC_OFF;
	PS2_ON_Flag = RC_OFF;
	Remote_ON_Flag = RC_OFF;
	ROS_ON_Flag = RC_OFF;
	Flag_Direction = 0;
	Move_X = 0;
	Move_Z = 0;
	Smart_Cruise_Reset();
}

static void APP_Enter_Mapping_Nav_Mode(void)
{
	Mode = Lidar_Mapping_Nav_Mode;
	APP_ON_Flag = RC_OFF;
	PS2_ON_Flag = RC_OFF;
	Remote_ON_Flag = RC_OFF;
	ROS_ON_Flag = RC_OFF;
	Flag_Direction = 0;
	Move_X = 0;
	Move_Z = 0;
	Lidar_Mapping_Nav_Reset();
}

static void APP_Enter_Manual_Mode(void)
{
	Mode = Normal_Mode;
	APP_ON_Flag = RC_ON;
	PS2_ON_Flag = RC_OFF;
	Remote_ON_Flag = RC_OFF;
	ROS_ON_Flag = RC_OFF;
	Flag_Direction = 0;
	Move_X = 0;
	Move_Z = 0;
	RC_Velocity = Default_Velocity;
	if(Car_Num == Akm_Car)
		RC_Turn_Velocity = Bluetooth_Turn_Angle;
	else
		RC_Turn_Velocity = Default_Turn_Bias;
}

static u8 APP_Mode_Command_Process(u8 bluetooth_receive)
{
	if(bluetooth_receive == APP_CMD_AUTO ||
		(APP_ON_Flag == RC_OFF && bluetooth_receive == APP_CMD_AUTO_UPPER))
	{
		APP_Enter_Smart_Cruise_Mode();
		return 1;
	}
	else if(bluetooth_receive == APP_CMD_MANUAL ||
		(APP_ON_Flag == RC_OFF && bluetooth_receive == APP_CMD_MANUAL_UPPER))
	{
		APP_Enter_Manual_Mode();
		return 1;
	}
	else if(bluetooth_receive == APP_CMD_MAPPING)
	{
		APP_Enter_Mapping_Nav_Mode();
		return 1;
	}
	return 0;
}
/**************************************************************************
Function: BLUETOOTH_USART_IRQHandler
Input   : none
Output  : none
函数功能：蓝牙接收中断函数，接收来自手机的数据
入口参数: 无 
返回  值：无
**************************************************************************/	 	
//蓝牙接收中断，串口3
void HAL_UART_RxCpltCallback(UART_HandleTypeDef*huart) //接收回调函数
{
	if(huart == &huart1)
	{
			static u8 Count=0;
	  u8 Usart_Receive;

		Usart_Receive = Usart1_Receive_buf[0];//Read the data //读取数据
		ROS_ON_Flag = RC_ON;//ros控制时，将小车模式设为ROS模式
		APP_ON_Flag = RC_OFF;		
		PS2_ON_Flag = RC_OFF;
		Remote_ON_Flag = RC_OFF;
//		if(Time_count<CONTROL_DELAY)
//			// Data is not processed until 10 seconds after startup
//		  //开机10秒前不处理数据
//		  return 0;	
		
		//Fill the array with serial data
		//串口数据填入数组
        Receive_Data.buffer[Count]=Usart_Receive;
		
		// Ensure that the first data in the array is FRAME_HEADER
		//确保数组第一个数据为FRAME_HEADER
		if(Usart_Receive == FRAME_HEADER||Count>0) 
			Count++; 
		else			
			Count=0;
		
		if (Count == 11) //Verify the length of the packet //验证数据包的长度
		{   
				Count=0; //Prepare for the serial port data to be refill into the array //为串口数据重新填入数组做准备
				if(Receive_Data.buffer[10] == FRAME_TAIL) //Verify the frame tail of the packet //验证数据包的帧尾
				{
					//Data exclusionary or bit check calculation, mode 0 is sent data check
					//数据异或位校验计算，模式0是发送数据校验
					if(Receive_Data.buffer[9] ==Check_Sum(9,0))	 
				  {	
						float Vz;						
						//All modes flag position 0, USART3 control mode
            //所有模式标志位置0，为Usart3控制模式						
//						PS2_ON_Flag=0;
//						Remote_ON_Flag=0;
//						APP_ON_Flag=0;
//						CAN_ON_Flag=0;
//						Usart1_ON_Flag=0;
//						Usart5_ON_Flag=0;
						//Calculate the target speed of three axis from serial data, unit m/s
						//从串口数据求三轴目标速度， 单位m/s
						Move_X=XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
//						Move_Y=XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
						Vz    =XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
						if(Car_Num==Akm_Car)
						{
							Move_Z=Vz_to_Akm_Angle(Move_X, Vz);
						}
						else
						{
							Move_Z=XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
						}			
					}
			}
		}
       HAL_UART_Receive_IT(&huart1,Usart1_Receive_buf,sizeof(Usart1_Receive_buf));//串口3回调函数执行完毕之后，需要再次开启接收中断等待下一次接收中断的发生			
	}
	if(huart == &huart3)
	{
		static int temp_count = 0;				//用于记录前进的指令的次数，第一次连接蓝牙的时候需要用到
	  static	int bluetooth_receive=0;		//蓝牙接收相关变量
	  static u8 Flag_PID,i,j,Receive[50];
	  static float Data;						//app界面接收参数用到的变量	  
		  bluetooth_receive=Usart3_Receive_buf[0]; 
		  if(APP_Mode_Command_Process((u8)bluetooth_receive))
		  {
			  temp_count = 0;
			  Flag_PID = 0;
			  i = 0;
			  j = 0;
			  Data = 0;
			  memset(Receive, 0, sizeof(u8)*50);
			  HAL_UART_Receive_IT(&huart3,Usart3_Receive_buf,sizeof(Usart3_Receive_buf));
			  return;
		  }
		  if(APP_ON_Flag == RC_OFF)						//未开启蓝牙控制，只接收数据进行简单的分析
	  	{
			  if(bluetooth_receive == 0x41)
			  {
				  if((++temp_count) == 5)					//需要连续发送5次前进的指令，上拉转盘一段时间可开始app控制
				  {
					  temp_count = 0;
					  APP_Enter_Manual_Mode();
				  }
			  }
			  else 
				  temp_count = 0;
		  }
		  else 
		  {
			  if(bluetooth_receive>=0x41&&bluetooth_receive<=0x48) 	//默认使用，8个方向手机发送的数值是0x41--0x48
				  Flag_Direction=bluetooth_receive-0x40;
			
			  else if(bluetooth_receive==0X5A)  						//停止时发送0x5a
				  Flag_Direction=0;

			  else if(bluetooth_receive<=10)  //备用app
				  Flag_Direction=bluetooth_receive;	
			
			  else if(bluetooth_receive==0x59) 						//减速按键
			  {
				  if((RC_Velocity -= X_Step)<MINI_RC_Velocity)
					  RC_Velocity = MINI_RC_Velocity;
				  if(Car_Num != Akm_Car)								//非阿克曼车转向速度可更改
				  {
				 	  if((RC_Turn_Velocity -= Z_Step)<MINI_RC_Turn_Velocity)
					  RC_Turn_Velocity = MINI_RC_Turn_Velocity;
				  }
			  }			 
			  else if(bluetooth_receive==0x58)						//加速
			  {
				  if((RC_Velocity += X_Step)>MAX_RC_Velocity)
					  RC_Velocity = MAX_RC_Velocity;
				  if(Car_Num != Akm_Car)
				  {
					  if((RC_Turn_Velocity += Z_Step)>MAX_RC_Turn_Bias)
					  	RC_Turn_Velocity = MAX_RC_Turn_Bias;
				  }
			  }			
			  else if(bluetooth_receive==0x7B) Flag_PID=1;   			//APP参数指令起始位
			  else if(bluetooth_receive==0x7D) Flag_PID=2;   			//APP参数指令停止位

			  if(Flag_PID==1)  //采集数据
			  {
				  Receive[i]=bluetooth_receive;
				  i++;
			  }
			  if(Flag_PID==2)  //分析数据
			  {
				  if(Receive[3]==0x50) 	 PID_Send=1;
				  else if(Receive[1]!=0x23) 
				  {								
					  for(j=i;j>=4;j--)
					  {
					  	Data+=(Receive[j-1]-48)*pow(10,i-j);
					  }
					  switch(Receive[1])//调参界面
					  {
						
						 case 0x30:  RC_Velocity=Data;break;
						 case 0x31:   CCD_Move_X=Data/1000;break;
						 case 0x32:   break;
						 case 0x33:   break; //预留
						  case 0x34:  break; //预留 
						  case 0x35:  break; //预留
						  case 0x36:  break; //预留
						  case 0x37:  break; //预留
						  case 0x38:  break; //预留
						  default:break;
 
					  }
				  }				 
				  Flag_PID=0;
				  i=0;
				  j=0;
				  Data=0;
				  memset(Receive, 0, sizeof(u8)*50);//数组清零
			} 
		}
   HAL_UART_Receive_IT(&huart3,Usart3_Receive_buf,sizeof(Usart3_Receive_buf));//串口3回调函数执行完毕之后，需要再次开启接收中断等待下一次接收中断的发生		
	}
	else if(huart == &huart5)
	{
		static u8 state = 0;//状态位	，指示当前数据帧的位置
	    static u8 crc = 0;	//校验和
	    static u8 cnt = 0;	//用于一帧12个点的计数
	    u8 temp_data; 
		
		temp_data=Uart5_Receive_buf[0]; 
#if LD14	
		if (state > 5)	
		{
			if(state < 42)
			{
				if(state%3 == 0)		//一帧数据中的序号为6,9.....39的数据，距离值低8位
				{
					Pack_Data.point[cnt].distance = (u16)temp_data;
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
				}
				else if(state%3 == 1)	//一帧数据中的序号为7,10.....40的数据，距离值高8位
				{
					Pack_Data.point[cnt].distance = ((u16)temp_data<<8)+Pack_Data.point[cnt].distance;
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
				}
				else					//一帧数据中的序号为8,11.....41的数据，置信度
				{
					Pack_Data.point[cnt].confidence = temp_data;
					cnt++;	
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
				}
			}
			else 
			{
				switch(state)
				{
					case 42:
						Pack_Data.end_angle = (u16)temp_data;						//结束角度低8位
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 43:
						Pack_Data.end_angle = ((u16)temp_data<<8)+Pack_Data.end_angle;//结束角度高8位
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 44:
						Pack_Data.timestamp = (u16)temp_data;						//时间戳低8位
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 45:
						Pack_Data.timestamp = ((u16)temp_data<<8)+Pack_Data.timestamp;//时间戳高8位
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 46:
						Pack_Data.crc8 = temp_data;		//雷达传来的校验和
						if(Pack_Data.crc8 == crc)		//校验正确
						{
							data_process();				//接收到一帧且校验正确可以进行数据处理
						}
						else
						{
							memset(&Pack_Data, 0, sizeof(LiDARFrameTypeDef));//清零
						}
						crc = 0;
						state = 0;
						cnt = 0;//复位
						break;
					default: break;
				}
			}
		}
		else 
		{
			switch(state)
			{
				case 0:
					if(temp_data == HEADER)									//头固定
					{
						Pack_Data.header = temp_data;
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];				//开始进行校验
					} else state = 0,crc = 0;
					break;
				case 1:
					if(temp_data == LENGTH)									//测量的点数，目前固定
					{
						Pack_Data.ver_len = temp_data;
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
					} else state = 0,crc = 0;
					break;
				case 2:
					Pack_Data.speed = (u16)temp_data;						//雷达的转速低8位，单位度每秒
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				case 3:
					Pack_Data.speed = ((u16)temp_data<<8)+Pack_Data.speed;	//雷达的转速高8位
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				case 4:
					Pack_Data.start_angle = (u16)temp_data;					//开始角度低8位，放大了100倍
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				case 5:
					Pack_Data.start_angle = ((u16)temp_data<<8)+Pack_Data.start_angle;
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				default: break;
			}
		}
#endif
#if LD14P
		if (state > 5)
		{
			if(state < 42)
			{
				if(state%3 == 0)//一帧数据中的序号为6,9.....39的数据，距离值低8位
				{
					Pack_Data.point[cnt].distance = (u16)temp_data;
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
				}
				else if(state%3 == 1)//一帧数据中的序号为7,10.....40的数据，距离值高8位
				{
					Pack_Data.point[cnt].distance = ((u16)temp_data<<8)+Pack_Data.point[cnt].distance;
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
				}
				else//一帧数据中的序号为8,11.....41的数据，置信度
				{
					Pack_Data.point[cnt].confidence = temp_data;
					cnt++;	
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
				}
			}
			else 
			{
				switch(state)
				{
					case 42:
						Pack_Data.end_angle = (u16)temp_data;//结束角度低8位
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 43:
						Pack_Data.end_angle = ((u16)temp_data<<8)+Pack_Data.end_angle;//结束角度高8位
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 44:
						Pack_Data.timestamp = (u16)temp_data;//时间戳低8位
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 45:
						Pack_Data.timestamp = ((u16)temp_data<<8)+Pack_Data.timestamp;//时间戳高8位
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
						break;
					case 46:
						Pack_Data.crc8 = temp_data;//雷达传来的校验和
						if(Pack_Data.crc8 == crc)//校验正确
						{
							data_process();//接收到一帧且校验正确可以进行数据处理
							receive_cnt++;//输出接收到正确数据的次数
							data_process_flag=1; //标志位
						}
						else
							memset(&Pack_Data,0,sizeof(Pack_Data));//清零
						crc = 0;
						state = 0;
						cnt = 0;//复位
					default: break;
				}
			}
		}
		else 
		{
			switch(state)
			{
				
				case 0:
					if(temp_data == HEADER)//头固定
					{
						Pack_Data.header = temp_data;
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];//开始进行校验
					}
					else state = 0,crc = 0;
					break;
				case 1:
					if(temp_data == LENGTH)//测量的点数，目前固定
					{
						Pack_Data.ver_len = temp_data;
						state++;
						crc = CrcTable[(crc^temp_data) & 0xff];
					}
					else state = 0,crc = 0;
					break;
				case 2:
					Pack_Data.speed = (u16)temp_data;//雷达的转速低8位，单位度每秒
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				case 3:
					Pack_Data.speed = ((u16)temp_data<<8)+Pack_Data.speed;//雷达的转速高8位
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				case 4:
					Pack_Data.start_angle = (u16)temp_data;//开始角度低8位，放大了100倍
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				case 5:
					Pack_Data.start_angle = ((u16)temp_data<<8)+Pack_Data.start_angle;
					state++;
					crc = CrcTable[(crc^temp_data) & 0xff];
					break;
				default: break;

			}
		}
#endif
	HAL_UART_Receive_IT(&huart5,Uart5_Receive_buf,sizeof(Uart5_Receive_buf));//串口5回调函数执行完毕之后，需要再次开启接收中断等待下一次接收中断的发生	
	}
}


/**************************************************************************
Function: The data sent by the serial port is assigned
Input   : none
Output  : none
函数功能：串口发送的数据进行赋值
入口参数：无
返回  值：无
**************************************************************************/
void data_transition(void)
{
	Send_Data.Sensor_Str.Frame_Header = FRAME_HEADER; //Frame_header //帧头
	Send_Data.Sensor_Str.Frame_Tail = FRAME_TAIL;     //Frame_tail //帧尾
	
	//According to different vehicle types, different kinematics algorithms were selected to carry out the forward kinematics solution, 
	//and the three-axis velocity was obtained from each wheel velocity
	//根据不同车型选择不同运动学算法进行运动学正解，从各车轮速度求出三轴速度
	switch(Car_Num)
	{	
		case Akm_Car:  
			Send_Data.Sensor_Str.X_speed = ((MotorA.Current_Encoder+MotorB.Current_Encoder)/2)*1000; 
			Send_Data.Sensor_Str.Y_speed = 0;
			Send_Data.Sensor_Str.Z_speed = ((MotorB.Current_Encoder-MotorA.Current_Encoder)/Akm_wheelspacing)*1000;
		  break; 
		
		case Diff_Car: 
			Send_Data.Sensor_Str.X_speed = ((MotorA.Current_Encoder+MotorB.Current_Encoder)/2)*1000; 
			Send_Data.Sensor_Str.Y_speed = 0;
			Send_Data.Sensor_Str.Z_speed = ((MotorB.Current_Encoder-MotorA.Current_Encoder)/Diff_wheelspacing)*1000;
			break; 
		
		case Small_Tank_Car: 
			Send_Data.Sensor_Str.X_speed = ((MotorA.Current_Encoder+MotorB.Current_Encoder)/2)*1000; 
			Send_Data.Sensor_Str.Y_speed = 0;
			Send_Data.Sensor_Str.Z_speed = ((MotorB.Current_Encoder-MotorA.Current_Encoder)/Small_Tank_wheelspacing)*1000;
			break; 
		
		case Big_Tank_Car: 
			Send_Data.Sensor_Str.X_speed = ((MotorA.Current_Encoder+MotorB.Current_Encoder)/2)*1000; 
			Send_Data.Sensor_Str.Y_speed = 0;
			Send_Data.Sensor_Str.Z_speed = ((MotorB.Current_Encoder-MotorA.Current_Encoder)/Big_Tank_wheelspacing)*1000;
			break; 
	}
	
	//The acceleration of the triaxial acceleration //加速度计三轴加速度
	Send_Data.Sensor_Str.Accelerometer.X_data= Accel_Y; //The accelerometer Y-axis is converted to the ros coordinate X axis //加速度计Y轴转换到ROS坐标X轴
	Send_Data.Sensor_Str.Accelerometer.Y_data=-Accel_X; //The accelerometer X-axis is converted to the ros coordinate y axis //加速度计X轴转换到ROS坐标Y轴
	Send_Data.Sensor_Str.Accelerometer.Z_data= Accel_Z; //The accelerometer Z-axis is converted to the ros coordinate Z axis //加速度计Z轴转换到ROS坐标Z轴
	
	//The Angle velocity of the triaxial velocity //角速度计三轴角速度
	Send_Data.Sensor_Str.Gyroscope.X_data= Gyro_Y; //The Y-axis is converted to the ros coordinate X axis //角速度计Y轴转换到ROS坐标X轴
	Send_Data.Sensor_Str.Gyroscope.Y_data=-Gyro_X; //The X-axis is converted to the ros coordinate y axis //角速度计X轴转换到ROS坐标Y轴
	if(Flag_Stop==0) 
		//If the motor control bit makes energy state, the z-axis velocity is sent normall
	  //如果电机控制位使能状态，那么正常发送Z轴角速度
		Send_Data.Sensor_Str.Gyroscope.Z_data=Gyro_Z;  
	else  
		//If the robot is static (motor control dislocation), the z-axis is 0
    //如果机器人是静止的（电机控制位失能），那么发送的Z轴角速度为0		
		Send_Data.Sensor_Str.Gyroscope.Z_data=0;        
	
	//Battery voltage (this is a thousand times larger floating point number, which will be reduced by a thousand times as well as receiving the data).
	//电池电压(这里将浮点数放大一千倍传输，相应的在接收端在接收到数据后也会缩小一千倍)
	Send_Data.Sensor_Str.Power_Voltage = Voltage*1000; 
	
	Send_Data.buffer[0]=Send_Data.Sensor_Str.Frame_Header; //Frame_heade //帧头
  Send_Data.buffer[1]=Flag_Stop; //Car software loss marker //小车软件失能标志位
	
	//The three-axis speed of / / car is split into two eight digit Numbers
	//小车三轴速度,各轴都拆分为两个8位数据再发送
	Send_Data.buffer[2]=Send_Data.Sensor_Str.X_speed >>8; 
	Send_Data.buffer[3]=Send_Data.Sensor_Str.X_speed ;    
	Send_Data.buffer[4]=Send_Data.Sensor_Str.Y_speed>>8;  
	Send_Data.buffer[5]=Send_Data.Sensor_Str.Y_speed;     
	Send_Data.buffer[6]=Send_Data.Sensor_Str.Z_speed >>8; 
	Send_Data.buffer[7]=Send_Data.Sensor_Str.Z_speed ;    
	
	//The acceleration of the triaxial axis of / / imu accelerometer is divided into two eight digit reams
	//IMU加速度计三轴加速度,各轴都拆分为两个8位数据再发送
	Send_Data.buffer[8]=Send_Data.Sensor_Str.Accelerometer.X_data>>8; 
	Send_Data.buffer[9]=Send_Data.Sensor_Str.Accelerometer.X_data;   
	Send_Data.buffer[10]=Send_Data.Sensor_Str.Accelerometer.Y_data>>8;
	Send_Data.buffer[11]=Send_Data.Sensor_Str.Accelerometer.Y_data;
	Send_Data.buffer[12]=Send_Data.Sensor_Str.Accelerometer.Z_data>>8;
	Send_Data.buffer[13]=Send_Data.Sensor_Str.Accelerometer.Z_data;
	
	//The axis of the triaxial velocity of the / /imu is divided into two eight digits
	//IMU角速度计三轴角速度,各轴都拆分为两个8位数据再发送
	Send_Data.buffer[14]=Send_Data.Sensor_Str.Gyroscope.X_data>>8;
	Send_Data.buffer[15]=Send_Data.Sensor_Str.Gyroscope.X_data;
	Send_Data.buffer[16]=Send_Data.Sensor_Str.Gyroscope.Y_data>>8;
	Send_Data.buffer[17]=Send_Data.Sensor_Str.Gyroscope.Y_data;
	Send_Data.buffer[18]=Send_Data.Sensor_Str.Gyroscope.Z_data>>8;
	Send_Data.buffer[19]=Send_Data.Sensor_Str.Gyroscope.Z_data;
	
	//Battery voltage, split into two 8 digit Numbers
	//电池电压,拆分为两个8位数据发送
	Send_Data.buffer[20]=Send_Data.Sensor_Str.Power_Voltage >>8; 
	Send_Data.buffer[21]=Send_Data.Sensor_Str.Power_Voltage; 

  //Data check digit calculation, Pattern 1 is a data check
  //数据校验位计算，模式1是发送数据校验
	Send_Data.buffer[22]=Check_Sum(22,1); 
	
	Send_Data.buffer[23]=Send_Data.Sensor_Str.Frame_Tail; //Frame_tail //帧尾
}

/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
函数功能：将上位机发过来目标前进速度Vx、目标角速度Vz，转换为阿克曼小车的右前轮转角
入口参数：目标前进速度Vx、目标角速度Vz，单位：m/s，rad/s
返回  值：阿克曼小车的右前轮转角，单位：rad
**************************************************************************/
float Vz_to_Akm_Angle(float Vx, float Vz)
{
	float R, AngleR, Min_Turn_Radius;
	//float AngleL;
	
	//Ackermann car needs to set minimum turning radius
	//If the target speed requires a turn radius less than the minimum turn radius,
	//This will greatly improve the friction force of the car, which will seriously affect the control effect
	//阿克曼小车需要设置最小转弯半径
	//如果目标速度要求的转弯半径小于最小转弯半径，
	//会导致小车运动摩擦力大大提高，严重影响控制效果
	Min_Turn_Radius=MINI_AKM_MIN_TURN_RADIUS;
	
	if(Vz!=0 && Vx!=0)
	{
		//If the target speed requires a turn radius less than the minimum turn radius
		//如果目标速度要求的转弯半径小于最小转弯半径
		if(float_abs(Vx/Vz)<=Min_Turn_Radius)
		{
			//Reduce the target angular velocity and increase the turning radius to the minimum turning radius in conjunction with the forward speed
			//降低目标角速度，配合前进速度，提高转弯半径到最小转弯半径
			if(Vz>0)
				Vz= float_abs(Vx)/(Min_Turn_Radius);
			else	
				Vz=-float_abs(Vx)/(Min_Turn_Radius);	
		}		
		R=Vx/Vz;
		//AngleL=atan(Axle_spacing/(R-0.5*Wheel_spacing));
		AngleR=atan(Akm_axlespacing/(R+0.5f*Akm_wheelspacing));
	}
	else
	{
		AngleR=0;
	}
	
	return AngleR;
}
/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
函数功能：将上位机发过来的高8位和低8位数据整合成一个short型数据后，再做单位还原换算
入口参数：高8位，低8位
返回  值：机器人X/Y/Z轴的目标速度
**************************************************************************/
float XYZ_Target_Speed_transition(u8 High,u8 Low)
{
	//Data conversion intermediate variable
	//数据转换的中间变量
	float transition; 
	
	//将高8位和低8位整合成一个16位的short型数据
	//The high 8 and low 8 bits are integrated into a 16-bit short data
	transition=((short)((High<<8)+Low)/1000.0); 
	return transition; //Unit conversion, mm/s->m/s //单位转换, mm/s->m/s	 					
}

/**************************************************************************
Function: Serial port 1 sends data
Input   : none
Output  : none
函数功能：串口1发送数据
入口参数：无
返回  值：无
**************************************************************************/
void USART1_SEND(void)
{
     u8 i = 0;	
	for(i=0; i<24; i++)
	{
		usart1_send(Send_Data.buffer[i]);
	}	 
}

/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
函数功能：串口1发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}

/**************************************************************************
Function: Calculates the check bits of data to be sent/received
Input   : Count_Number: The first few digits of a check; Mode: 0-Verify the received data, 1-Validate the sent data
Output  : Check result
函数功能：计算要发送/接收的数据校验结果
入口参数：Count_Number：校验的前几位数；Mode：0-对接收数据进行校验，1-对发送数据进行校验
返回  值：校验结果
**************************************************************************/
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode)
{
	unsigned char check_sum=0,k;
	
	//Validate the data to be sent
	//对要发送的数据进行校验
	if(Mode==1)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Send_Data.buffer[k];
	}
	
	//Verify the data received
	//对接收到的数据进行校验
	if(Mode==0)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Receive_Data.buffer[k];
	}
	return check_sum;
}


/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
函数功能：浮点型数据计算绝对值
入口参数：浮点数
返回  值：输入数的绝对值
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}
