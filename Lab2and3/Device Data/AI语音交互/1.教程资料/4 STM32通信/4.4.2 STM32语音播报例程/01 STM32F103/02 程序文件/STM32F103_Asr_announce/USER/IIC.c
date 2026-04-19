#include "include.h"


void SDA_OUT(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;	
	GPIO_InitStructure.GPIO_Pin=IIC_IO_SDA;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure); 						
}

void SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	GPIO_InitStructure.GPIO_Pin=IIC_IO_SDA;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}


void IIC_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_1|GPIO_Pin_12);	
    IIC_SDA=1;
	IIC_SCL=1;
}


void IIC_Start(void)
{
	SDA_OUT();
	IIC_SDA=1;
	IIC_SCL=1;
	DelayUs(5);
	IIC_SDA=0;
	DelayUs(2);
	IIC_SCL=0;
}

void IIC_Stop(void)
{
	SDA_OUT();
	IIC_SCL=0;
	IIC_SDA=0;
	DelayUs(5);
	IIC_SCL=1;
	IIC_SDA=1;
	DelayUs(5);
}

void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	DelayUs(2);
	IIC_SCL=1;
	DelayUs(2);
	IIC_SCL=0;
}
	

void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	DelayUs(2);
	IIC_SCL=1;
	DelayUs(2);
	IIC_SCL=0;
}

u8 IIC_Wait_Ack(void)
{
	u8 Time=0;
	SDA_IN();
	IIC_SDA=1;
	DelayUs(2);
	IIC_SCL=1;
	DelayUs(2);
	while(READ_SDA)
	{
		Time++;
		if(Time>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;
	return 0;
}
	

void IIC_Send_Byte(u8 txd)
{
	u8 t;
	SDA_OUT();
	IIC_SCL=0;
	for(t=0;t<8;t++)
	{
		IIC_SDA=(txd&0x80)>>7;
		txd<<=1;
		DelayUs(2);
		IIC_SCL=1;
		DelayUs(2);
		IIC_SCL=0;
	//  DelayUs(2);
	}
}


u8 IIC_Read_Byte(u8 ack)
{
	u8 i,receive=0;
	SDA_IN();
	for(i=0;i<8;i++)
	{
		IIC_SCL=0;
		DelayUs(2);
		IIC_SCL=1;
		receive<<=1;
		if(READ_SDA)receive++;
		DelayUs(1);
	}
	if(!ack) IIC_NAck();
	else IIC_Ack();
	return receive;
}


void WriteOneByte(u16 addr, u8 data)
{
	IIC_Start();
	IIC_Send_Byte(addr); //发送写指令
	IIC_Wait_Ack(); //应答
	IIC_Send_Byte(0xA0); //发送添加词条指令
	IIC_Wait_Ack(); //应答	
	IIC_Send_Byte(data); //发送词条编号
	IIC_Wait_Ack(); //应答
	IIC_Stop(); //停止
//	DelayMs(10);
}

//设置播报
void Asr_Speak(u8 cmd ,u8 idNum)
{	
    int i;
	int addr = Asr_Addr;
    u8 send[2] = {0x00 , 0x00};
    if(cmd == 0xFF || cmd == 0x00)
    {
        send[0] = cmd;
        send[1] = idNum;
        IIC_Start(); //起始信号
        IIC_Send_Byte(addr<<1 | 0); //发送写指令
        IIC_Wait_Ack(); //应答
        IIC_Send_Byte(ASR_SPEAK_ADDR); //发送添加词条指令
        IIC_Wait_Ack(); //应答
        //IIC_Stop();
        for(i = 0; i < 2; ++i)
        {
            IIC_Send_Byte(send[i]);
            IIC_Wait_Ack(); //应答
        }
        IIC_Stop(); //停止
        DelayMs(20);
    }
}

//获取识别结果
int Asr_Result()
{
	int result = 0;
	IIC_Start(); //起始信号
	IIC_Send_Byte(Asr_Addr<<1 | 0); //发送写指令
	IIC_Wait_Ack(); //应答
	IIC_Send_Byte(ASR_RESULT_ADDR); //识别结果存放处，通过不断读取此地址的值判断是否识别到语音，不同的值对应不同的语音，
	if (IIC_Wait_Ack()) //应答
	{
		IIC_Stop(); //停止	
		return 0;
	}
	IIC_Stop(); //停止	
	IIC_Start(); //起始信号
	IIC_Send_Byte(Asr_Addr<<1 | 1); //发送读指令
	IIC_Wait_Ack(); //应答
	result = IIC_Read_Byte(0); //读取一个字节
	IIC_Stop(); //停止	
	DelayMs(20);
	return result;
}
