#ifndef __IIC_H
#define __IIC_H

//*****软件模拟IIC，
//*****修改宏定义即可
//*****不同芯片时注意时钟和延时函数
#define    IIC_IO_SDA      GPIO_Pin_12  //SDA的IO口
#define    IIC_IO_SCL      GPIO_Pin_1  //SCL的IO口
#define    GPIOX           GPIOB       //GPIOx选择
#define    CLOCK		   RCC_APB2Periph_GPIOB //时钟信号
 
#define    IIC_SCL         PBout(1) //SCL
#define    IIC_SDA         PBout(12) //输出SDA
#define    READ_SDA        PBin(12)  //输入SDA

#define    Asr_Addr        0x34  
//#define    Asr_Read        0x69

#define ASR_RESULT_ADDR   0x64
#define ASR_SPEAK_ADDR    0x6E

#define ASR_CMDMAND    0x00
#define ASR_ANNOUNCER  0xFF

void I2C_SDA_OUT(void);
void I2C_SDA_IN(void);
void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
void IIC_ack(void);
void IIC_noack(void);
u8 IIC_Wait_Ack(void);
void IIC_Send_Byte(u8 txd);
u8 IIC_Read_Byte(u8 ack);

void Asr_Speak(u8 cmd ,u8 idNum);
int Asr_Result(void);
#endif
