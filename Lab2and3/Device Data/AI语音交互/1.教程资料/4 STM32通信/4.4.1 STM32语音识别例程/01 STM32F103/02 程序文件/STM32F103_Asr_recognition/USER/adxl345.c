/***********************
IIC总线接口
SDA：PC1
SCL：PC9
************************/

#include "include.h"

//读取数据函数
u8 read_data(void)
{
	u8 val;
	IIC_start();  
	IIC_send_byte(0x78);
	//IIC_wait_ack();	   
	//IIC_send_byte(0x01);   		
	val = IIC_wait_ack(); 	 										  		   

	//IIC_start();  	 	   		
	//IIC_send_byte(0x78);
	//IIC_wait_ack();
	//val = IIC_read_byte(1);	     	   
	IIC_stop();
	return val;	
}
