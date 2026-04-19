#include "include.h"

void Control(void)
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
	printf("%d",val);
  //printf("\n");
	DelayMs(1000);
}