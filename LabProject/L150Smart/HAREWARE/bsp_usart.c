
#include "bsp_usart.h"

/**************************************************************************
Function: fputc
Input   : none
Output  : none
函数功能：重定向c库函数printf到串口
入口参数: 无 
返回  值：无
**************************************************************************/	 	
//重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
	if(!Flag_Show)			//长按按键，可以使用printf输出到串口1
	{
		/* 等待发送完毕 */
		while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET);///<等待发送完成		

		/* 发送一个字节数据到串口 */
		HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
		
		return ch;
	}
	else					//使用printf输出到串口3
	{
		
		while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET);///<等待发送完成

		HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
	
		return ch;
	}
	
}


///重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		/* 等待串口输入数据 */
	 uint8_t ch = 0;
		while (__HAL_UART_GET_FLAG (&huart1,UART_FLAG_TC) == RESET);

		return (int)HAL_UART_Receive(&huart1,&ch, 1, 0xffff);;
}

