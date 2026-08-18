#include "main.h"
#include "usart.h"
#include "System.h"

uint8_t RecvDirectBuffer;
uint8_t Buffer[4];
uint8_t Cursor;
uint8_t Ack = 'A';

void MyUART_Init(void)
{
	HAL_UART_Receive_IT(&huart1, &RecvDirectBuffer, 1);
}

void MyUART_SendChar(char charx)
{
	char a = charx;
	HAL_UART_Transmit(&huart1, (uint8_t *)&a, 1, 100);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		Buffer[Cursor] = RecvDirectBuffer - '0';
		Cursor++;
		if(Cursor >= 3)
		{
			Cursor = 0;
			Sys_UART_CompleteCallBack(Buffer[0] * 100 + Buffer[1] * 10 + Buffer[2]);
		}
		HAL_UART_Receive_IT(huart, &RecvDirectBuffer, 1);
	}
}
