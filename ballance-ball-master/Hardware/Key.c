#include "Key.h"

KeyNameTypeDef Key_GetKeyNum(void)
{
	KeyNameTypeDef KeyNum = None;
	static KeyNameTypeDef PresentState = None, LastState = None;
	PresentState = None;
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET)
		PresentState = SW2;
	else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET)
		PresentState = SW3;
	else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET)
		PresentState = SW4;

	if (PresentState != None && LastState == None) // 按下
	{
		Watch_Start(4);
	}
	else if (PresentState != None && LastState != None) // 按住
	{
		if (Watch_Read(4) >= 500)
		{
			if (LastState == SW2)
				KeyNum = SW2_L;
			else if (LastState == SW3)
				KeyNum = SW3_L;
			else if (LastState == SW4)
				KeyNum = SW4_L;
		}
	}
	else if (PresentState == None && LastState != None) // 抬起
	{
		if (Watch_Read(4) < 500)
			KeyNum = LastState;
		Watch_Stop(4);
	}
	LastState = PresentState;
	return KeyNum;
}
