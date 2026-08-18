#include "tim.h"

uint16_t Count1;
uint16_t Count3;
void Watch_Start(uint8_t Watchx)
{
	if(Watchx == 1)
	{
		__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		HAL_TIM_Base_Start_IT(&htim1);
		Count1 = 0;
	}
	else if(Watchx == 3)
	{
		__HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		HAL_TIM_Base_Start_IT(&htim3);
		Count3 = 0;
	}
	else if(Watchx == 4)
	{
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		HAL_TIM_Base_Start(&htim4);
	}
}

float Watch_Read(uint8_t Watchx)
{
	float RetVal = 0;
	if(Watchx == 1)
		RetVal = Count1 * 1000 + __HAL_TIM_GET_COUNTER(&htim1) / 10.0;	//ms
	else if(Watchx == 3)
		RetVal = Count3 + __HAL_TIM_GET_COUNTER(&htim3) / 10000.0;	//s
	else if(Watchx == 4)
		RetVal = __HAL_TIM_GET_COUNTER(&htim4) / 10.0;	//ms
	return RetVal;
}

void Watch_Stop(uint8_t Watchx)
{
	if(Watchx == 1)
	{
		HAL_TIM_Base_Stop_IT(&htim1);
		Count1 = 0;		
	}
	else if(Watchx == 3)
	{
		HAL_TIM_Base_Stop_IT(&htim3);
		Count3 = 0;		
	}
	else if(Watchx == 4)
	{
		HAL_TIM_Base_Stop(&htim4);
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)	//1s
	{
		Count1++;
	}
	else if(htim->Instance == TIM3)	//1s
	{
		Count3++;
	}
}
