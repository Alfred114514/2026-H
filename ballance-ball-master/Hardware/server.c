#include "server.h"

// 内部函数
void Server_SetDuty(float Duty)
{
	uint16_t CCR;
	CCR = Duty * SERVER_ARR;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, CCR);
}

void Server_Start(void)
{
	HAL_TIM_PWM_Start(&SERVER_HANDLE, SERVER_CHANEL);
}

void Server_Stop(void)
{
	HAL_TIM_PWM_Stop(&SERVER_HANDLE, SERVER_CHANEL);
}

// 0~180 此项目测得最佳范围为20~60
void Server_Setangle(uint8_t Angle)
{
	Server_SetDuty(0.025 + Angle * 0.025 / 45.0);
}
