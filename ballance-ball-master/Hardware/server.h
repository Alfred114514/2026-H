#ifndef __SERVER_H
#define __SERVER_H

#include "main.h"
#include "tim.h"

#define SERVER_HANDLE htim2
#define SERVER_CHANEL TIM_CHANNEL_1
#define SERVER_ARR 10000

void Server_Start(void);
void Server_Stop(void);
void Server_Setangle(uint8_t Angle);

#endif
