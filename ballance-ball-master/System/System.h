#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "main.h"
#include "tim.h"

void Sys_MainOnce(void);
void Sys_MainLoop(void);
void Sys_UART_CompleteCallBack(uint16_t Num);

#endif
