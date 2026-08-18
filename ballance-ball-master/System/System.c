#include "System.h"
#include "server.h"
#include "OLED.h"
#include "TOF.h"
#include "UART.h"
#include "watch.h"
#include "Key.h"

#define FrameTime				30.0	//帧时间
#define DifferenceEdge	13			//误差容许范围
#define StableTimeXms		800		//稳定的时长标准

//测量模式枚举
typedef enum
{
	Camera,
	TOF
} MeasureModeTypeDef;
MeasureModeTypeDef MeasureMode;		//测量模式

//系统模式枚举
typedef enum
{
	Prepare,									//准备模式：输入目标位置
	Task,											//任务模式：执行移球任务
	Conclude,									//结算模式：任务模式执行结束后进行结算
	Test
}	SystemModeTypeDef;
SystemModeTypeDef SysMode;				//系统模式

uint16_t PresentPosition;					//当前位置
int16_t CameraMeasure;						//摄像头测得的当前位置
uint16_t TargetPosition;					//目标位置

int16_t Difference;								//误差

//PID相关
float K_p;
float K_i;
float K_d;
float IntegralVal;
float DifferentialVal;

float Conclude_Time;							//结算用时

void Sys_PID_Set(void);

//按键检测即按键逻辑
void Sys_KeyHandle(void)
{
	KeyNameTypeDef Key = Key_GetKeyNum();
	if(Key != None)
	{
		//准备模式的按键逻辑
		if((Key == SW2 || Key == SW2_L) && SysMode == Prepare)
		{
			if(TargetPosition < 180)
				TargetPosition += 1;
		}
		else if((Key == SW3 || Key == SW3_L) && SysMode == Prepare)
		{
			if(TargetPosition > 20)
				TargetPosition -= 1;
		}
		else if(Key == SW4 && SysMode == Prepare)	//确认键
		{
			SysMode = Task;
			IntegralVal = 0;
			if(TargetPosition >= 110)
			Server_Setangle(40);
			OLED_Clear();
			OLED_ShowString(2, 3, "Loading...");
			HAL_Delay(700);
			OLED_Clear();
			Watch_Start(3);
			//开始摄像头测距
			MyUART_SendChar('R');
			OLED_Clear();
		}
				
		//结算模式的按键逻辑
		if(SysMode == Conclude)
		{
			SysMode = Prepare;
			//进入准备模式停止测距
			MyUART_SendChar('S');
			Server_Setangle(10);
			OLED_Clear();
		}
	}
}

//输入当前的误差和积分存储地址，更新积分
void Sys_IntegralUpdate(int16_t PresentDif, float *IntegralVar)
{
	*IntegralVar += PresentDif * 50.0 * 0.001;
	if(*IntegralVar >= (double)100)
		*IntegralVar = 100.0;
	else if(*IntegralVar <= (double)-100)
		*IntegralVar = -100.0;

}

//输入当前的误差和微分存储地址，更新微分
void Sys_DifferentialUpdate(int16_t PresentDif, float* DifferentialVar)
{
	static int16_t LastDif = 0;
	*DifferentialVar = (PresentDif - LastDif) / 50.0 * 1000;
	LastDif = PresentDif;
}

//输入误差、积分变量存储、微分变量存储、Kp、Ki、Kd，返回计算得的下一次输入
float Sys_PID_Calculate(int16_t DifferenceVar, float IntegralVar, float DifferentialVar,
												float Kp, float Ki, float Kd)
{
	float	Value = Kp * DifferenceVar + Ki * IntegralVar + Kd * DifferentialVar;
	if(Value > 15)
		Value = 15;
	else if(Value < -15)
		Value = -15;
	return Value;
}

//TOF校准：像左倾斜40度持续两秒，使球滚动到最长处，测量10次滤波的结果来校准
void Sys_TOF_Correct(void)
{
	Server_Setangle(40);
	HAL_Delay(2000);
	TOF_SetOffset(50);
}

////摄像头校准
//void Sys_Camera_Correct(void)
//{
//	Server_Setangle(10);
//	HAL_Delay(2000);
//	MyUART_SendChar('R');
//	HAL_Delay(500);
//	Camera_Offset = -CameraMeasure;
//	MyUART_SendChar('S');
//}

//输入误差，判断是否达到停止条件，即计时误差是否能在小范围内维持一定时间
uint8_t Sys_CompleteJudgement(int16_t DistanceVar)
{
	uint8_t RetVal = 0;
	static uint8_t Is_Watch_Running = 0;	//用于标识当前时钟是否正在计时
	if((DistanceVar < DifferenceEdge && DistanceVar > 0) || 
		(DistanceVar > -DifferenceEdge && DistanceVar < 0))	//如果误差在范围内就计时
	{
		if(Is_Watch_Running == 0)	//如果还没有开始计时，那么就开始计时
		{
			Watch_Start(1);
			Is_Watch_Running = 1;
		}
		else	//如果已经开始了计时，那么就读取当前时间，并且判断是否大于StableTimeXms
		{
			if(Watch_Read(1) >= StableTimeXms)
			{
				RetVal = 1;	//如果大于了，那么就返回1表示达成了停止条件
				Watch_Stop(1);
				Is_Watch_Running = 0;
			}
		}
	}
	else if(Is_Watch_Running == 1)	//误差不在范围内，如过此时还在计时就停止
	{
		Watch_Stop(1);	//内部自动清零计时
		Is_Watch_Running = 0;
	}
	return RetVal;
}
//测量函数，读取传感器当前的测量值，计算并更新当前位置和误差变量
void Sys_Measure(void)
{
	//测量，有两个测量来源
	if(MeasureMode == TOF)	//TOF测量
	{
		PresentPosition = TOF_Measure(1, 100);
//		if(PresentPosition < 140)
//			MeasureMode = Camera;
	}
	else										//摄像头测量
	{
		PresentPosition = CameraMeasure;
//		if(PresentPosition > 160)
//			MeasureMode = TOF;
	}
	
	//计算误差
	Difference = TargetPosition - PresentPosition;
}
//执行把球稳定到目标位置的任务。即计算当前轮PID，启动舵机
void Sys_Task(void)
{
	//初始化PID结果值
	int16_t Val = 0;
	
	//读取当前已用时
	if(SysMode == Task)
		Conclude_Time = Watch_Read(3);
	
	//判断是否达到停止条件
	if(SysMode == Task && Sys_CompleteJudgement(Difference) == 1)
	{
		SysMode = Conclude;
		Watch_Stop(3);
	}
	//计算并更新积分、微分值变量
	Sys_IntegralUpdate(Difference, &IntegralVal);
	Sys_DifferentialUpdate(Difference, &DifferentialVal);

	//计算PID值——Val
	Val = Sys_PID_Calculate(Difference, IntegralVal, DifferentialVal,
																			K_p, K_i, K_d);

	//Val输入系统
	Server_Setangle(Val + 30);
}

void Sys_PID_Set(void)
{
	K_p = 0.23;
	K_i = 0.10;
	K_d = 0.09;
//	K_p = 0.26;
//	K_i = 0.12;
//	K_d = 0.17;
//	K_p = 0.3;
//	K_i = 0.15;
//	K_d = 0.2;
//	K_p = 0.22;
//	K_i = 0.26;
//	K_d = 0.25;
//	K_p = 0.3;
//	K_i = 0.25;
//	K_d = 0.2;

}

//在主函数中只执行一次
void Sys_MainOnce(void)
{
//	//TOF初始化
//	TOF_Init();
	//OLED初始化
	OLED_Init();
	//串口初始化
	MyUART_Init();
	//舵机设为0，开始舵机
	Server_Setangle(10);
	Server_Start();
	
	//初始化测量模式为Camera
	MeasureMode = Camera;

	//初始化系统模式为准备模式
	SysMode = Prepare;
	//设定Kp，Ki，Kd的值
	Sys_PID_Set();
	//初始化积分值
	IntegralVal = 0;
	
	//初始化目标位置
	TargetPosition = 110;

//	//TOF校准
//	OLED_ShowString(2, 3, "Correcting...");
//	Sys_TOF_Correct();
//	OLED_Clear();
	
	//摄像头校准
//	OLED_ShowString(2, 3, "Correcting...");
//	Sys_Camera_Correct();
//	OLED_Clear();
}

//主循环
void Sys_MainLoop(void)
{
	//按键检测与处理
	Sys_KeyHandle();

	//测量，更新误差变量
	Sys_Measure();
	
	//准备模式：选择目标位置
	if(SysMode == Prepare)
	{
		OLED_ShowString(1, 3, "PID BALL");
		OLED_ShowString(2, 1, "Input Target:");
		OLED_ShowNum(3, 6, TargetPosition, 3);
		OLED_ShowString(4, 1, "(2:+|3:-|4:C)");
	}
	
	//任务模式：
	else if(SysMode == Task)
	{
		Sys_Task();
		
		//OELD显示任务模式信息
		OLED_ShowString(1, 1, "PID_Working...");

		OLED_ShowString(2, 1, "Position:");
		OLED_ShowNum(2, 11, PresentPosition, 3);
		
		OLED_ShowString(3, 1, "T:");
		OLED_ShowNum(3, 4, TargetPosition, 3);
		
		OLED_ShowString(3, 8, "d:");
		OLED_ShowSignedNum(3, 11, Difference, 3);
		
		OLED_ShowString(4, 1, "Time:");
		OLED_ShowNum(4, 6, Conclude_Time, 3);
		OLED_ShowString(4, 9, ".");
		OLED_ShowNum(4, 10, (int16_t)(Conclude_Time * 1000) % 10000, 3);
		
//		OLED_ShowNum(1, 1, PresentPosition, 3);
//		OLED_ShowNum(1, 10, TargetPosition, 3);
//		OLED_ShowSignedNum(1, 5, Difference, 3);
//		
//		OLED_ShowSignedNum(2, 1, K_p * Difference, 3);
//		OLED_ShowSignedNum(2, 6, K_i * IntegralVal, 3);
//		OLED_ShowSignedNum(2, 11, K_d * DifferentialVal, 3);

//		OLED_ShowString(3, 1, "T:");
//		OLED_ShowNum(3, 3, Conclude_Time, 3);
//		OLED_ShowString(3, 6, ".");
//		OLED_ShowNum(3, 7, (int16_t)(Conclude_Time * 1000) % 10000, 3);
//		
//		OLED_ShowNum(4, 1, Watch_Read(1), 5);

		if(SysMode == Conclude)	//如果在执行任务之后成为了结算模式，需要进行清屏
			OLED_Clear();
		//延迟FrameTime ms
		HAL_Delay(FrameTime);
	}

	//结算模式
	else if(SysMode == Conclude)
	{
		Sys_Task();
		
		OLED_ShowString(2, 1, "Success!");
		
		OLED_ShowString(3, 1, " Time:");
		OLED_ShowNum(3, 7, Conclude_Time, 3);
		OLED_ShowString(3, 10, ".");
		OLED_ShowNum(3, 11, (int16_t)(Conclude_Time * 1000) % 10000, 3);
		OLED_ShowString(3, 14, "s");
		HAL_Delay(FrameTime);	
	}
	
//	else if(SysMode == Test)
//	{
//		OLED_ShowSignedNum(1, 1, PresentPosition, 3);
//		OLED_ShowNum(1, 11, TargetPosition, 3);
//		OLED_ShowSignedNum(1, 6, Difference, 3);
//		
//		OLED_ShowSignedNum(2, 1, K_p * Difference, 3);
//		OLED_ShowSignedNum(2, 6, K_i * IntegralVal, 3);
//		OLED_ShowSignedNum(2, 11, K_d * DifferentialVal, 3);

//		OLED_ShowString(3, 1, "T:");
//		OLED_ShowNum(3, 3, Conclude_Time, 3);
//		OLED_ShowString(3, 6, ".");
//		OLED_ShowNum(3, 7, (int16_t)(Conclude_Time * 1000) % 10000, 3);
//		
//		OLED_ShowNum(4, 1, Watch_Read(1), 5);
//	}
	

}

//串口接收回调函数，读取摄像头测距结果
void Sys_UART_CompleteCallBack(uint16_t Num)
{
	CameraMeasure = Num;
}
