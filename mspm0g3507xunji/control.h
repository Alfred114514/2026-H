#ifndef	__CONTROL_H__
#define __CONTROL_H__

#include "ti_msp_dl_config.h"
#include "bsp_ir_eight.h"

extern float Pitch, Roll, Yaw;
extern uint8_t g_curve_flag;   // 弯道标志：1=当前处于弯道（Control_AB/Control_AB3 更新）


	void Control_ABCCCCC(void);
void Control_AB(void);
void Control_AB2(void);
void Control_AB3(void);
void Control_ABCDA(void);
void Control_ACBDA(void);
void Control_ACBDAx4(void);
float PID_A(float Encoder,float Target);
float PID_B(float Encoder,float Target);
void set_Duty(uint8_t duty,uint8_t channel);
float PWM_Limit(float IN,float max,float min);
void Set_Pwm(int motor_left,int motor_right);
int xunji(void);
int xunji_highspeed_left(void);
int xunji_highspeed_right(void);
int xunji_lowspeed_left(void);
int xunji_lowspeed_right(void);
int myabs(int a);
float GYRO_Control(uint8_t now,float target);
int GetTrackErr(void);
void MPU_DMP_Start(void);
void PID_Reset(void);
int xunji_full_linewalk(void);
static float IR_Walk_PID(int8_t err);
#endif
