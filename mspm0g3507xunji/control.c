#include "control.h"
#include "stdio.h"
#include "string.h"
#include <stdint.h>
uint8_t pid_init_flag_A = 0;   // PID_A 独立初始化标志
uint8_t pid_init_flag_B = 0;   // PID_B 独立初始化标志
#define AIN11	DL_GPIO_setPins(GPIO_IN_PORT,GPIO_IN_PIN_AIN1_PIN)
#define AIN10	DL_GPIO_clearPins(GPIO_IN_PORT,GPIO_IN_PIN_AIN1_PIN)
#define AIN21	DL_GPIO_setPins(GPIO_IN_PORT,GPIO_IN_PIN_AIN2_PIN)
#define AIN20	DL_GPIO_clearPins(GPIO_IN_PORT,GPIO_IN_PIN_AIN2_PIN)

#define BIN11	DL_GPIO_setPins(GPIO_IN_PORT,GPIO_IN_PIN_BIN1_PIN)
#define BIN10	DL_GPIO_clearPins(GPIO_IN_PORT,GPIO_IN_PIN_BIN1_PIN)
#define BIN21	DL_GPIO_setPins(GPIO_IN_PORT,GPIO_IN_PIN_BIN2_PIN)
#define BIN20	DL_GPIO_clearPins(GPIO_IN_PORT,GPIO_IN_PIN_BIN2_PIN)
//A为左轮，B为右轮
#define PID_OUT_MAX 200
#define PID_OUT_MIN -200
volatile uint32_t gpioA,gpioB;
volatile int32_t gEncoderCount_left = 0, gEncoderVal_left = 0;          //左轮编码器计数值；左轮编码器记录值，50ms定时中断输出
volatile int32_t gEncoderCount_right = 0, gEncoderVal_right = 0;        //右轮编码器计数值；右轮编码器记录值，50ms定时中断输出
extern float Yaw;                                                       //陀螺仪偏转角返回值

#define  Limit		100						//PWM波限幅，百分比制
  static uint8_t first_frame = 1;



extern  float pitch, roll, pianhang;
extern uint8_t ir_buf[8];
//速度环PID — 位置式 + 独立前馈（左右轮硬件不对称，分开标定）
// FF_GAIN_L: 左轮前馈 → 标定方法：Kp=Ki=0，调此值使左轮稳态速度≈目标
// FF_GAIN_R: 右轮前馈 → 同上
// Kp: 小偏差快速修正（前馈标定好后恢复）
// Ki: 消除残余静差
// Kd: 预留，可加 0.5~1.0 抑制震荡
// 当前值推算: FF=3.1时左轮26.5→需2.34, 右轮29→需2.14

#define FF_GAIN_LEFT   2.20f//2.20
#define FF_GAIN_RIGHT  2.18f//2.18

#define Kp1      0.30f//上次0.//追踪目标的速度0.2
#define Ki1      0.08f//上次0.1
#define Kd1      0.25f//上次0.52f


//循迹全局滤波gaile
#define BIAS_FILT_ALPHA  0.9f   // 滤波系数//0.7
#define LINE_GAIN        0.65f   // 循迹纠偏增益 (原 BIAS_SCALE×DIRECT_DIFF_GAIN=0.16×3.8)
#define BIAS_MAX         14.0f   // 差速上限（参考值）
//改了KD,前馈

//Control_AB（）弯道减速参数（全速）
#define SPEED_STRAIGHT    12.0f   // 直道目标速度//12
#define SPEED_CURVE       6.6f    // 弯道目标速度//6.5
#define CURVE_BIAS_THD    8       // 偏差绝对值超过此值判定为弯道
#define SPEED_FILT_ALPHA  0.89f   // 速度平滑系数（越接近1越平滑）

unsigned char ir_cnt[8] = {0};
 
// Control_AB2 慢速巡线参数（到达弯道就停下的参数）
#define AB2_STRAIGHT_SLOW  5.0f    // 慢速直道目标速度5.5
#define AB2_CURVE_SLOW     4.0f    // 慢速弯道目标速度


// Control_AB3 慢速巡线参数（比 Control_AB 全程更慢，可自行调整）
#define SPEED_STRAIGHT_SLOW  5.0f    // 慢速直道目标速度5.5
#define SPEED_CURVE_SLOW     4.0f    // 慢速弯道目标速度

// 弯道标志：Control_AB / Control_AB3 每次调用更新，1=当前处于弯道
uint8_t g_curve_flag = 0;
#define IR_VALID_CNT 2  //连续2帧黑才认定有效

  // 在你的滤波代码之前加这几行：


float CurrentA, CurrentB;			//编码器测得当前速度
float targetA=0, targetB=0;			//当前目标速度
float Speed_diff; 					//当前差速
int flag=1,n=1,whiteflag=0,whiteflag1=0,whiteflag2=0;         //白色区域计数标志位；白色区域计数值；陀螺仪模式标志位
int timebegin=0,timenum=0;          //陀螺仪模式启动延时标志位；延时计数值
int timebegin1=0,timenum1=0;          //陀螺仪模式启动延时标志位；延时计数值
int timebegin2=0,timenum2=0;          //陀螺仪模式启动延时标志位；延时计数值
int ledbegin=0,lednum=0,ledflag=0,ledflag1=0,ledflag2=0;        //声光模块延时标志位；延时计数值；声光模块启动标志位
float error=180;                                                //两个方向的偏转
int m=0;                                                        //模式切换计数值，用以判断是否跑完全程
int a=0;                                                        //AB模式停止标志位
int pwmstart=1;
float tim=0;
int timbegin=0,timnum=0;








//
void Control_AB(void)                       //模式一
{
    static float smooth_speed = SPEED_STRAIGHT; // 平滑后的目标速度
    int Motor_Left, Motor_Right;            //电机赋值
    float diff_inject;                      //直接差速注入量

    int raw_bias = xunji_full_linewalk();


  static float bias_filt;
static uint8_t bias_filt_init = 0;
if(!bias_filt_init)
{
    bias_filt = 0.0f;
    bias_filt_init = 1;
}



    // 一阶低通滤波：平滑红外信号，减少突变
    bias_filt = bias_filt * BIAS_FILT_ALPHA + raw_bias * (1.0f - BIAS_FILT_ALPHA);

    // ===== 自适应弯道减速 =====
    // 滤波后的偏差超过阈值 → 判定为弯道 → 降低目标速度
    {
        uint8_t in_curve = (myabs((int)bias_filt) > CURVE_BIAS_THD) ? 1 : 0;
        float desired_speed = in_curve ? SPEED_CURVE : SPEED_STRAIGHT;

        // 目标速度一阶低通，平滑过渡避免车身抖动
        smooth_speed = smooth_speed * SPEED_FILT_ALPHA
                     + desired_speed * (1.0f - SPEED_FILT_ALPHA);

        g_curve_flag = in_curve;   // 上报弯道状态，供状态机判断
    }
    float Speed_Middle = smooth_speed;

    // ===== 直接差速：IR偏差 → 直驱PWM =====
    diff_inject = raw_bias * LINE_GAIN;


    // PID 只追踪基础速度（不再承担差速职责），避免速度环滞后
    CurrentA = (float)gEncoderVal_left/3;  //left
    CurrentB = (float)gEncoderVal_right/3; //right

    // 左轮：PID维持基础速度 + 直接差速前馈（bias>0时加速左轮=右转）
    Motor_Left  = (int)PWM_Limit(PID_A(CurrentA, Speed_Middle) + diff_inject, Limit, -Limit);
    // 右轮：PID维持基础速度 - 直接差速前馈（bias>0时减速右轮=右转）
    Motor_Right = (int)PWM_Limit(PID_B(CurrentB, Speed_Middle) - diff_inject, Limit, -Limit);

    if(pwmstart==1) Set_Pwm(Motor_Left, Motor_Right);
    else if(pwmstart==0) Set_Pwm(1, 1);
}
	

void Control_AB2(void)
{
   static float smooth_speed = 0; // 平滑后的目标速度（慢速）SPEED_STRAIGHT_SLOW
    int Motor_Left, Motor_Right;            //电机赋值
    float diff_inject;                      //直接差速注入量

    int raw_bias = xunji_full_linewalk();


  static float bias_filt;
static uint8_t bias_filt_init = 0;
if(!bias_filt_init)
{
    bias_filt = 0.0f;
    bias_filt_init = 1;
}



    // 一阶低通滤波：平滑红外信号，减少突变
    bias_filt = bias_filt * BIAS_FILT_ALPHA + raw_bias * (1.0f - BIAS_FILT_ALPHA);

    // ===== 自适应弯道减速（慢速档） =====
    // 滤波后的偏差超过阈值 → 判定为弯道 → 降低目标速度
    {
        uint8_t in_curve = (myabs((int)bias_filt) > CURVE_BIAS_THD) ? 1 : 0;
        float desired_speed = in_curve ? AB2_CURVE_SLOW : AB2_STRAIGHT_SLOW;

        // 目标速度一阶低通，平滑过渡避免车身抖动
        smooth_speed = smooth_speed * SPEED_FILT_ALPHA
                     + desired_speed * (1.0f - SPEED_FILT_ALPHA);

        g_curve_flag = in_curve;   // 上报弯道状态，供状态机判断
    }
    float Speed_Middle = smooth_speed;

    // ===== 直接差速：IR偏差 → 直驱PWM =====
    diff_inject = raw_bias * LINE_GAIN;


    // PID 只追踪基础速度（不再承担差速职责），避免速度环滞后
    CurrentA = (float)gEncoderVal_left/3;  //left
    CurrentB = (float)gEncoderVal_right/3; //right

    // 左轮：PID维持基础速度 + 直接差速前馈（bias>0时加速左轮=右转）
    Motor_Left  = (int)PWM_Limit(PID_A(CurrentA, Speed_Middle) + diff_inject, Limit, -Limit);
    // 右轮：PID维持基础速度 - 直接差速前馈（bias>0时减速右轮=右转）
    Motor_Right = (int)PWM_Limit(PID_B(CurrentB, Speed_Middle) - diff_inject, Limit, -Limit);

    if(pwmstart==1) Set_Pwm(Motor_Left, Motor_Right);
    else if(pwmstart==0) Set_Pwm(1, 1);
}




void Control_AB3(void)
{ static float smooth_speed = 0; // 平滑后的目标速度（慢速）SPEED_STRAIGHT_SLOW
    int Motor_Left, Motor_Right;            //电机赋值
    float diff_inject;                      //直接差速注入量

    int raw_bias = xunji_full_linewalk();


  static float bias_filt;
static uint8_t bias_filt_init = 0;
if(!bias_filt_init)
{
    bias_filt = 0.0f;
    bias_filt_init = 1;
}



    // 一阶低通滤波：平滑红外信号，减少突变
    bias_filt = bias_filt * BIAS_FILT_ALPHA + raw_bias * (1.0f - BIAS_FILT_ALPHA);

    // ===== 自适应弯道减速（慢速档） =====
    // 滤波后的偏差超过阈值 → 判定为弯道 → 降低目标速度
    {
        uint8_t in_curve = (myabs((int)bias_filt) > CURVE_BIAS_THD) ? 1 : 0;
        float desired_speed = in_curve ? SPEED_CURVE_SLOW : SPEED_STRAIGHT_SLOW;

        // 目标速度一阶低通，平滑过渡避免车身抖动
        smooth_speed = smooth_speed * SPEED_FILT_ALPHA
                     + desired_speed * (1.0f - SPEED_FILT_ALPHA);

        g_curve_flag = in_curve;   // 上报弯道状态，供状态机判断
    }
    float Speed_Middle = smooth_speed;

    // ===== 直接差速：IR偏差 → 直驱PWM =====
    diff_inject = raw_bias * LINE_GAIN;


    // PID 只追踪基础速度（不再承担差速职责），避免速度环滞后
    CurrentA = (float)gEncoderVal_left/3;  //left
    CurrentB = (float)gEncoderVal_right/3; //right

    // 左轮：PID维持基础速度 + 直接差速前馈（bias>0时加速左轮=右转）
    Motor_Left  = (int)PWM_Limit(PID_A(CurrentA, Speed_Middle) + diff_inject, Limit, -Limit);
    // 右轮：PID维持基础速度 - 直接差速前馈（bias>0时减速右轮=右转）
    Motor_Right = (int)PWM_Limit(PID_B(CurrentB, Speed_Middle) - diff_inject, Limit, -Limit);

    if(pwmstart==1) Set_Pwm(Motor_Left, Motor_Right);
    else if(pwmstart==0) Set_Pwm(1, 1);
  
}

void Control_ACBDAx4(void)
{
    float Speed_Middle=50;				//中值速度
    int Motor_Left, Motor_Right;
    float bias;

    if(m==18) DL_GPIO_clearPins(GPIO_STBY_PORT,GPIO_STBY_PIN_STBY_PIN);

    if (ledflag==1) DL_GPIO_setPins(GPIO_LED_PORT,GPIO_LED_PIN_LED_PIN);
    else DL_GPIO_clearPins(GPIO_LED_PORT,GPIO_LED_PIN_LED_PIN);

    // if (P1==0 && P2==0 && P3==0 && P4==0 && P5==0 && P6==0 && P7==0 && P8==0) timebegin2=1;
    // else whiteflag2=0;

    if (whiteflag2==1) 
    {
        ledflag2=0;
        if(ledflag1==0)
        {
            ledflag1=1;
            ledbegin=1;
            m++;
        }

        if (n%2==0)
        {
            bias = Yaw + 103;
        }
        else if(n%2==1)
        {
            bias = Yaw;
        }
        flag=0;
    }
    else if(whiteflag2==0)
    {
        ledflag1=0;
        if(ledflag2==0)
        {
            ledflag2=1;
            ledbegin=1;
            m++;
        }

        if(flag==0)
        {
            flag=1;
            n=n+1;
        }
        
        if(n%2==1)
        {
            bias = xunji_lowspeed_right();
        }
        else if(n%2==0)
        {
            bias = xunji_lowspeed_left();
        }

        whiteflag2=0;
    }


    targetA = Speed_Middle+bias;
	targetB = Speed_Middle-bias;
    CurrentA = (float)gEncoderVal_left/3; //left
	CurrentB = (float)gEncoderVal_right/3; //right
	Motor_Left  = (int)PWM_Limit(PID_A(CurrentA,targetA),Limit, -Limit);
	Motor_Right = (int)PWM_Limit(PID_B(CurrentB,targetB),Limit, -Limit);		//PWM限幅
    if(m==18) 
    {
        Set_Pwm(1,1);
        timbegin=0;
    }
	else Set_Pwm(Motor_Left, Motor_Right);
}






// 左A右B
// 加权平均8路循迹，移植商家Line_Tracke逻辑，输出bias偏差值
int xunji_full_linewalk(void)
{
    static int8_t err = 0;
    // 8路传感器权重 ir_buf[0]~ir_buf[7] 对应 X1~X8
    int weights[8] = {-20, -14, -10, -4, 4, 10, 14, 20};
    int weighted_sum = 0;
    int sensor_active_count = 0;

    // ir_buf[i] != 0 代表检测到黑线（对应商家Xn==1）
    for(int i = 0; i < 8; i++)
    {
        // 跳过故障通道 ir_buf[7]（硬件常高，传感器损坏可选）（可选）
        // if (i == 7) continue;//发现硬件是正常的

        if(ir_buf[i] != 0)
        {
            weighted_sum += weights[i];
            sensor_active_count++;
        }
    }

    if (sensor_active_count > 0)
    {
        // 加权平均消除单传感器跳变影响
        err = weighted_sum / sensor_active_count;
    }
    else
    {
        // 全部无黑线，沿上次偏差方向持续搜线
        if (err > 0)
        {
            err = 30;
        }
        else if (err < 0)
        {
            err = -30;
        }
        else
        {
            err = 0;
        }
    }

    return err;
}







// int xunji_highspeed_left(void)			//输出差速，左轮速度为middle+x，右轮速度为middle-x
// {
//     if(P1!=0&&P2!=0)
//     {
//         return -110;
//     }
//     else if(P1!=0)
// 	{
// 		return -70;
// 	}
//     else if(P2!=0)
// 	{
// 		return -40;
// 	}
//     else if(P3!=0)
// 	{
// 		return -20;
// 	}
//     else if(P4!=0)
// 	{
// 		return -10;
// 	}
	
// 	return 0;
// }

// int xunji_highspeed_right(void)			//输出差速，左轮速度为middle+x，右轮速度为middle-x
// {
//     if(P8!=0&&P7!=0)
//     {
//         return 110;
//     }
//     else if(P8!=0)
// 	{
// 		return 70;
// 	}
//     else if(P7!=0)
// 	{
// 		return 40;
// 	}
//     else if(P6!=0)
// 	{
// 		return 20;
// 	}
// 	else if(P5!=0)
// 	{
// 		return 10;
// 	}
	
// 	return 0;
// }

// int xunji_lowspeed_left(void)			//输出差速，左轮速度为middle+x，右轮速度为middle-x
// {
//     if(P1!=0)
// 	{
// 		return -45;
// 	}
//     else if(P2!=0)
// 	{
// 		return -25;
// 	}
//     else if(P3!=0)
// 	{
// 		return -16;
// 	}
//     else if(P4!=0)
// 	{
// 		return -10;
// 	}
	
// 	return 0;
// }

// int xunji_lowspeed_right(void)			//输出差速，左轮速度为middle+x，右轮速度为middle-x
// {
//     if(P8!=0)
// 	{
// 		return 45;
// 	}
//     else if(P7!=0)
// 	{
// 		return 25;
// 	}
//     else if(P6!=0)
// 	{
// 		return 16;
// 	}
// 	else if(P5!=0)
// 	{
// 		return 10;
// 	}
	
// 	return 0;
// }


// ========== 位置式 PID + 前馈（左/A轮）==========
// FeedForward = Target * FF_GAIN → 预估稳态PWM
// PID 仅做 ±Δ 微调，Kp 不需很大，从根本上避免震荡
// 积分衰减（非清零）：大偏差时 Integral*=0.5，过渡更平滑
float PID_A(float Encoder, float Target)
{
    static float Bias, Integral, Last_bias, Pwm;

    if (pid_init_flag_A == 0)
    {
        Bias     = 0;
        Integral = 0;
        Last_bias = 0;
        Pwm      = 0;
        pid_init_flag_A = 1;
    }

    Bias = Target - Encoder;

    // 积分分离：小偏差积分，大偏差衰减（比清零更平滑）
    if (myabs((int)Bias) < 5)
    {
        Integral += Bias;
    }
    else
    {
        Integral *= 0.5f;   // 衰减而非清零，过渡不突变
    }

    // 积分限幅
    if (Integral >  50) Integral =  50;
    if (Integral < -50) Integral = -50;

    // 前馈 + PID 微调
    float FeedForward = Target * FF_GAIN_LEFT;
    Pwm = FeedForward
        + Kp1 * Bias
        + Ki1 * Integral
        + Kd1 * (Bias - Last_bias);

    // 输出限幅
    if (Pwm > PID_OUT_MAX) Pwm = PID_OUT_MAX;
    if (Pwm < PID_OUT_MIN) Pwm = PID_OUT_MIN;

    Last_bias = Bias;
    return Pwm;
}

// ========== 位置式 PID + 前馈（右/B轮）==========
float PID_B(float Encoder, float Target)
{
    static float Bias, Integral, Last_bias, Pwm;

    if (pid_init_flag_B == 0)
    {
        Bias     = 0;
        Integral = 0;
        Last_bias = 0;
        Pwm      = 0;
        pid_init_flag_B = 1;
    }

    Bias = Target - Encoder;

    if (myabs((int)Bias) < 5)
    {
        Integral += Bias;
    }
    else
    {
        Integral *= 0.5f;
    }

    if (Integral >  50) Integral =  50;
    if (Integral < -50) Integral = -50;

    float FeedForward = Target * FF_GAIN_RIGHT;
    Pwm = FeedForward
        + Kp1 * Bias
        + Ki1 * Integral
        + Kd1 * (Bias - Last_bias);

    if (Pwm > PID_OUT_MAX) Pwm = PID_OUT_MAX;
    if (Pwm < PID_OUT_MIN) Pwm = PID_OUT_MIN;

    Last_bias = Bias;
    return Pwm;
}

void Set_Pwm(int motor_left,int motor_right)
{
	if(motor_left > 0)	//前进
	{
        AIN10;
		AIN21;
	}
	else				//后退
	{
		AIN11;
		AIN20;
	}
	//PWMA = myabs(motor_left);
    set_Duty(myabs(motor_left),1);

	if(motor_right > 0) //前进
	{
		BIN10;
		BIN21;
	}
	else				//后退
	{
        BIN11;
		BIN20;
	}
	//PWMB = myabs(motor_right);
    set_Duty(myabs(motor_right),0);
}

void set_Duty(uint8_t duty,uint8_t channel)               //修改pwm波占空比  0~100                 //用timer不要用timerg或timera
{
    uint32_t CompareValue;
    CompareValue = 2500 - 2500/100*duty;                  //占空比转换

    if(channel == 0)
    {
        DL_Timer_setCaptureCompareValue(PWM_0_INST,CompareValue,DL_TIMER_CC_0_INDEX);           //修改占空比
    }
    else if(channel == 1)
    {
        DL_Timer_setCaptureCompareValue(PWM_0_INST,CompareValue,DL_TIMER_CC_1_INDEX);           
    }
}

void GROUP1_IRQHandler(void)                    //中断服务函数
{
    /**********************编码器读取***********************/
    gpioA = DL_GPIO_getEnabledInterruptStatus(GPIOA,GPIO_EncoderA_PIN_0_PIN | GPIO_EncoderA_PIN_1_PIN);
    gpioB = DL_GPIO_getEnabledInterruptStatus(GPIOB,GPIO_EncoderB_PIN_2_PIN | GPIO_EncoderB_PIN_3_PIN);
    if((gpioA & GPIO_EncoderA_PIN_0_PIN) == GPIO_EncoderA_PIN_0_PIN)
    {
        //Pin0上升沿
        if(!DL_GPIO_readPins(GPIOA,GPIO_EncoderA_PIN_1_PIN))//P1为高电平
        {
            gEncoderCount_left++;
        }
        else//P1为低电平
        {
            gEncoderCount_left--;
        }
    }
    else if((gpioA & GPIO_EncoderA_PIN_1_PIN) == GPIO_EncoderA_PIN_1_PIN)
    {
        //Pin1上升沿
        if(!DL_GPIO_readPins(GPIOA,GPIO_EncoderA_PIN_0_PIN))//P0为高电平
        {
            gEncoderCount_left--;
        }
        else//P1为低电平
        {
            gEncoderCount_left++;
        }
    }

    if((gpioB & GPIO_EncoderB_PIN_2_PIN) != 0)
    {
        if(!DL_GPIO_readPins(GPIOB,GPIO_EncoderB_PIN_3_PIN))
        {
            gEncoderCount_right--;
        }
        else
        {
            gEncoderCount_right++;
        }
    }
    else if((gpioB & GPIO_EncoderB_PIN_3_PIN) != 0)
    {
        if(!DL_GPIO_readPins(GPIOB,GPIO_EncoderB_PIN_2_PIN))
        {
            gEncoderCount_right++;
        }
        else
        {
            gEncoderCount_right--;
        }
    }
    DL_GPIO_clearInterruptStatus(GPIOA, GPIO_EncoderA_PIN_0_PIN|GPIO_EncoderA_PIN_1_PIN);
    DL_GPIO_clearInterruptStatus(GPIOB, GPIO_EncoderB_PIN_2_PIN|GPIO_EncoderB_PIN_3_PIN);
    /*********************************************************************************************/
}

void TIMER_Encoder_Read_INST_IRQHandler(void)                   //定时中断
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_Encoder_Read_INST)){            //定时，将编码器数据存入val
        case DL_TIMER_IIDX_ZERO:
            gEncoderVal_left = gEncoderCount_left;                              //读取左轮编码器数据
            gEncoderCount_left = 0;
            gEncoderVal_right = gEncoderCount_right;                            //读取右轮编码器数据
            gEncoderCount_right = 0;
            // printf("%d %d %d\r\n",gEncoderVal_left,gEncoderVal_right,xunji());
            if (timebegin==1)
            {
                if (timenum==2)
                {
                    whiteflag=1;
                    timebegin=0;
                    timenum=0;
                }
                timenum++;
            }

            // if (timebegin1==1)
            // {
            //     if (timenum1==1)
            //     {
            //         whiteflag1=1;
            //         timebegin1=0;
            //         timenum1=0;
            //     }
            //     timenum1++;
            // }

            if (ledbegin==1)
            {
                ledflag=1;
                if (lednum==10)
                {
                    ledflag=0;
                    ledbegin=0;
                    lednum=0;
                }
                lednum++;
            }
            break;
        default:
            break;
    }
}

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST))
    {
        case DL_TIMER_IIDX_ZERO:

            if (timebegin1==1)
            {
                if (timenum1==3)
                {
                    whiteflag1=1;
                    timebegin1=0;
                    timenum1=0;
                }
                timenum1++;
            }

            if (timebegin2==1)
            {
                if (timenum2==19)
                {
                    whiteflag2=1;
                    timebegin2=0;
                    timenum2=0;
                }
                timenum2++;
            }

            if (timbegin==1)
            {
                tim = tim + 0.01;
            }
            
            break;
        default:
            break;
    }
}

float PWM_Limit(float IN,float max,float min)                   //pwm限幅
{
	float OUT = IN;
	if(OUT > max) OUT = max;
	if(OUT < min) OUT = min;
	return OUT;
}

int myabs(int a)                                                //自定义绝对值函数
{
	int temp;
	if(a < 0)  temp = -a;
	else temp = a;
	return temp;
}


