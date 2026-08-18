/**
 ******************************************************************************
 * @file    BallBalance_Control.c
 * @brief   小球平衡闭环控制模块实现
 *
 *          控制架构:
 *          ┌──────────┐    USART3     ┌──────────┐    PID      ┌──────────┐
 *          │  OpenMV  │───#±xx.xx$──→│  STM32   │───→ 角度 ──→│  Emm_V5  │
 *          │  摄像头  │              │  主控    │   (位置式)  │  步进电机 │
 *          └──────────┘              └──────────┘             └─────┬────┘
 *               ↑                                                    │
 *               │                    ┌──────────┐                    │
 *               └──── 小球位置反馈 ───│  小球/平板 │←── 传动杆运动 ───┘
 *                                    └──────────┘
 *
 *          核心思想 (球-梁系统):
 *          平板倾角 θ 决定球加速度 a = g·sinθ ≈ g·θ,
 *          球位置是 θ 的二重积分。位置式 PID 直接输出目标角度 θ,
 *          电机(闭环步进, 自带位置环)把平板转到 θ, 即退化为标准的
 *          "θ ∝ (Kp·e + Ki·∫e + Kd·ė)" 二重积分系统, 易稳定。
 *
 *          电机: 1.8°步进电机, Emm_V5.0 闭环驱动, 16细分 (3200脉冲/圈)
 ******************************************************************************
 */

#include "BallBalance_Control.h"
#include "Emm_V5.h"
#include <string.h>

/* ============================================================
 * 全局变量
 * ============================================================ */
static BallBalance_PID_t g_PID;                     /* PID 控制结构体              */
static BallBalance_State_t g_State;                 /* 当前控制状态                */
static float g_TargetPos_cm = 0.0f;                 /* 用户目标球位置 (cm)         */
static float g_Setpoint_cm = 0.0f;                  /* 斜坡后的实际 setpoint (cm)  */
static float g_LastPIDOutput = 0.0f;                /* 最近一次目标角度 (度)       */
static int32_t g_LastMotorPulses = 0;               /* 最近一次发送的增量脉冲      */
static int32_t g_AccumPulses = 0;                   /* 累计电机脉冲 (当前角度)     */
static int32_t g_PulseLimitMin = MOTOR_PULSE_MIN;   /* 脉冲下限 (运行时可变)       */
static int32_t g_PulseLimitMax = MOTOR_PULSE_MAX;   /* 脉冲上限 (运行时可变)       */
static BallBalance_Limit_t g_LimitState = LIMIT_OK; /* 当前限位状态                */

/* 斜坡状态 */
static float    g_RampStart_cm = 0.0f;   /* 斜坡起点 (当前球位置)         */
static float    g_RampTarget_cm = 0.0f;  /* 斜坡终点 (用于检测目标变化)   */
static uint32_t g_RampStart_ms = 0;      /* 斜坡起始时刻                  */

/* 速度前馈系数 (可选) */
static float g_Kff  = DEFAULT_KFF;
static float g_Kff2 = DEFAULT_KFF2;

/* ============================================================
 * 内部工具函数
 * ============================================================ */

/**
 * @brief  浮点绝对值 (避免引入 math.h)
 */
static float fabs_local(float x)
{
    return x < 0.0f ? -x : x;
}

/**
 * @brief  位置式 PID 更新 (时间归一化)
 *
 *         输出 = 目标角度 (度), 输入 error = setpoint - 球位置 (cm)
 *
 *         P = Kp · error
 *         I = Ki · ∫error dt        (积分分离 + 限幅)
 *         D = Kd · d(error)/dt      (时间归一化微分)
 *
 * @param  pid:   PID 结构体
 * @param  error: 当前误差 (cm)
 * @param  dt_s:  距上次控制的时间 (s)
 * @retval float: PID 输出 (度)
 */
static float PID_Update(BallBalance_PID_t *pid, float error, float dt_s)
{
    /* P 项 */
    float P = pid->Kp * error;

    /* I 项 (积分分离: 大误差不积分, 避免饱和) */
    if (fabs_local(error) < PID_INTEGRAL_SEP)
    {
        pid->Integral += error * dt_s;
        if (pid->Integral >  PID_INTEGRAL_MAX) pid->Integral =  PID_INTEGRAL_MAX;
        if (pid->Integral < -PID_INTEGRAL_MAX) pid->Integral = -PID_INTEGRAL_MAX;
    }
    float I = pid->Ki * pid->Integral;

    /* D 项 (时间归一化微分, 抑制速度) */
    float D = pid->Kd * (error - pid->LastError) / dt_s;
    pid->LastError = error;

    return P + I + D;
}

/**
 * @brief  对增量脉冲进行机械角度限位钳位
 *
 *         new_accum = g_AccumPulses + delta, 确保不超过 [min, max]。
 *         同时更新 g_LimitState。
 *
 * @param  delta: 期望的增量脉冲 (正=CW, 负=CCW)
 * @retval int32_t: 限位钳位后的实际增量脉冲
 */
static int32_t ClampPulsesToLimit(int32_t delta)
{
    int32_t new_accum = g_AccumPulses + delta;

    /* 上限位 */
    if (new_accum > g_PulseLimitMax)
    {
        if (g_AccumPulses >= g_PulseLimitMax)
        {
            g_LimitState = LIMIT_MAX;
            return 0;
        }
        g_LimitState = LIMIT_MAX;
        return (g_PulseLimitMax - g_AccumPulses);
    }

    /* 下限位 */
    if (new_accum < g_PulseLimitMin)
    {
        if (g_AccumPulses <= g_PulseLimitMin)
        {
            g_LimitState = LIMIT_MIN;
            return 0;
        }
        g_LimitState = LIMIT_MIN;
        return (g_PulseLimitMin - g_AccumPulses);
    }

    g_LimitState = LIMIT_OK;
    return delta;
}

/* ============================================================
 * 外部接口函数
 * ============================================================ */

/**
 * @brief  初始化小球平衡控制系统
 */
void BallBalance_Init(void)
{
    /* 设置默认 PID 参数 */
    g_PID.Kp = DEFAULT_KP;
    g_PID.Ki = DEFAULT_KI;
    g_PID.Kd = DEFAULT_KD;
    g_PID.Integral = 0.0f;
    g_PID.LastError = 0.0f;
    g_PID.LastBallPos = 0.0f;
    g_PID.LastVelocity = 0.0f;
    g_PID.LastTimeMs = 0;

    /* 初始化状态 */
    g_State = BALANCE_STOP;
    g_TargetPos_cm = 0.0f;
    g_Setpoint_cm = 0.0f;
    g_LastPIDOutput = 0.0f;
    g_LastMotorPulses = 0;
    g_AccumPulses = 0;
    g_PulseLimitMin = MOTOR_PULSE_MIN;
    g_PulseLimitMax = MOTOR_PULSE_MAX;
    g_LimitState = LIMIT_OK;

    /* 斜坡状态 */
    g_RampStart_cm  = 0.0f;
    g_RampTarget_cm = 0.0f;
    g_RampStart_ms  = 0;

    g_Kff  = DEFAULT_KFF;
    g_Kff2 = DEFAULT_KFF2;

    /*
     * 配置电机快速定位模式
     * 速度 500 RPM, 不使用加减速, 相对定位模式
     * 与 main.c 初始化时的配置保持一致
     */
    Emm_V5_Set_QPos_Params(MOTOR_ADDR, 500, 0, 0, 0);
}

/**
 * @brief  小球平衡闭环控制主函数
 *
 *         每次 OpenMV 传来新数据时调用 (帧率 16~18fps, 时间归一化自适应):
 *         1. setpoint 斜坡 (可选)
 *         2. error = setpoint - 球位置, 死区清零
 *         3. 位置式 PID → pid_out (度)
 *         4. 目标角度 = 中性角 + 方向·pid_out, 限幅 ±30°
 *         5. 目标脉冲 - 累计脉冲 = 增量
 *         6. 机械限位钳位 + 死区 → Emm_V5_QPos_Control 走增量
 */
void BallBalance_Control(float ball_pos_cm)
{
    /* 仅在运行状态下执行 PID 控制 */
    if (g_State != BALANCE_RUNNING)
        return;

    uint32_t now = HAL_GetTick();

    /* ---- 第1步: 目标变化检测 → 启动斜坡 & 清 PID 历史 ---- */
    if (g_TargetPos_cm != g_RampTarget_cm)
    {
        g_RampTarget_cm = g_TargetPos_cm;
        g_RampStart_cm  = ball_pos_cm;      /* 从当前球位置平滑过渡 */
        g_RampStart_ms  = now;
    }

    /* ---- 第2步: 生成 setpoint (斜坡 or 直接阶跃) ---- */
#if BALANCE_ENABLE_RAMP
    {
        uint32_t elapsed = now - g_RampStart_ms;
        float t = (float)elapsed / (float)BALANCE_RAMP_MS;
        if (t > 1.0f) t = 1.0f;
        g_Setpoint_cm = g_RampStart_cm + t * (g_RampTarget_cm - g_RampStart_cm);
    }
#else
    g_Setpoint_cm = g_TargetPos_cm;
#endif

    /* ---- 第3步: 误差 (cm) ---- */
    float error = g_Setpoint_cm - ball_pos_cm;

    /* ---- 第4步: 时间差 dt (s), 首次/异常做钳位 ---- */
    uint32_t dt_ms;
    if (g_PID.LastTimeMs == 0)
    {
        /* 首帧无历史: 用典型帧周期并让微分项为 0, 避免微分尖峰 */
        dt_ms = 60;
        g_PID.LastError = error;
    }
    else
    {
        dt_ms = now - g_PID.LastTimeMs;
        if (dt_ms > 200) dt_ms = 200;
        if (dt_ms < 5)   dt_ms = 5;
    }
    g_PID.LastTimeMs = now;
    float dt_s = (float)dt_ms / 1000.0f;

    /* ---- 第5步: 球位置死区 (相对 setpoint) ---- */
    /* 死区内误差清零: 平板回水平, 积分自然保持以补偿机械不对称 */
    if (fabs_local(error) <= BALL_DEADBAND_CM)
        error = 0.0f;

    /* ---- 第6步: 位置式 PID ---- */
    float pid_out = PID_Update(&g_PID, error, dt_s);

    /* ---- 第7步: 速度前馈 (可选) ---- */
#if BALANCE_ENABLE_FF
    {
        float v = (ball_pos_cm - g_PID.LastBallPos) / dt_s;
        g_PID.LastVelocity += 0.3f * (v - g_PID.LastVelocity);   /* 一阶低通 */
        pid_out += g_Kff  * g_PID.LastVelocity
                 + g_Kff2 * g_PID.LastVelocity * fabs_local(g_PID.LastVelocity);
    }
#endif
    g_PID.LastBallPos = ball_pos_cm;

    /* ---- 第8步: 方向 + 目标角度限幅 ---- */
    pid_out *= BALANCE_DIRECTION_SIGN;
    float target_angle = BALANCE_NEUTRAL_ANGLE_DEG + pid_out;
    if (target_angle >  BALANCE_MAX_ANGLE_DEG) target_angle =  BALANCE_MAX_ANGLE_DEG;
    if (target_angle < -BALANCE_MAX_ANGLE_DEG) target_angle = -BALANCE_MAX_ANGLE_DEG;
    g_LastPIDOutput = target_angle;

    /* ---- 第9步: 角度 → 目标脉冲 → 增量 ---- */
    int32_t target_pulses = (int32_t)(target_angle / DEG_PER_PULSE);
    int32_t delta = target_pulses - g_AccumPulses;

    /* ---- 第10步: 机械限位钳位 ---- */
    delta = ClampPulsesToLimit(delta);

    /* ---- 第11步: 脉冲死区 ---- */
    int32_t abs_delta = delta < 0 ? -delta : delta;
    if (abs_delta <= MOTOR_DEADBAND_PULSES)
    {
        g_LastMotorPulses = 0;
        return;
    }

    /* ---- 第12步: 发送增量定位命令 ---- */
    /*
     * Emm_V5_QPos_Control: 相对运动, delta > 0 正转(CW), < 0 反转(CCW)。
     * 由于 delta = 目标角度脉冲 - 当前累计脉冲, 电机被驱动到目标绝对角度,
     * 而 Emm_V5 内部位置环负责精确到位与保持。
     */
    Emm_V5_QPos_Control(MOTOR_ADDR, delta);

    g_LastMotorPulses = delta;
    g_AccumPulses += delta;
}

/**
 * @brief  设置目标位置 (cm)
 */
void BallBalance_SetTarget(float target_cm)
{
    g_TargetPos_cm = target_cm;
    /* 不清积分: 积分承载中性角补偿(δ), 与目标无关, 应跨目标保持 */
}

/**
 * @brief  启动平衡控制
 */
void BallBalance_Start(void)
{
    g_PID.Integral = 0.0f;
    g_PID.LastError = 0.0f;
    g_PID.LastBallPos = 0.0f;
    g_PID.LastVelocity = 0.0f;
    g_PID.LastTimeMs = 0;      /* 让下一次 Control 重新计算 dt */
    g_LastPIDOutput = 0.0f;
    g_LastMotorPulses = 0;

    g_State = BALANCE_RUNNING;
}

/**
 * @brief  停止平衡控制
 */
void BallBalance_Stop(void)
{
    g_State = BALANCE_STOP;
}

/**
 * @brief  获取当前控制状态
 */
BallBalance_State_t BallBalance_GetState(void)
{
    return g_State;
}

/**
 * @brief  获取最近一次目标角度 (度), 调试/显示用
 */
float BallBalance_GetPIDOutput(void)
{
    return g_LastPIDOutput;
}

/**
 * @brief  获取最近一次发送的增量脉冲数 (调试用)
 */
int32_t BallBalance_GetMotorPulses(void)
{
    return g_LastMotorPulses;
}

/**
 * @brief  运行时设置 PID 参数
 */
void BallBalance_SetPID(float Kp, float Ki, float Kd)
{
    g_PID.Kp = Kp;
    g_PID.Ki = Ki;
    g_PID.Kd = Kd;

    /* 参数变更后清除积分, 避免旧参数累积的积分与新参数不匹配 */
    g_PID.Integral = 0.0f;
    g_PID.LastError = 0.0f;
}

/**
 * @brief  紧急停止: 停止控制 + 立即停止电机
 */
void BallBalance_EmergencyStop(void)
{
    g_State = BALANCE_STOP;

    g_PID.Integral = 0.0f;
    g_PID.LastError = 0.0f;
    g_PID.LastBallPos = 0.0f;
    g_PID.LastVelocity = 0.0f;
    g_LastPIDOutput = 0.0f;
    g_LastMotorPulses = 0;

    Emm_V5_Stop_Now(MOTOR_ADDR, 0);
}

/**
 * @brief  获取电机当前累计角度 (以初始化位置为 0° 原点, 度)
 */
float BallBalance_GetMotorAngle(void)
{
    return (float)g_AccumPulses * 360.0f / (float)PULSES_PER_REV;
}

/**
 * @brief  获取电机当前累计脉冲数 (以初始化位置为原点)
 */
int32_t BallBalance_GetAccumPulses(void)
{
    return g_AccumPulses;
}

/**
 * @brief  检查电机是否触发限位
 */
BallBalance_Limit_t BallBalance_GetLimitState(void)
{
    return g_LimitState;
}

/**
 * @brief  运行时设置电机角度限位范围
 */
void BallBalance_SetAngleLimits(float min_deg, float max_deg)
{
    g_PulseLimitMin = (int32_t)(min_deg * (float)PULSES_PER_REV / 360.0f);
    g_PulseLimitMax = (int32_t)(max_deg * (float)PULSES_PER_REV / 360.0f);

    if (g_PulseLimitMin > g_PulseLimitMax)
    {
        int32_t tmp = g_PulseLimitMin;
        g_PulseLimitMin = g_PulseLimitMax;
        g_PulseLimitMax = tmp;
    }

    if (g_AccumPulses >= g_PulseLimitMax)
        g_LimitState = LIMIT_MAX;
    else if (g_AccumPulses <= g_PulseLimitMin)
        g_LimitState = LIMIT_MIN;
    else
        g_LimitState = LIMIT_OK;
}

/**
 * @brief  将电机复位到限位范围的中心位置 (0° 原点)
 *
 *         阻塞式, 仅在停止状态下调用。
 */
void BallBalance_ReturnToCenter(void)
{
    if (g_AccumPulses == 0)
        return;

    BallBalance_State_t prev_state = g_State;
    g_State = BALANCE_STOP;

    Emm_V5_QPos_Control(MOTOR_ADDR, -g_AccumPulses);

    int32_t abs_pulses = g_AccumPulses;
    if (abs_pulses < 0)
        abs_pulses = -abs_pulses;
    uint32_t delay_ms = (uint32_t)((float)abs_pulses / 26667.0f * 1000.0f) + 200;
    HAL_Delay(delay_ms);

    g_AccumPulses = 0;
    g_LastMotorPulses = 0;
    g_PID.Integral = 0.0f;
    g_PID.LastError = 0.0f;
    g_LimitState = LIMIT_OK;

    g_State = prev_state;
}
