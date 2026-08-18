/**
 ******************************************************************************
 * @file    BallBalance_Control.h
 * @brief   小球平衡闭环控制模块
 *          - OpenMV 读取小球偏移坐标 (USART3)
 *          - STM32 位置式 PID 计算 → 目标平板角度 (度)
 *          - Emm_V5 步进电机绝对角度闭环 (USART1)
 *
 *          控制架构 (参考 2026-H-balance 级联控制):
 *          球位置(cm) → 位置式PID → 目标角度(°) → 增量脉冲 → Emm_V5 绝对定位
 *
 *          关键改动 (相对旧版):
 *          - 旧: PID 输出 → 每周期相对脉冲 (平板角度 = 脉冲的积分, 三重积分难稳定)
 *          - 新: PID 输出 = 目标角度(°), 电机直接走到该绝对角度
 *                 (平板角度 ∝ PID 输出, 退化为标准球-梁二重积分, 更容易稳定)
 ******************************************************************************
 * @attention
 * 电机参数: 1.8° 步进电机, 16 细分 → 3200 脉冲/圈
 * 电机驱动: Emm_V5.0 闭环步进驱动器 (张大头)
 * PID 算法: 位置式 + 积分分离 + 时间归一化微分 (帧率 16~18fps 自适应)
 ******************************************************************************
 */

#ifndef __BALLBALANCE_CONTROL_H
#define __BALLBALANCE_CONTROL_H

#include "main.h"
#include "stdbool.h"

/* ============================================================
 * 电机参数定义
 * ============================================================ */
#define MOTOR_ADDR 1          /* 电机地址 (Emm_V5)               */
#define PULSES_PER_REV 3200   /* 3200 脉冲/圈 (16细分)           */
#define DEG_PER_PULSE 0.1125f /* 360° / 3200                     */

/* ============================================================
 * 电机角度限位参数 (机械行程硬保护)
 * ============================================================ */
/*
 * 角度限位范围: 以初始化位置为 0° 原点
 * 默认 ±90° (机械硬限位), 控制软限位见下方 BALANCE_MAX_ANGLE_DEG
 *
 * 脉冲换算: 角度 * 3200 / 360
 *   ±90° → ±800 脉冲
 *   ±30° → ±267 脉冲
 */
#define MOTOR_ANGLE_MIN_DEG (-90.0f) /* 最小角度 (度)                  */
#define MOTOR_ANGLE_MAX_DEG (90.0f)  /* 最大角度 (度)                  */
#define MOTOR_PULSE_MIN ((int32_t)(MOTOR_ANGLE_MIN_DEG * PULSES_PER_REV / 360.0f))
#define MOTOR_PULSE_MAX ((int32_t)(MOTOR_ANGLE_MAX_DEG * PULSES_PER_REV / 360.0f))

/* ============================================================
 * 控制参数定义 (可调)
 * ============================================================ */

/*
 * 目标平板角度限幅 (控制软限位, 单位: 度)
 * 球-梁系统角度不需要太大, ±30° 足够; 过大反而容易翻越/饱和振荡
 */
#define BALANCE_MAX_ANGLE_DEG      30.0f
#define BALANCE_NEUTRAL_ANGLE_DEG  0.0f   /* 水平位置对应角度(°) */

/*
 * PID 默认参数 —— 位置式, 输出 = 目标角度(°), 误差单位 = cm
 *
 * 增益含义:
 *   Kp : deg/cm      —— 1cm 误差产生多少度倾角 (≈ 参考 0.4 deg/mm)
 *   Ki : deg/(cm·s)  —— 时间归一化积分, 消除稳态静差 (从 0 起调)
 *   Kd : deg/(cm/s)  —— 时间归一化微分, 阻尼项 (≈ 参考 D 项等效)
 *
 * 起始值换算自 2026-H-balance (Kp=0.4 deg/mm, Kd=5.0@20ms, kff=-0.325):
 *   Kp = 0.4 * 10 = 4.0 deg/cm
 *   Kd = 5.0 * 0.02 * 10 = 1.0 deg/(cm/s)
 *
 * 调参顺序 (参考工程调试指南):
 *   1) Ki=0, 只调 Kp: 从 2.0 逐步加到球能到目标但略有超调 (约 4~6)
 *   2) 加 Kd 抑制超调/震荡: 0.5 → 2.0, 过大 (D 饱和) 会变成等幅振荡
 *   3) 最后加 Ki 消除静差: 0.5 → 2.0, 每次 +0.5
 *   4) 若球加速远离中心(方向反了), 把 BALANCE_DIRECTION_SIGN 取反
 */
#define DEFAULT_KP  6.0f    /* deg/cm       */
#define DEFAULT_KI  1.2f    /* deg/(cm·s)   */
#define DEFAULT_KD  1.7f    /* deg/(cm/s)   */

/*
 * 控制方向符号: 决定 "误差为正时" 电机朝哪个方向倾斜
 * 若首次上电球越调越偏(加速远离中心), 改成 -1.0f
 */
#define BALANCE_DIRECTION_SIGN 1.0f

/*
 * 积分分离 + 限幅
 * - |error| > PID_INTEGRAL_SEP 时不积分 (避免大误差下积分饱和)
 * - 积分累加限幅 PID_INTEGRAL_MAX (cm·s)
 */
#define PID_INTEGRAL_SEP    15.0f   /* 积分分离阈值 (cm) — 调大=基本不分离 */
#define PID_INTEGRAL_MAX    50.0f   /* 积分累加限幅 (cm·s) — 足够补偿约30°中性角偏差 */

/*
 * 球位置死区: |error| ≤ 此值时误差清零, 平板回水平, 积分保持
 * (相对目标位置的死区, 目标=±5cm 时同样生效)
 */
#define BALL_DEADBAND_CM 0.1f

/*
 * 电机脉冲死区: 增量脉冲小于此值不发送命令, 避免微小抖动
 */
#define MOTOR_DEADBAND_PULSES 1

/*
 * setpoint 斜坡 (可选): 目标切换时从当前球位置线性过渡到目标,
 * 避免阶跃误差瞬间把平板打到 30° 导致过冲。总时长 5s 内仍充裕。
 */
#define BALANCE_ENABLE_RAMP 1
#define BALANCE_RAMP_MS     800     /* 斜坡时长 (ms)                 */

/*
 * 速度前馈 (可选): 用相机位置微分 + 低通滤波估算速度, 加强阻尼。
 * 参考工程依赖相机速度 (kff=-0.325 deg/(mm/s)) 取得良好阻尼;
 * 这里默认关闭, 用 Kd 项提供阻尼, 若阻尼不足再开启。
 */
#define BALANCE_ENABLE_FF 0
#define DEFAULT_KFF   0.0f   /* deg/(cm/s) 线性前馈                 */
#define DEFAULT_KFF2  0.0f   /* deg/(cm/s)^2 二次前馈(大速度阻尼)   */

/* ============================================================
 * 类型定义
 * ============================================================ */

/* 平衡控制状态枚举 */
typedef enum
{
    BALANCE_STOP = 0,    /* 停止控制                     */
    BALANCE_RUNNING = 1, /* 运行中, 闭环控制激活          */
    BALANCE_HOLD = 2     /* 保持位置, 仅维持不做PID调整   */
} BallBalance_State_t;

/* 电机限位状态枚举 */
typedef enum
{
    LIMIT_OK = 0,  /* 未触发限位, 正常范围内        */
    LIMIT_MIN = 1, /* 触发下限位 (负方向极限)       */
    LIMIT_MAX = 2  /* 触发上限位 (正方向极限)       */
} BallBalance_Limit_t;

/* PID 控制结构体 (位置式, 时间归一化) */
typedef struct
{
    float Kp;             /* 比例增益 deg/cm                       */
    float Ki;             /* 积分增益 deg/(cm·s)                   */
    float Kd;             /* 微分增益 deg/(cm/s)                   */
    float Integral;       /* 积分累加值 (cm·s)                     */
    float LastError;      /* 上一次误差 (cm), 微分计算用            */
    float LastBallPos;    /* 上一次球位置 (cm), 速度估算用          */
    float LastVelocity;   /* 滤波后速度 (cm/s), 前馈用              */
    uint32_t LastTimeMs;  /* 上一次控制时刻 (ms), dt 计算用         */
} BallBalance_PID_t;

/* ============================================================
 * 函数声明 (接口保持不变, 供 main.c / BallDemo.c 调用)
 * ============================================================ */

/**
 * @brief  初始化小球平衡控制系统
 *         - 设置默认 PID 参数
 *         - 配置电机快速定位模式
 *         - 初始化状态为 BALANCE_STOP
 */
void BallBalance_Init(void);

/**
 * @brief  小球平衡闭环控制主函数
 *         在收到 OpenMV 新数据帧时调用 (main.c rxFrameFlag3 块内)
 *
 *         控制流程:
 *         1. 目标斜坡 (可选) → 当前 setpoint (cm)
 *         2. error = setpoint - 球位置 (cm), 死区清零
 *         3. 位置式 PID (时间归一化 I/D) → pid_out (度)
 *         4. 目标角度 = 中性角 + pid_out, 限幅 ±30°
 *         5. 角度 → 目标脉冲 → 增量 = 目标 - 累计
 *         6. 机械限位钳位 + 脉冲死区 → Emm_V5_QPos_Control 走增量
 *
 * @param  ball_pos_cm: OpenMV 传回的小球偏移量 (cm)
 *                     正值=偏左, 负值=偏右, 0=正中
 */
void BallBalance_Control(float ball_pos_cm);

/**
 * @brief  设置目标位置 (cm), 0 = 居中
 */
void BallBalance_SetTarget(float target_cm);

/**
 * @brief  启动平衡控制 (清除 PID 历史, 状态 = BALANCE_RUNNING)
 */
void BallBalance_Start(void);

/**
 * @brief  停止平衡控制 (状态 = BALANCE_STOP)
 */
void BallBalance_Stop(void);

/**
 * @brief  获取当前控制状态
 */
BallBalance_State_t BallBalance_GetState(void);

/**
 * @brief  获取最近一次 PID 计算的目标角度 (度), 调试/显示用
 */
float BallBalance_GetPIDOutput(void);

/**
 * @brief  获取最近一次发送的电机增量脉冲数 (调试用)
 */
int32_t BallBalance_GetMotorPulses(void);

/**
 * @brief  运行时设置 PID 参数
 */
void BallBalance_SetPID(float Kp, float Ki, float Kd);

/**
 * @brief  紧急停止: 停止控制 + 立即停止电机 + 清历史
 */
void BallBalance_EmergencyStop(void);

/**
 * @brief  获取电机当前累计角度 (以初始化位置为原点, 度)
 */
float BallBalance_GetMotorAngle(void);

/**
 * @brief  获取电机当前累计脉冲数 (以初始化位置为原点)
 */
int32_t BallBalance_GetAccumPulses(void);

/**
 * @brief  检查电机是否触发限位
 */
BallBalance_Limit_t BallBalance_GetLimitState(void);

/**
 * @brief  运行时设置电机角度限位范围
 */
void BallBalance_SetAngleLimits(float min_deg, float max_deg);

/**
 * @brief  将电机复位到限位范围的中心位置 (0° 原点)
 */
void BallBalance_ReturnToCenter(void);

#endif /* __BALLBALANCE_CONTROL_H */
