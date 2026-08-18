/**
 ******************************************************************************
 * @file    BallBalance_Control.h
 * @brief   小球平衡闭环控制模块
 *          - OpenMV 读取小球偏移坐标 (USART3)
 *          - STM32 PID 闭环计算
 *          - Emm_V5 步进电机控制 (USART1)
 *          - 参考 Sys_MainLoop() 舵机角度控制逻辑，转换为步进电机脉冲控制
 ******************************************************************************
 * @attention
 * 电机参数: 1.8° 步进电机, 16 细分 → 3200 脉冲/圈
 * 电机驱动: Emm_V5.0 闭环步进驱动器 (张大头)
 * PID 算法: 参考 System.c 的 Sys_PID_Calculate / Sys_IntegralUpdate
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
 * 电机角度限位参数 (电机不可360°旋转, 需限制机械行程)
 * ============================================================ */
/*
 * 角度限位范围: 以初始化位置为 0° 原点
 * 默认 ±30° (根据实际机械结构调至此值)
 *
 * 脉冲换算: 角度 * 3200 / 360
 *   ±30° → ±267 脉冲
 *   ±20° → ±178 脉冲
 *   ±45° → ±400 脉冲
 */
#define MOTOR_ANGLE_MIN_DEG (-90.0f) /* 最小角度 (度)                  */
#define MOTOR_ANGLE_MAX_DEG (90.0f)  /* 最大角度 (度)                  */
#define MOTOR_PULSE_MIN ((int32_t)(MOTOR_ANGLE_MIN_DEG * PULSES_PER_REV / 360.0f))
#define MOTOR_PULSE_MAX ((int32_t)(MOTOR_ANGLE_MAX_DEG * PULSES_PER_REV / 360.0f))

/* ============================================================
 * 控制参数定义 (可调)
 * ============================================================ */

/*
 * 误差缩放: 将 OpenMV 的 cm 偏移量映射到与原始 System.c 相似的数值范围
 * 原始系统: CameraMeasure 0~255, Difference ≈ ±110
 * 新系统:   ball_pos_cm ±15cm (最大)
 * 缩放比:   1cm ≈ 17 单位 → 15cm * 17 ≈ 255 (匹配原始范围)
 */
#define CM_TO_ERROR_SCALE 17.0f

/*
 * PID 输出 → 电机脉冲缩放
 * 原始系统: PID 输出 ±15 → 舵机角度偏移 ±15° (中心 30°, 范围 15°~45°)
 * 新系统:   PID 输出 ±15 → 电机脉冲, 30° 对应 3200*30/360 ≈ 267 脉冲
 *           ±15 对应 ±133 脉冲, 取 ±150 作为最大单周期脉冲
 *
 * PID_TO_PULSE_SCALE = 10.0 → ±15 * 10 = ±150 脉冲/周期 = ±16.875°/周期
 */
#define PID_TO_PULSE_SCALE 1.5f

/*
 * 控制死区: 小球偏移在此范围内不发送电机命令, 避免抖动
 * 单位: cm, 例如 0.05cm = 0.5mm
 */
#define BALL_DEADBAND_CM 0.1f

/*
 * 电机脉冲死区: PID 输出小于此值不发送电机命令, 避免微小抖动
 * 单位: 脉冲数
 */
#define MOTOR_DEADBAND_PULSES 2

/*
 * 单周期最大脉冲限制: 防止单次调整过大
 * 单位: 脉冲数, ±300 脉冲 ≈ ±33.75°
 */
#define MAX_PULSES_PER_CYCLE 45

/*
 * PID 输出限幅 (与 System.c 保持一致)
 */
#define PID_OUTPUT_MAX 15.0f
#define PID_OUTPUT_MIN -15.0f

/*
 * 积分限幅 (与 System.c 保持一致)
 */
#define PID_INTEGRAL_MAX 100.0f

/*
 * PID 默认参数 (与 System.c Sys_PID_Set() 保持一致)
 */
#define DEFAULT_KP 0.0162f
#define DEFAULT_KI 0.0095f
#define DEFAULT_KD 0.012f

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

/* PID 控制结构体 (参考 System.c 全局 PID 变量) */
typedef struct
{
    float Kp;           /* 比例增益                     */
    float Ki;           /* 积分增益                     */
    float Kd;           /* 微分增益                     */
    float Integral;     /* 积分累积值                   */
    float Differential; /* 微分值                       */
    int16_t LastError;  /* 上一次误差 (微分计算用)       */
} BallBalance_PID_t;

/* ============================================================
 * 函数声明
 * ============================================================ */

/**
 * @brief  初始化小球平衡控制系统
 *         - 设置默认 PID 参数
 *         - 配置电机快速定位模式
 *         - 初始化状态为 BALANCE_STOP
 * @retval None
 */
void BallBalance_Init(void);

/**
 * @brief  小球平衡闭环控制主函数
 *         在收到 OpenMV 新数据帧时调用 (main.c rxFrameFlag3 块内)
 *
 *         控制流程:
 *         1. 球偏移量(cm) → 缩放为误差值(整数)
 *         2. 更新 PID 积分/微分项
 *         3. PID 计算 → 输出值 (-15 ~ +15)
 *         4. PID 输出 → 电机脉冲数
 *         5. 死区判断 → Emm_V5_QPos_Control 驱动电机
 *         6. OpenMV 下一帧读取新位置 → 闭环
 *
 * @param  ball_pos_cm: OpenMV 传回的小球偏移量 (cm)
 *                     正值=偏左, 负值=偏右, 0=正中
 * @retval None
 */
void BallBalance_Control(float ball_pos_cm);

/**
 * @brief  设置目标位置
 *         默认目标为 0 (小球居中), 可设置其他目标实现偏移平衡
 * @param  target_cm: 目标偏移量 (cm), 0 = 居中
 * @retval None
 */
void BallBalance_SetTarget(float target_cm);

/**
 * @brief  启动平衡控制
 *         清除积分/微分历史, 设置状态为 BALANCE_RUNNING
 * @retval None
 */
void BallBalance_Start(void);

/**
 * @brief  停止平衡控制
 *         设置状态为 BALANCE_STOP, 不发送新的电机命令
 * @retval None
 */
void BallBalance_Stop(void);

/**
 * @brief  获取当前控制状态
 * @retval BallBalance_State_t: 当前状态
 */
BallBalance_State_t BallBalance_GetState(void);

/**
 * @brief  获取上一次 PID 计算输出 (用于 OLED 显示/调试)
 * @retval float: PID 输出值 (-15 ~ +15)
 */
float BallBalance_GetPIDOutput(void);

/**
 * @brief  获取上一次发送的电机脉冲数 (用于调试)
 * @retval int32_t: 脉冲数
 */
int32_t BallBalance_GetMotorPulses(void);

/**
 * @brief  设置 PID 参数 (运行时调参)
 * @param  Kp, Ki, Kd: PID 增益
 * @retval None
 */
void BallBalance_SetPID(float Kp, float Ki, float Kd);

/**
 * @brief  紧急停止并复位电机
 *         - 停止 PID 控制
 *         - 发送电机立即停止命令
 *         - 清除所有积分/历史数据
 * @retval None
 */
void BallBalance_EmergencyStop(void);

/**
 * @brief  获取电机当前累计角度 (以初始化位置为原点)
 * @retval float: 当前角度 (度)
 */
float BallBalance_GetMotorAngle(void);

/**
 * @brief  获取电机当前累计脉冲数 (以初始化位置为原点)
 * @retval int32_t: 累计脉冲数
 */
int32_t BallBalance_GetAccumPulses(void);

/**
 * @brief  检查电机是否触发限位
 * @retval BallBalance_Limit_t: LIMIT_OK / LIMIT_MIN / LIMIT_MAX
 */
BallBalance_Limit_t BallBalance_GetLimitState(void);

/**
 * @brief  设置电机角度限位范围 (运行时调整)
 * @param  min_deg: 最小角度 (度), 负值
 * @param  max_deg: 最大角度 (度), 正值
 * @retval None
 */
void BallBalance_SetAngleLimits(float min_deg, float max_deg);

/**
 * @brief  将电机复位到限位范围的中心位置
 *         - 如果当前角度 < 0, 发送正向脉冲回到 0°
 *         - 如果当前角度 > 0, 发送负向脉冲回到 0°
 *         - 清零累计脉冲记录
 * @retval None
 */
void BallBalance_ReturnToCenter(void);

#endif /* __BALLBALANCE_CONTROL_H */
