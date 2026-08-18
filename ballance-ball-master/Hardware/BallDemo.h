/**
 ******************************************************************************
 * @file    BallDemo.h
 * @brief   小球 ±5cm 折返演示任务
 *          - 复用 BallBalance_SetTarget 实现目标切换
 *          - 状态机: 0 -> +5cm -> -5cm(稳定) -> 完成
 *          - 按键 SW4 短按启动
 ******************************************************************************
 */

#ifndef __BALLDEMO_H
#define __BALLDEMO_H

#include "main.h"
#include "stdbool.h"

/* 演示任务状态 */
typedef enum
{
    DEMO_IDLE = 0,    /* 未启动                       */
    DEMO_GO_PLUS5,    /* 0 -> +5cm (到达即折返)        */
    DEMO_GO_MINUS5,   /* +5 -> -5cm (到达并稳定)       */
    DEMO_DONE         /* -5cm 稳定完成                 */
} BallDemo_State_t;

/**
 * @brief  启动演示: 目标设为 +5cm 并开始计时
 */
void BallDemo_Start(void);

/**
 * @brief  状态机主函数, 每次收到球位置时调用
 * @param  ball_pos: 当前球偏移 (cm)
 */
void BallDemo_Tick(float ball_pos);

/**
 * @brief  获取当前演示状态
 */
BallDemo_State_t BallDemo_GetState(void);

/**
 * @brief  是否已超时 (总时长 > 5s 仍未完成)
 */
bool BallDemo_IsTimeout(void);

#endif /* __BALLDEMO_H */
