/**
 ******************************************************************************
 * @file    BallDemo.c
 * @brief   小球 ±5cm 折返演示任务实现
 *
 *          任务要求:
 *          - 球从中心 O 运行到 +5cm, 到达后折返
 *          - 再运行到 -5cm 并稳定在该点附近
 *          - 总时长 ≤ 5s, ±5cm 处最大误差 ≤ 1cm
 *
 *          实现方式:
 *          复用 BallBalance_SetTarget 切换目标, PID 闭环自动把球带到目标。
 *          - +5cm: 到达 (|ball-5| ≤ 1) 即折返, 不判稳
 *          - -5cm: 到达 (|ball+5| ≤ 1) 且持续 ≥ DEMO_SETTLE_MS 才算完成
 ******************************************************************************
 */

#include "BallDemo.h"
#include "BallBalance_Control.h"

#define DEMO_TOL_CM     1.0f   /* ±5cm 处允许误差                */
#define DEMO_SETTLE_MS  500    /* -5cm 稳定保持时间 (ms)          */
#define DEMO_TOTAL_MS   5000   /* 总时长要求 (ms)                 */

static BallDemo_State_t s_state     = DEMO_IDLE;
static uint32_t         s_start_ms  = 0;
static uint32_t         s_settle_ms = 0;
static bool             s_timeout   = false;

static float fabs_local(float x)
{
    return x < 0.0f ? -x : x;
}

void BallDemo_Start(void)
{
    BallBalance_SetTarget(+5.0f);       /* 从中心 O 往 +5 走 */
    s_state     = DEMO_GO_PLUS5;
    s_start_ms  = HAL_GetTick();
    s_settle_ms = 0;
    s_timeout   = false;
}

void BallDemo_Tick(float ball_pos)
{
    uint32_t now = HAL_GetTick();

    switch (s_state)
    {
    case DEMO_GO_PLUS5:
        /* 到达 +5 (4~6cm) 即折返, 不判稳 */
        if (fabs_local(ball_pos - 5.0f) <= DEMO_TOL_CM)
        {
            BallBalance_SetTarget(-5.0f);
            s_settle_ms = now;
            s_state = DEMO_GO_MINUS5;
        }
        break;

    case DEMO_GO_MINUS5:
        /* 到达 -5 (-6~-4cm) 且持续稳定一段时间才算完成 */
        if (fabs_local(ball_pos + 5.0f) <= DEMO_TOL_CM)
        {
            if (now - s_settle_ms >= DEMO_SETTLE_MS)
                s_state = DEMO_DONE;
        }
        else
        {
            s_settle_ms = now;          /* 离开区间, 重新计时 */
        }
        break;

    case DEMO_DONE:
    case DEMO_IDLE:
    default:
        break;
    }

    /* 总时长超时判定 (保留最后目标, 不停电机) */
    if (s_state != DEMO_IDLE && s_state != DEMO_DONE &&
        now - s_start_ms > DEMO_TOTAL_MS)
    {
        s_timeout = true;
    }
}

BallDemo_State_t BallDemo_GetState(void)
{
    return s_state;
}

bool BallDemo_IsTimeout(void)
{
    return s_timeout;
}
