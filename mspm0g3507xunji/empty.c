#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "string.h"
#include "control.h"
#include "oled.h"
#include "JY61P.h"
#include "Delay.h"
#include "mpu_port.h"
#include "bsp_ir_eight.h"


extern float tim;
extern int timbegin;
uint8_t ir_buf[8];
extern uint8_t pid_init_flag_A;
extern uint8_t pid_init_flag_B;
extern volatile int32_t gEncoderVal_left;
extern volatile int32_t gEncoderVal_right;


float pitch = 0, roll = 0, pianhang = 0;
extern volatile uint32_t sys_tick_ms;
float pitch1 = 0, roll1 = 0, pianhang1 = 0;


// ===== 循迹状态机 =====
enum {
    STATE_IDLE         = 0,   // 等待按键
    STATE_RUN_AB       = 1,   // S1: 全速巡线到终点
    STATE_RUN_AB_CURVE = 2,   // S2: 巡线到第一个弯道停止
    STATE_RUN_AB3      = 3,   // S3: 慢速巡线到终点
    STATE_DONE         = 4
};
static uint8_t g_state = STATE_IDLE;
static float   g_final_time = 0;

// 按键消抖（SysConfig: 上拉输入 → 按下=读到0）
static uint8_t g_s1_press_cnt = 0;
static uint8_t g_s2_press_cnt = 0;
static uint8_t g_s3_press_cnt = 0;
#define PRESS_THRESHOLD  10

// 返回 1：该按键本次采样产生一次“按下”事件（已消抖）
static uint8_t Key_Pressed(GPIO_Regs *port, uint32_t pin, uint8_t *cnt)
{
    uint8_t pressed = (DL_GPIO_readPins(port, pin) == 0) ? 1 : 0;
    if (pressed) {
        if (*cnt < 255) (*cnt)++;
    } else {
        *cnt = 0;
    }
    if (*cnt >= PRESS_THRESHOLD) {
        *cnt = 0;
        return 1;
    }
    return 0;
}

// 启动指定巡线模式（解除 STBY、开始计时）
static void Arm_Run(uint8_t next_state)
{
    DL_GPIO_setPins(GPIO_STBY_PORT, GPIO_STBY_PIN_STBY_PIN);
    g_state      = next_state;
    g_curve_flag = 0;
    tim          = 0;
    timbegin     = 1;
    // OLED_Clear();
}

// 停止巡线：失能 STBY、停车、显示标题与用时
static void Stop_Run(const char *title)
{
    g_state      = STATE_DONE;
    timbegin     = 0;
    g_final_time = tim;

    DL_GPIO_clearPins(GPIO_STBY_PORT, GPIO_STBY_PIN_STBY_PIN);
    Set_Pwm(0, 0);

    char buf[32];
    OLED_Clear();
    OLED_ShowString(0, 16, (u8 *)title, 16);

    int sec = (int)g_final_time;
    int ms  = (int)((g_final_time - sec) * 100);
    sprintf(buf, "Time: %d.%02ds", sec, ms);
    OLED_ShowString(0, 36, (u8 *)buf, 16);
    OLED_Refresh();
}


void SysTick_Handler(void) {
    sys_tick_ms++;
}


int main(void)
{
    pid_init_flag_A = 0;
    pid_init_flag_B = 0;
    SYSCFG_DL_init();
    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();

    OLED_ShowString(0, 0,  (u8 *)"S1: Run",  16);
    OLED_ShowString(0, 16, (u8 *)"S2: Curve",16);
    OLED_ShowString(0, 32, (u8 *)"S3: Slow", 16);
    OLED_Refresh();

    DMP_Init();
    NVIC_EnableIRQ(TIMER_Encoder_Read_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(GPIO_EncoderA_INT_IRQN);
    NVIC_EnableIRQ(GPIO_EncoderB_INT_IRQN);
    DL_Timer_startCounter(TIMER_Encoder_Read_INST);
    DL_Timer_startCounter(TIMER_0_INST);
    DL_Timer_startCounter(PWM_0_INST);

    // 上电默认 STBY 失能
    DL_GPIO_clearPins(GPIO_STBY_PORT, GPIO_STBY_PIN_STBY_PIN);
    // Set_Pwm(0, 0);

    static uint16_t loop_cnt = 0;

    while (1) {
        char oled_str[50];

        ReadEightIR(ir_buf);

        // ===== 按键扫描（上拉输入：按下=读到0，松开=读到1）=====
        if (g_state == STATE_IDLE) {
            if (Key_Pressed(GPIO_Key_PIN_S1_PORT, GPIO_Key_PIN_S1_PIN, &g_s1_press_cnt)) {
                Arm_Run(STATE_RUN_AB);
            }
            else if (Key_Pressed(GPIO_Key_PIN_S2_PORT, GPIO_Key_PIN_S2_PIN, &g_s2_press_cnt)) {
                Arm_Run(STATE_RUN_AB_CURVE);
            }
            else if (Key_Pressed(GPIO_Key_PIN_S3_PORT, GPIO_Key_PIN_S3_PIN, &g_s3_press_cnt)) {
                Arm_Run(STATE_RUN_AB3);
            }
        }

        // ===== 状态机 =====
        if (g_state == STATE_RUN_AB) {
            Control_AB();
            // 终点：中心4路全黑
            if (tim >= 14.7f) {
                Stop_Run("Finished!");
            }
        }
        else if (g_state == STATE_RUN_AB_CURVE) {
            Control_AB2();
            // 第一次循迹到弯道 → 停止
            if (tim >= 5.0f) {
                Stop_Run("Curve!");
            }
        }
        else if (g_state == STATE_RUN_AB3) {
            Control_AB3();
            // 终点：中心4路全黑:g_state == STATE_RUN_AB3
            if (tim >= 27.0f) {
                Stop_Run("Finished!");
            }
        }

        // ===== OLED 刷新（每20轮）=====
              loop_cnt++;
        if (loop_cnt >= 20) {
            loop_cnt = 0;
             float speed_L = (float)gEncoderVal_left / 3.0f;
            float speed_R = (float)gEncoderVal_right / 3.0f;
         
            if (g_state == STATE_RUN_AB || g_state == STATE_RUN_AB_CURVE || g_state == STATE_RUN_AB3) {
                // 显示 ir_buf[0]-[7] 传感器值
                sprintf(oled_str, "0:%d 1:%d 2:%d 3:%d",
                    ir_buf[0], ir_buf[1], ir_buf[2], ir_buf[3]);
                OLED_ShowString(0, 0, (u8 *)oled_str, 12);
                sprintf(oled_str, "4:%d 5:%d 6:%d 7:%d",
                    ir_buf[4], ir_buf[5], ir_buf[6], ir_buf[7]);
                OLED_ShowString(0, 12, (u8 *)oled_str, 12);

            //     int sec = (int)tim;
            //     int ms  = (int)((tim - sec) * 100);
            //     sprintf(oled_str, "Time: %d.%02ds", sec, ms);
            //     OLED_ShowString(0, 24, (u8 *)oled_str, 16);
            sprintf(oled_str, "L_Spd:%.1f", speed_L);
            OLED_ShowString(0, 40, (u8 *)oled_str, 16);
            sprintf(oled_str, "R_Spd: %.1f", speed_R);
            OLED_ShowString(0, 56, (u8 *)oled_str, 16);
            // OLED_Refresh();   // 运行期间禁刷：整屏 100kHz I2C 逐字节写约 300ms 阻塞，会卡死控制主循环（OLED正常时车晃/甩出的根因）
            }
        
            }
        }
    }

