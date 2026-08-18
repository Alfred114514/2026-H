/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "system.h"
#include "Emm_V5.h"
#include "OpenMV.h"
#include "OLED.h"
#include "BallBalance_Control.h"
#include "BallDemo.h"
#include "Key.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float pos = 0.0f, Motor_Cur_Pos = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  Sys_MainOnce();
  /* USER CODE END 2 */
  __HAL_UART_CLEAR_IDLEFLAG(&huart1);                       // 清除IDLE标志
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);              // 使能串UART1 IDLE中断
  HAL_UART_Receive_DMA(&huart1, (uint8_t *)rxCmd, CMD_LEN); // 开启DMA接收模式

  /**********************************************************
  *** USART3 - OpenMV 通信初始化
  *** 使用 DMA1_Channel3 + IDLE 中断接收球偏移数据
  **********************************************************/
  OpenMV_Init();

  /* OLED 显示初始化提示 */
  OLED_ShowString(1, 1, "OpenMV UART3");
  OLED_ShowString(2, 1, "Waiting data...");
  /* USER CODE BEGIN WHILE */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /**********************************************************
  ***	使能电机 (建立保持力矩)
  ***	- 尽早使能, 防止上电延时期间摆杆因重力下垂
  ***	- 若驱动器尚未就绪此命令可能丢失, 延时后再次使能
  **********************************************************/
  Emm_V5_En_Control(1, true, false);

  /**********************************************************
  ***	上电延时等待闭环驱动器初始化完毕
  **********************************************************/
  HAL_Delay(500);
  Emm_V5_En_Control(1, true, false);   /* 驱动器就绪后再次使能, 确保保持力矩 */

  /**********************************************************
  ***	定义当前位置为原点(水平位置)
  ***	- 上电/复位前请确保摆杆处于水平位置
  ***	- 软件以 g_AccumPulses=0 作为 "水平 = 0°" 中性角
  ***	- 取消原来的 3200 脉冲转圈测试: 它会破坏中性角参考,
  ***	  且复位时驱动器内部位置不清零, 导致每次上电中性角
  ***	  不一致 → 小球偏移到 ±5cm
  **********************************************************/
  Emm_V5_Reset_CurPos_To_Zero(1);
  HAL_Delay(100);
  /* USER CODE END 2 */

  Emm_V5_Set_QPos_Params(1, 500, 0, 0, 0);

  /**********************************************************
  *** 小球平衡闭环控制初始化
  *** - 设置 PID 参数 (Kp=0.23, Ki=0.10, Kd=0.09)
  *** - 配置电机快速定位模式
  *** - 启动闭环平衡控制
  **********************************************************/
  BallBalance_Init();
  BallBalance_Start();

  /* Infinite loop */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Sys_MainLoop();

    /* SW4 短按启动 ±5cm 演示任务 */
    KeyNameTypeDef key = Key_GetKeyNum();
    if (key == SW4)
    {
      BallDemo_Start();
    }
    /* USER CODE END WHILE */

    /**********************************************************
    *** 检查 USART3 是否收到 OpenMV 发来的完整数据帧
    *** rxFrameFlag3 在 USART3_IRQHandler 中置位
    **********************************************************/
    if (rxFrameFlag3 == true)
    {
      rxFrameFlag3 = false;

      /* 解析 OpenMV 数据帧 */
      OpenMV_ProcessFrame((uint8_t *)rxCmd3, rxCount3);

      if (OpenMV_IsDataFresh())
      {
        float ball_pos = OpenMV_GetBallPosition();

        /*
         * 小球平衡闭环控制:
         * OpenMV 读取球坐标 → STM32 PID 计算 → 步进电机驱动
         * → 传动杆运动 → 平板倾斜 → 球移动 → OpenMV 再读取 → 闭环
         *
         * 参考 Sys_MainLoop() 的舵机角度控制逻辑,
         * 将 PID 输出值转换为步进电机脉冲 (QPos 相对定位)
         */
        BallBalance_Control(ball_pos);
        BallDemo_Tick(ball_pos);

        /* 在 OLED 上显示接收到的球偏移值，验证通信成功 */
        OLED_Clear();
        OLED_ShowString(1, 1, "OpenMV Recv OK!");
        OLED_ShowString(2, 1, "Ball Pos(cm):");

        /* 显示数值（保留2位小数） */
        char disp_buf[16];
        if (ball_pos >= 0)
        {
          sprintf(disp_buf, " +%.2f", ball_pos);
        }
        else
        {
          sprintf(disp_buf, " %.2f", ball_pos);
        }
        OLED_ShowString(3, 1, disp_buf);

        /* 显示数据方向指示 */
        if (ball_pos > 0.05f)
          OLED_ShowString(4, 1, "Ball: LEFT");
        else if (ball_pos < -0.05f)
          OLED_ShowString(4, 1, "Ball: RIGHT");
        else
          OLED_ShowString(4, 1, "Ball: CENTER");

        /*
         * 可选: 通过 USART3 回传确认字符给 OpenMV
         * 取消下面注释即可启用回传功能
         */
        // uint8_t ack[] = "OK\r\n";
        // HAL_UART_Transmit(&huart3, ack, 4, 100);
      }
      else
      {
        /* 解析失败，在 OLED 上提示 */
        OLED_Clear();
        OLED_ShowString(1, 1, "OpenMV Recv ERR");
        OLED_ShowString(2, 1, "Frame parse fail");
      }
    }

    /* 原有的电机测试代码（验证 UART3 时可保留或注释） */
    // Emm_V5_QPos_Control(1, 100);
    // // HAL_Delay(2000);
    // Emm_V5_QPos_Control(1, -100);
    // HAL_Delay(2000);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
