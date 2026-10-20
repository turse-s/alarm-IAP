/**
  ******************************************************************************
  * @file    main.c
  * @brief   IAP Bootloader 主程序
  *          启动流程：
  *            1. 初始化系统时钟(48MHz)与 USART1(115200)
  *            2. App 有效时，打开 3 秒监听窗口，等待升级握手命令
  *               （99 ?? ?? C1 ?? ?? 00 C4）；收到则进入 Ymodem 升级
  *            3. 监听超时且 App 有效 → 直接跳转 App
  *            4. App 无效（空片）→ 自动进入 Ymodem 升级，等待上位机
  ******************************************************************************
  */

/* 头文件包含 */
#include "main.h"
#include "usart.h"
#include "ymod.h"


IWDG_HandleTypeDef hiwdg;

/* IWDG init function */
void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

void FLASH_ReadState()
{

    
}


/* 私有函数声明 */
void SystemClock_Config(void);

/**
  * @brief  程序入口
  * @retval 无（正常运行不返回）
  */
int main(void)
{
    uint32_t boot_tick;
    uint32_t c_tick;
    uint16_t iap_state;

    /* 复位所有外设，初始化 Flash 接口和 SysTick 滴答定时器 */
    HAL_Init();
    MX_IWDG_Init();

    /* 配置系统时钟：HSE 8MHz 经 PLL 6 倍频 = 48MHz */
    SystemClock_Config();

    /* RS485 串口初始化，注册 Ymodem 接收回调并启动接收 */
    MX_USART1_UART_Init();
    RS485_Init(&huart1);
    RS485_Set_Callback(Ymod_RxCallback);
    RS485_Start_Receive();
    
    iap_state    = *(__IO uint16_t *)FLASH_IAP_SATE_ADDR;
    
    if (iap_state == 0x04) {
        iap_load_app(IAP_APP_ADDR);
    }
    
    
    IAP_SetState(0x01);
     _PUTOUT(CNC);

    /* 升级主循环：收包处理写 Flash，超时重发 'C' */
    c_tick = HAL_GetTick();
    while (1)
    {
        HAL_IWDG_Refresh(&hiwdg);
        Ymod_Poll();
        if (!Ymod_IsInUpgrade()) {
            if (HAL_GetTick() - c_tick >= IAP_LISTEN_MS) {
                c_tick = HAL_GetTick();
                _PUTOUT(CNC);                          /* 待机态：每 100ms 一次 */
            }
        } 
    }
}

/**
  * @brief  系统时钟配置
  *         HSE 8MHz × PLL6 = 48MHz，SYSCLK/HCLK/PCLK 均为 48MHz
  * @retval 无
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* 配置振荡器：开启 HSE，PLL 时钟源选 HSE，6 倍频，预分频 1 */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* 配置总线时钟：SYSCLK 源选 PLL，AHB、APB 均不分频 */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  /* Flash 等待周期设为 1（48MHz 下必须），配置失败则进入错误处理 */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  HAL 库致命错误处理：关闭全局中断并进入死循环
  * @retval 无
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  断言参数检查失败回调（仅定义 USE_FULL_ASSERT 时编译）
  * @param  file: 出错源文件名指针
  * @param  line: 出错代码行号
  * @retval 无
  */
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
