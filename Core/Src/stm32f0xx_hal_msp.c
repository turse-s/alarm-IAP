/**
  ******************************************************************************
  * @file    stm32f0xx_hal_msp.c
  * @brief   HAL 库 MSP（MCU Support Package）底层初始化
  *          全局 MSP 初始化位于本文件；外设专用 MSP 位于对应驱动文件
  ******************************************************************************
  */

/* 头文件包含 */
#include "main.h"

/**
  * @brief  全局 MSP 初始化（HAL_Init 内部回调）
  *         使能系统配置(SYSCFG)和电源(PWR)外设时钟
  */
void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}
