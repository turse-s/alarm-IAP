/**
  ******************************************************************************
  * @file    stm32f0xx_it.h
  * @brief   中断服务函数头文件
  ******************************************************************************
  */
#ifndef __STM32F0xx_IT_H
#define __STM32F0xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* 内核异常处理函数 */
void NMI_Handler(void);
void HardFault_Handler(void);
void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F0xx_IT_H */
