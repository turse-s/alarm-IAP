/**
  ******************************************************************************
  * @file    main.h
  * @brief   主程序头文件，包含 HAL 头文件及全局函数声明
  ******************************************************************************
  */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"

/* HAL 库致命错误处理函数（main.c 中实现） */
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
