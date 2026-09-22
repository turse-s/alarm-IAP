/**
  ******************************************************************************
  * @file    usart.h
  * @brief   USART1(RS485) 驱动头文件，提供对外接口声明
  ******************************************************************************
  */
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern UART_HandleTypeDef huart1;   /* USART1 句柄（外部文件可引用） */

/* USART1 外设初始化（9600, 8N1） */
void MX_USART1_UART_Init(void);

/* RS485 初始化：绑定串口句柄 */
void RS485_Init(UART_HandleTypeDef *huart);
/* RS485 发送指定长度数据（阻塞式） */
void RS485_Send(uint8_t *data, uint16_t len);
/* RS485 发送以 '\0' 结尾的字符串 */
void RS485_Send_String(char *str);
/* 启动 RS485 中断接收 */
void RS485_Start_Receive(void);
/* 注册一帧接收完成回调函数 */
void RS485_Set_Callback(void (*callback)(uint8_t *data, uint16_t len));
/* 总线空闲中断处理（在 USART1_IRQHandler 中调用） */
void RS485_Process_IRQ(void);

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
