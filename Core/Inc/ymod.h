/**
  ******************************************************************************
  * @file    ymod.h
  * @brief   Ymodem IAP 升级协议（接收写 Flash / 发送）及 App 跳转接口
  *          由 STM32F407 标准外设库版本移植到 STM32F042F6 HAL 库
  *
  *          Flash 分区（共 32K，每页 1K）：
  *            Bootloader : 0x08000000 ~ 0x08003FFF (16K)
  *            App        : 0x08004000 ~ 0x08007FFF (16K)
  ******************************************************************************
  */
#ifndef __YMOD_H__
#define __YMOD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* App 区起始地址、总大小、Flash 页大小 */
#define IAP_APP_ADDR        0x08004000U
#define IAP_APP_SIZE        0x00004000U
#define IAP_FLASH_PAGE_SIZE 0x00000400U

/* 上电后等待升级命令的监听窗口（毫秒），超时且 App 有效则跳转 */
#define IAP_LISTEN_MS       10000U
/* 升级过程中收不到数据时重发字符 'C' 的超时（毫秒） */
#define YMOD_C_TIMEOUT_MS   3000U

/* 布尔定义 */
#define TRUE                1
#define FALSE               0

/* 接收状态机状态 */
#define YMODEM_RX_IDLE      0
#define YMODEM_RX_ACK       1
#define YMODEM_RX_EOT       2
#define YMODEM_RX_ERR       3
#define YMODEM_RX_EXIT      4

/* 数据包结构：[SOH][序号][序号反码][128字节数据][CRC高][CRC低] */
#define PACKET_SEQNO_INDEX      (1)
#define PACKET_SEQNO_COMP_INDEX (2)
#define PACKET_HEADER           (3)     /* 头：控制字 + 序号 + 反码 */
#define PACKET_TRAILER          (2)     /* 尾：CRC16 两个字节 */
#define PACKET_OVERHEAD         (PACKET_HEADER + PACKET_TRAILER)
#define PACKET_SIZE             (128)   /* 本工程仅支持 128 字节数据包 */

/* Ymodem ASCII 控制字符 */
#define SOH (0x01)   /* 128 字节数据包起始（仅支持该格式） */
#define STX (0x02)   /* 128 字节数据包传输 */
#define EOT (0x04)   /* 传输结束 */
#define ACK (0x06)   /* 确认 */
#define NAK (0x15)   /* 否认/请求重传 */
#define CAN (0x18)   /* 取消传输 */
#define CNC (0x43)   /* 字符 'C'：请求以 CRC 方式传输 */

/* 函数返回值 */
#define YMODEM_OK               0
#define YMODEM_ERR              1

/* 升级握手命令帧（8 字节）：99 ?? ?? C1 ?? ?? 00 C4 */
#define YMOD_CMD_HEAD       0x99
#define YMOD_CMD_FUNC       0xC3
#define YMOD_CMD_RSV        0x00
#define YMOD_CMD_TAIL       0xC6

/* -------- 对外接口 -------- */

/* 初始化 Ymodem 接收状态（不发送 'C'） */
void    Ymod_Init(void);
/* 判断一帧是否为升级握手命令；是则初始化升级状态并返回 1 */
uint8_t Ymod_CheckUpgradeCmd(uint8_t *buf, uint16_t len);
/* 进入升级模式：复位接收状态并主动发送字符 'C' */
void    Ymod_EnterUpgrade(void);
/* 是否正处于升级模式（监听窗口内调用） */
uint8_t Ymod_IsInUpgrade(void);
/* 串口"一帧接收完成"回调（在中断中调用），数据拷入内部缓冲 */
void    Ymod_RxCallback(uint8_t *data, uint16_t len);
/* 升级模式主循环轮询：处理收到的包，超时重发 'C' */
void    Ymod_Poll(void);

/* 检查 App 区第一个字（栈顶）是否合法 */
uint8_t IAP_App_IsValid(void);
/* 跳转到 App（Cortex-M0：向量表复制到 SRAM 起始并重映射） */
void    iap_load_app(uint32_t appxaddr);

/* -------- Ymodem 协议内部函数（保留原工程接口名） -------- */
void     ymodem_rx_put(uint8_t *buf, uint16_t rx_sz);
uint8_t  ymodem_rx_pac_if_empty(uint8_t *buf, uint16_t sz);
uint8_t  ymodem_rx_pac_check(uint8_t *buf, uint16_t sz);
uint16_t crc16(uint8_t *buf, uint16_t count);
uint32_t crc32(const uint8_t *buf, uint32_t count, uint32_t crc);
uint8_t  ymodem_rx_pac_get(uint8_t *buf, uint16_t sz);
void     _PUTOUT(uint8_t data);
void     iap_write_appbin(uint32_t appxaddr, uint8_t *appbuf, uint32_t appsize);
void     Ymod_Send_Start(void);
void     Ymod_Send_Buff(uint8_t *Ymodbuff);

#ifdef __cplusplus
}
#endif

#endif /* __YMOD_H__ */
