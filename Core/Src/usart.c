/**
  ******************************************************************************
  * @file    usart.c
  * @brief   USART1(RS485) 底层配置与收发驱动
  *          通信参数：115200bps，8 数据位，1 停止位，无校验
  *          帧界定方式：单字节中断接收 + 总线空闲(IDLE)中断判定一帧结束
  *          接收缓冲区可容纳最大 Ymodem 数据包(133 字节)
  ******************************************************************************
  */

/* 头文件包含 */
#include "usart.h"
#include <string.h>

/* 接收缓冲区大小（字节）：最大 Ymodem 128 包 = 3+128+2 = 133，向上取整留余量 */
#define RS485_RX_BUFFER_SIZE    136
/* 单次发送最大长度（字节） */
#define RS485_TX_BUFFER_SIZE    64

/* RS485 工作状态 */
typedef enum {
    RS485_IDLE = 0,      /* 空闲 */
    RS485_RECEIVING,     /* 接收中 */
    RS485_TX_COMPLETE    /* 发送完成 */
} RS485_State_t;

/* RS485 收发管理结构体 */
typedef struct {
    UART_HandleTypeDef *huart;                        /* 串口句柄 */
    uint8_t rx_buffer[RS485_RX_BUFFER_SIZE];          /* 接收缓冲区 */
    uint16_t rx_index;                                /* 当前接收位置索引 */
    uint16_t rx_len;                                  /* 一帧数据的长度 */
    RS485_State_t state;                              /* 当前工作状态 */
    void (*rx_callback)(uint8_t *data, uint16_t len); /* 一帧接收完成回调函数 */
} RS485_Handle_t;

UART_HandleTypeDef huart1;     /* USART1 句柄 */
RS485_Handle_t h485 = {0};     /* RS485 收发管理实体 */

/**
  * @brief  RS485 初始化：绑定串口句柄，清零接收状态与缓冲区
  * @param  huart: 串口句柄指针
  */
void RS485_Init(UART_HandleTypeDef *huart)
{
    h485.huart = huart;
    h485.rx_index = 0;
    h485.rx_len = 0;
    h485.state = RS485_IDLE;
    h485.rx_callback = NULL;
    memset(h485.rx_buffer, 0, RS485_RX_BUFFER_SIZE);
}

/**
  * @brief  RS485 发送数据（阻塞式，超时 100ms）
  * @param  data: 待发送数据指针
  * @param  len : 待发送字节数（超过上限自动截断）
  */
void RS485_Send(uint8_t *data, uint16_t len)
{
    if (len == 0 || data == NULL) return;
    if (len > RS485_TX_BUFFER_SIZE) len = RS485_TX_BUFFER_SIZE;

    HAL_UART_Transmit(h485.huart, data, len, 100);
    h485.state = RS485_TX_COMPLETE;
}

/**
  * @brief  RS485 发送字符串
  * @param  str: 以 '\0' 结尾的字符串
  */
void RS485_Send_String(char *str)
{
    if (str == NULL) return;
    RS485_Send((uint8_t *)str, strlen(str));
}

/**
  * @brief  启动 RS485 中断接收（每次接收 1 个字节），并使能总线空闲中断
  */
void RS485_Start_Receive(void)
{
    h485.rx_index = 0;
    h485.rx_len = 0;
    h485.state = RS485_RECEIVING;
    __HAL_UART_ENABLE_IT(h485.huart, UART_IT_IDLE);
    HAL_UART_Receive_IT(h485.huart, &h485.rx_buffer[0], 1);
}

/**
  * @brief  注册"一帧数据接收完成"回调函数
  * @param  callback: 用户回调，参数为数据指针和帧长度；不需要时传 NULL
  */
void RS485_Set_Callback(void (*callback)(uint8_t *data, uint16_t len))
{
    h485.rx_callback = callback;
}

/**
  * @brief  RS485 总线空闲中断处理
  *         总线产生空闲中断即认为一帧结束：锁存长度、调用用户回调，
  *         然后清零索引并重新启动接收
  * @note   在 USART1_IRQHandler 中调用
  */
void RS485_Process_IRQ(void)
{
    /* 判断是否为总线空闲标志 */
    if (__HAL_UART_GET_FLAG(h485.huart, UART_FLAG_IDLE))
	{
        /* 清除空闲标志（读 SR 再读 DR 的操作由该宏完成） */
        __HAL_UART_CLEAR_IDLEFLAG(h485.huart);

        /* 锁存本帧长度 */
        h485.rx_len = h485.rx_index;

        /* 收到有效数据时调用用户回调 */
        if (h485.rx_len > 0)
		{
            if (h485.rx_callback != NULL)
			{
                h485.rx_callback(h485.rx_buffer, h485.rx_len);
            }
            /* 复位接收索引和长度，准备下一帧 */
            h485.rx_index = 0;
            h485.rx_len = 0;
        }

        /* 中止回调中已自动挂起的旧接收，重新从缓冲区首字节挂起 */
        h485.state = RS485_RECEIVING;
        HAL_UART_AbortReceive(h485.huart);
        HAL_UART_Receive_IT(h485.huart, &h485.rx_buffer[0], 1);
    }
}

/**
  * @brief  HAL 库单字节接收完成回调（弱定义函数的重写）
  *         每收到 1 个字节进入一次：索引后移，缓冲区满则回卷，
  *         并立即重新开放下一个字节的中断接收
  * @param  huart: 触发回调的串口句柄
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == h485.huart->Instance)
	{
        /* 写入位置后移 */
        h485.rx_index++;
        /* 索引到达缓冲区末尾时回卷到 0 */
        if (h485.rx_index >= RS485_RX_BUFFER_SIZE)
		{
            h485.rx_index = 0;
        }
        /* 继续接收下一个字节，存入索引指向的位置 */
        HAL_UART_Receive_IT(h485.huart, &h485.rx_buffer[h485.rx_index], 1);
    }
}

/**
  * @brief  USART1 初始化：115200, 8N1，收发模式，无硬件流控，16 倍过采样
  */
void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;                   /* 波特率 115200 */
    huart1.Init.WordLength = UART_WORDLENGTH_8B;     /* 8 位数据位 */
    huart1.Init.StopBits = UART_STOPBITS_1;          /* 1 位停止位 */
    huart1.Init.Parity = UART_PARITY_NONE;           /* 无校验 */
    huart1.Init.Mode = UART_MODE_TX_RX;              /* 收发模式 */
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;     /* 无硬件流控 */
    huart1.Init.OverSampling = UART_OVERSAMPLING_16; /* 16 倍过采样 */
    HAL_UART_Init(&huart1);
}

/**
  * @brief  USART1 底层初始化（HAL_UART_Init 内部回调）
  *         开启 USART1、GPIOA 时钟，配置 PA9/PA10，并开启 USART1 中断
  * @param  uartHandle: 串口句柄
  */
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(uartHandle->Instance == USART1)
	{
        /* 使能 USART1 和 GPIOA 外设时钟 */
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* PA9 = USART1_TX：复用推挽输出，上拉，高速 */
        GPIO_InitStruct.Pin = GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* PA10 = USART1_RX：复用输入，上拉（F0 接收脚同样需配置 AF1） */
        GPIO_InitStruct.Pin = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USART1 中断：抢占优先级 5，子优先级 0，并使能 */
				HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10    ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);
      
  /* USART1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
    /* USER CODE BEGIN USART1_MspDeInit 1 */
    __HAL_UART_DISABLE_IT(uartHandle,UART_IT_RXNE);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/**
  * @brief  USART1 中断服务函数
  *         先处理总线空闲帧界定，再交给 HAL 库处理接收等中断
  */
void USART1_IRQHandler(void)
{
    RS485_Process_IRQ();
    HAL_UART_IRQHandler(&huart1);
}
