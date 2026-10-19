/**
  ******************************************************************************
  * @file    ymod.c
  * @brief   Ymodem IAP 升级协议实现（STM32F042F6 / HAL 库版本）
  *          - 接收上位机下发的 .bin 文件，校验 CRC16/CRC32 后写入 App Flash
  *          - 支持文件结束(EOT)确认、空包头结束整批传输并跳转 App
  *          - 保留原工程的 Ymodem 数据包发送接口
  *          - 无 RS485 方向控制脚（自动收发芯片），应答直接阻塞发送
  ******************************************************************************
  */

/* 头文件包含 */
#include "ymod.h"
#include "usart.h"
#include <string.h>

/* 单包最大长度：SOH + 序号2 + 128数据 + CRC16 2 = 133 */
#define YMOD_PACK_BUF_SIZE   133

typedef struct YMOD
{
    uint32_t File_Len;            /* 文件总长度（首包数据区偏移 8~11，小端） */
    uint32_t Read_File_Len;       /* 已接收数据长度 */
    uint32_t Data_32_CRC;         /* 首包携带的整文件 CRC32（数据区偏移 0~3） */
    uint32_t Read_Data_32_CRC;    /* 接收过程中实时计算的 CRC32 */
    uint8_t  Positive_number;     /* 下一数据包的正向序号 */
    uint8_t  Reverse_numbering;   /* 下一数据包的反码序号 */
    uint8_t  ym_rx_status;        /* 接收状态机当前状态 */
    int      Appxaddr;            /* 已写入的数据包计数（每包 128 字节） */
} YMOD;

static YMOD Ymod;

/* 接收包缓冲（中断回调中拷入，主循环处理） */
static uint8_t  ymod_rx_buf[YMOD_PACK_BUF_SIZE];
static volatile uint16_t ymod_rx_len   = 0;
static volatile uint8_t  ymod_rx_ready = 0;
static volatile uint8_t  ymod_in_upgrade = 0;
static uint32_t ymod_last_tick = 0;

/* 发送用整包缓冲：3 字节头 + 128 数据 + 2 字节 CRC = 133 */
static uint8_t  Byte_count = 0xFF;
static uint8_t  Data[PACKET_SIZE + PACKET_OVERHEAD] = {0};
static uint32_t CRC32 = 0;
static uint16_t CRC16 = 0;

/* 私有函数声明 */
static void iap_erase_app(void);
static void (*boot_entry)(void);

void Ymod_Init(void)
{
    Ymod.Read_Data_32_CRC = 0xffffffff;
    Ymod.Data_32_CRC      = 0;
    Ymod.File_Len         = 0;
    Ymod.Read_File_Len    = 0;
    Ymod.Positive_number  = 0x00;
    Ymod.Reverse_numbering= 0xFF;
    Ymod.Appxaddr         = 0;
    Ymod.ym_rx_status     = YMODEM_RX_IDLE;

    ymod_rx_len   = 0;
    ymod_rx_ready = 0;
    ymod_last_tick = HAL_GetTick();
}

/**
  * @brief  判断一帧是否为升级握手命令：99 ?? ?? C1 ?? ?? 00 C4
  * @retval 1=是（已进入升级模式），0=否
  */
uint8_t Ymod_CheckUpgradeCmd(uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len < 8) return 0;
    if (buf[0] == YMOD_CMD_HEAD && buf[3] == YMOD_CMD_FUNC &&
        buf[6] == YMOD_CMD_RSV  && buf[7] == YMOD_CMD_TAIL)
    {
        Ymod_EnterUpgrade();
        return 1;
    }
    return 0;
}

/**
  * @brief  进入升级模式：复位状态、延时等待后发送字符 'C' 请求传输
  */
void Ymod_EnterUpgrade(void)
{
    Ymod_Init();
    ymod_in_upgrade = 1;
    HAL_Delay(300);
    _PUTOUT(CNC);
}

/**
  * @brief  是否处于升级模式
  */
uint8_t Ymod_IsInUpgrade(void)
{
    return ymod_in_upgrade;
}

/**
  * @brief  串口一帧接收完成回调（中断上下文调用）
  *         只负责拷贝数据并置位标志，协议处理放到主循环，避免在中断中擦写 Flash
  */
void Ymod_RxCallback(uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0 || len > YMOD_PACK_BUF_SIZE) return;
    memcpy((void *)ymod_rx_buf, data, len);
    ymod_rx_len = len;
    ymod_rx_ready = 1;
}

/**
  * @brief  升级模式主循环轮询
  *         有完整包则交给协议状态机；升级中超时无数据则重发 'C'
  *         非升级模式下仅识别握手命令（用于上电监听窗口）
  */
void Ymod_Poll(void)
{
    int len;
    if (ymod_rx_ready)
    {
        ymod_rx_ready = 0;
        if (ymod_in_upgrade)
        {
            ymodem_rx_put(ymod_rx_buf, ymod_rx_len);
        }
        else
        {
            if(Ymod_CheckUpgradeCmd(ymod_rx_buf, ymod_rx_len)) { 
            } else if (ymod_rx_len >= 128U &&
                     (ymod_rx_buf[0] == SOH || ymod_rx_buf[0] == STX)) {
                         len = ymod_rx_len;
                        Ymod_Init();
                        ymod_in_upgrade = 1;
                        ymod_rx_len = len;
                        ymodem_rx_put(ymod_rx_buf, ymod_rx_len);
                     }
        }
        ymod_last_tick = HAL_GetTick();
    }
    else if (ymod_in_upgrade && (HAL_GetTick() - ymod_last_tick >= YMOD_C_TIMEOUT_MS))
    {
        ymodem_rx_put(NULL, 0);
        ymod_last_tick = HAL_GetTick();
    }
    
}

/**
  * @brief  Ymodem 接收状态机：每收到一个完整包（或超时）调用一次
  * @param  buf   : 数据包；rx_sz 为 0 时可为 NULL，表示超时
  * @param  rx_sz : 包长度
  */
void ymodem_rx_put(uint8_t *buf, uint16_t rx_sz)
{
    /* 长度为 0（超时），发送 'C' 请求开始传输 */
    if (0 == rx_sz)
    {
        _PUTOUT(CNC);
        return;
    }
    
    if (ymod_rx_len >= 8 &&
        YMOD_CMD_HEAD == ymod_rx_buf[0] &&
        YMOD_CMD_FUNC == ymod_rx_buf[3] &&
        YMOD_CMD_RSV  == ymod_rx_buf[6] &&
        YMOD_CMD_TAIL == ymod_rx_buf[7])
    {
        Ymod_EnterUpgrade();   /* 复位状态机 + 延时 300ms + 应答 'C' */
        return;
    }
		
		if(ymod_rx_buf[0]== 0x01 && ymod_rx_buf[1]== 0xA5  && ymod_rx_buf[2]== 0xA5)
	{
		if(ymod_rx_buf[3] == 0x01)
		{
			_PUTOUT(0xF7);HAL_Delay(3);
			_PUTOUT(0x01);HAL_Delay(3);
			_PUTOUT(0xA5);HAL_Delay(3);
			_PUTOUT(0xA5);HAL_Delay(3);
			_PUTOUT(0xC4);HAL_Delay(3);
			_PUTOUT(0x8F);HAL_Delay(3);			
			return;
		}
	}
		 
    switch (Ymod.ym_rx_status)
    {
    /* ================= 空闲：等待文件信息包（序号 00） ================= */
    case YMODEM_RX_IDLE:
        switch (ymodem_rx_pac_check(buf, rx_sz))
        {
        case SOH:
            if (TRUE == ymodem_rx_pac_if_empty(buf + PACKET_HEADER, PACKET_SIZE))
            {
                /* 空文件名包：整批传输结束，应答后跳转到 App */
                Ymod.ym_rx_status = YMODEM_RX_EXIT;
                HAL_Delay(350);
                _PUTOUT(ACK);
                HAL_Delay(400);
                iap_load_app(IAP_APP_ADDR);
                /* 跳转失败（App 无效）才会执行到这里 */
                _PUTOUT(CAN);
                _PUTOUT(CAN);
                _PUTOUT(0xf5);
                HAL_Delay(1000);
                Ymod.ym_rx_status = YMODEM_RX_IDLE;
            }
            else
            {
                /* 文件信息包（序号 00，128 字节）：擦除 App 区后应答并请求数据 */
                iap_erase_app();

                Ymod.ym_rx_status = YMODEM_RX_ACK;
                HAL_Delay(350);
                _PUTOUT(ACK);
                /* 文件长度：数据区偏移 8~11，小端 */
                Ymod.File_Len = ((uint32_t)buf[PACKET_HEADER+11] << 24) |
                                ((uint32_t)buf[PACKET_HEADER+10] << 16) |
                                ((uint32_t)buf[PACKET_HEADER+9]  << 8) |
                                ((uint32_t)buf[PACKET_HEADER+8]);
                /* 整文件 CRC32：数据区偏移 0~3，小端 */
                Ymod.Data_32_CRC = ((uint32_t)buf[PACKET_HEADER+3] << 24) |
                                   ((uint32_t)buf[PACKET_HEADER+2] << 16) |
                                   ((uint32_t)buf[PACKET_HEADER+1] << 8) |
                                   ((uint32_t)buf[PACKET_HEADER+0]);
                HAL_Delay(500);
                _PUTOUT(CNC);
            }
            break;
        case EOT:
        default:
            /* 包有误，取消传输 */
            _PUTOUT(CAN);
            _PUTOUT(CAN);
            _PUTOUT(0xf7);
            HAL_Delay(1000);
            Ymod.ym_rx_status = YMODEM_RX_IDLE;
        }
        break;

    /* ================= 接收文件数据包中 ================= */
    case YMODEM_RX_ACK:
        switch (ymodem_rx_pac_check(buf, rx_sz))
        {
        case STX:
            /* 128 字节数据包：校验、累计 CRC32 并写入 Flash（重复序号不重写） */
            ymodem_rx_pac_get(buf, PACKET_SIZE);
            HAL_Delay(60);
            _PUTOUT(ACK);
            break;
        case EOT:
            /* 第一次 EOT：长度与 CRC32 均匹配则回 NAK，等待对方重发 EOT */
            if ((Ymod.Read_File_Len - Ymod.File_Len <= PACKET_SIZE) &&
                (Ymod.Read_Data_32_CRC == Ymod.Data_32_CRC))
            {
                Ymod.ym_rx_status = YMODEM_RX_EOT;
                HAL_Delay(350);
                _PUTOUT(NAK);
                HAL_Delay(60);
            }
            else
            {
                HAL_Delay(350);
                Ymod.ym_rx_status = YMODEM_RX_ERR;
                _PUTOUT(CAN);
                _PUTOUT(CAN);
                _PUTOUT(0xf9);
                HAL_Delay(1000);
                Ymod.ym_rx_status = YMODEM_RX_IDLE;
            }
            break;
        default:
            break;
        }
        break;

    /* ================= 等待第二次 EOT（结束确认） ================= */
    case YMODEM_RX_EOT:
        switch (ymodem_rx_pac_check(buf, rx_sz))
        {
        case EOT:
            HAL_Delay(350);
            _PUTOUT(ACK);
            HAL_Delay(300);
            _PUTOUT(CNC);
            Ymod.ym_rx_status = YMODEM_RX_IDLE;
            break;
        default:
            _PUTOUT(CAN);
            _PUTOUT(CAN);
            _PUTOUT(0xE8);
            HAL_Delay(1000);
            Ymod.ym_rx_status = YMODEM_RX_IDLE;
        }
        break;

    case YMODEM_RX_ERR:
        /* 放弃文件，终止传输 */
        _PUTOUT(CAN);
        _PUTOUT(CAN);
        _PUTOUT(0xE9);
        HAL_Delay(1000);

    case YMODEM_RX_EXIT:
        /* 收拾状态退出 */
        Ymod.ym_rx_status = YMODEM_RX_IDLE;
        ymod_in_upgrade = 0;
        return;

    default:
        break;
    }
}

/**
  * @brief  判断数据区是否全 0（用于识别空文件名包，数据区固定 128 字节）
  * @retval TRUE=全 0（空包），FALSE=非空
  */
uint8_t ymodem_rx_pac_if_empty(uint8_t *buf, uint16_t sz)
{
    uint16_t offset = 0;
    while (buf[offset] == 0x00 && ++offset < sz) {}
    if (offset == sz) return TRUE;
    return FALSE;
}

/**
  * @brief  检查包合法性并返回包类型
  * @retval SOH/EOT/ACK/NAK/CAN/CNC：有效；0xFF：无效包
  */
uint8_t ymodem_rx_pac_check(uint8_t *buf, uint16_t sz)
{
    uint16_t crc2;
    uint16_t crc1;
    uint8_t ch;
    uint16_t i;

    ch = buf[0];
    if (sz < 128)
    {
        /* 单字符指令包：包内所有字符相同才认为有效 */
        if (ch == EOT || ch == ACK || ch == NAK || ch == CAN || ch == CNC)
        {
            i = 1;
            while (i < sz && buf[i++] == ch) {}
            if (sz == i) return ch;
            return 0xff;
        }
        return 0xff;
    }
    else
    {
        /* 数据包：仅支持 128 字节的 SOH 包，CRC16 校验 + 序号/反码和为 0xFF */
        if (ch == SOH)
        {
            crc1 = crc16((uint8_t *)(buf + PACKET_HEADER), sz - PACKET_OVERHEAD);
            crc1 = (uint16_t)(crc1 << 8) | (uint16_t)(crc1 >> 8);
            crc2 = (uint16_t)buf[sz - 2] * 256 + buf[sz - 1];
            if (crc1 == crc2 && 0xff == (uint8_t)buf[1] + (uint8_t)buf[2])
                return ch;
        }
        
        if (ch == 0x02)
        {
            crc1 = crc16((uint8_t *)(buf + PACKET_HEADER), sz - PACKET_OVERHEAD);
            crc1 = (uint16_t)(crc1 << 8) | (uint16_t)(crc1 >> 8);
            crc2 = (uint16_t)buf[sz - 2] * 256 + buf[sz - 1];
            if (crc1 == crc2 && 0xff == (uint8_t)buf[1] + (uint8_t)buf[2])
                return ch;
        }
        return 0xff;
    }
}

/**
  * @brief  CRC16-XMODEM 校验（多项式 0x1021，初值 0，不反射）
  *    
  */
uint16_t crc16(uint8_t *buf, uint16_t count)
{
    uint16_t crc = 0;
    int i;

    while (count--)
    {
        crc = (uint16_t)(crc ^ ((uint16_t)(*buf++) << 8));
        for (i = 0; i < 8; i++)
        {
            if (crc & 0x8000)
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            else
                crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

uint32_t crc32(const uint8_t *buf, uint32_t count, uint32_t crc)
{
    static const uint32_t crc_table[16] = {
        0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC,
        0x76DC4190, 0x6B6B51F4, 0x4DB26158, 0x5005713C,
        0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C,
        0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C
    };
    uint32_t i;

    for (i = 0; i < count; i++)
    {
        crc ^= buf[i];
        crc = crc_table[crc & 0x0F] ^ (crc >> 4);
        crc = crc_table[crc & 0x0F] ^ (crc >> 4);
    }
    return crc;
}

uint8_t ymodem_rx_pac_get(uint8_t *buf, uint16_t sz)
{
    uint32_t remain;

    if (sz != PACKET_SIZE)
    {
        return YMODEM_ERR;
    }

    /* 序号与反码都与当前期望不符才认为是新包，否则按重传丢弃 */
    if (Ymod.Positive_number != buf[1] && Ymod.Reverse_numbering != buf[2])
    {
        Ymod.Positive_number++;
        Ymod.Reverse_numbering--;

        /* 本包中属于文件实际内容的字节数（末包可能不足 128） */
        remain = Ymod.File_Len - Ymod.Read_File_Len;
        if (remain > PACKET_SIZE)
        {
            remain = PACKET_SIZE;
        }

        /* CRC32 只累计实际文件字节，与首包声明的整文件 CRC 对应 */
        Ymod.Read_Data_32_CRC = crc32(buf + PACKET_HEADER, remain,
                                      Ymod.Read_Data_32_CRC);
        Ymod.Read_File_Len += remain;
        if (Ymod.Read_File_Len >= Ymod.File_Len)
        {
            /* 全部文件数据接收完成，结尾异或 0xFFFFFFFF */
            Ymod.Read_Data_32_CRC ^= 0xffffffff;
        }

        /* 整包 128 字节写入 Flash（末包填充字节不影响程序运行） */
        iap_write_appbin(IAP_APP_ADDR + (uint32_t)Ymod.Appxaddr * PACKET_SIZE,
                         buf + PACKET_HEADER, PACKET_SIZE);
        Ymod.Appxaddr++;
    }

    return YMODEM_OK;
}

/**
  * @brief  阻塞发送 1 个字节（无 RS485 方向脚，直接 HAL 阻塞发送）
  */
void _PUTOUT(uint8_t data)
{
    HAL_UART_Transmit(&huart1, &data, 1, 100);
}

/**
  * @brief  擦除整个 App 区（16 页 × 1K）
  */
static void iap_erase_app(void)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error = 0;

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = IAP_APP_ADDR;
    erase.NbPages = IAP_APP_SIZE / IAP_FLASH_PAGE_SIZE;

    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase, &page_error);
    HAL_FLASH_Lock();
}

/**
  * @brief  将数据写入 App Flash（按 16 位半字顺序写入）
  *         F042 Flash 最小编程单位为半字，调用前须保证所在页已擦除
  * @param  appxaddr: 32 位对齐的写入起始地址
  * @param  appbuf  : 数据缓冲
  * @param  appsize : 字节数
  */
void iap_write_appbin(uint32_t appxaddr, uint8_t *appbuf, uint32_t appsize)
{
    uint32_t i;
    uint16_t halfword;

    HAL_FLASH_Unlock();
    for (i = 0; i + 1 < appsize; i += 2)
    {
        halfword = (uint16_t)appbuf[i] | ((uint16_t)appbuf[i + 1] << 8);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, appxaddr + i, halfword);
    }
    /* 奇数个字节时高 8 位补 0xFF */
    if (appsize & 1)
    {
        halfword = (uint16_t)appbuf[i] | 0xFF00;
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, appxaddr + i, halfword);
    }
    HAL_FLASH_Lock();
}

/**
  * @brief  检查 App 是否有效（App 区第一个字为栈顶，应落在 6K SRAM 范围）
  * @retval 1=有效，0=无效（空片或未烧录）
  */
uint8_t IAP_App_IsValid(void)
{
    uint32_t app_sp = *(volatile uint32_t *)IAP_APP_ADDR;
    if ((app_sp & 0xFFFFE000U) == 0x20000000U)
        return 1;
    return 0;
}

/**
  * @brief  跳转到 App
  *         Cortex-M0 没有 VTOR 寄存器：先把 App 向量表复制到 SRAM 起始，
  *         再通过 SYSCFG 将 0x00000000 重映射到 SRAM，最后设置 MSP 并跳转。
  *         注意：App 工程 RAM 起始需预留 0xC0（IRAM1: 0x200000C0，Size 0x1740）
  * @param  appxaddr: App 起始地址
  */
void iap_load_app(uint32_t appxaddr)
{
    uint32_t boot_sp;
    uint32_t boot_pc;

    boot_sp = *(volatile uint32_t *)(appxaddr + 0U);   /* 初始 MSP  */
    boot_pc = *(volatile uint32_t *)(appxaddr + 4U);   /* Reset_Handler */

    if ((boot_sp & 0x2FFE0000U) != 0x20000000U)
        return;

    while (huart1.Instance->ISR & USART_ISR_RXNE)
    {
        (void)huart1.Instance->RDR;
    }

    __disable_irq();

    NVIC->ICER[0] = 0xFFFFFFFFU;
    NVIC->ICPR[0] = 0xFFFFFFFFU;

//    HAL_UART_DeInit(&huart1);
    __HAL_RCC_USART1_FORCE_RESET();     /* 硬复位：所有寄存器/标志归零 */
    __HAL_RCC_USART1_RELEASE_RESET();

    HAL_RCC_DeInit();

    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL  = 0U;
    
    for (int i = 0; i < 48U; i++)
        *(volatile uint32_t *)(0x20000000U + 4U * i) =
        *(volatile uint32_t *)(appxaddr + 4U * i);
    
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_SYSCFG_REMAPMEMORY_SRAM();
//    SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_MEM_MODE;   /* 00 = Flash 映射到 0x0 */

//    __enable_irq();

//    boot_entry = (void (*)(void))boot_pc;
    __set_MSP(boot_sp);
    __DSB();
    __ISB();
    ((void (*)(void))boot_pc)();
}

void Ymod_Send_Start(void)
{
    uint8_t Ymod_Buff_Start[8] = {0x99, 0x01, 0x00, 0xa5, 0x02, 0x00, 0x00, 0x42};
    HAL_UART_Transmit(&huart1, Ymod_Buff_Start, 8, 100);
}

void Ymod_Send_Buff(uint8_t *Ymodbuff)
{
    int i;

    Data[0] = SOH;
    /* 填充序号与反码 */
    Data[1] = 0xFF - Byte_count;
    Data[2] = Byte_count;

    CRC32 = crc32(Ymodbuff, PACKET_SIZE, CRC32);
    CRC16 = crc16(Ymodbuff, PACKET_SIZE);
    for (i = PACKET_HEADER; i < PACKET_SIZE + PACKET_HEADER; i++)
        Data[i] = *Ymodbuff++;
    Data[PACKET_SIZE + PACKET_HEADER]     = (CRC16 >> 8) & 0xFF;
    Data[PACKET_SIZE + PACKET_HEADER + 1] = CRC16 & 0xFF;
    for (i = 0; i < PACKET_SIZE + PACKET_OVERHEAD; i++)
        _PUTOUT(Data[i]);

    Byte_count--;
}
