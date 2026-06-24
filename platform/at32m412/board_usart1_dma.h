#ifndef BOARD_USART1_DMA_H
#define BOARD_USART1_DMA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* USART1 (MSH/finsh console) DMA 后端
 *
 * 通道分配 (与官方 AT32M412_LV_MC_Library_Porject_V2.1.5 一致):
 *   - DMA1_CHANNEL2 / DMA1MUX_CHANNEL2 -> USART1_TX  (DMAMUX_DMAREQ_ID_USART1_TX = 0x19)
 *   - DMA1_CHANNEL3 / DMA1MUX_CHANNEL3 -> USART1_RX  (DMAMUX_DMAREQ_ID_USART1_RX = 0x18)
 *
 * RX 工作模式: 循环 DMA -> 软件环形 buffer (128B), CPU 不参与字符接收;
 *               getchar 通过 (size - dma_data_number_get) 计算 DMA 写指针, 对比软件读指针取字节.
 * TX 工作模式: 单次 DMA, 调用者阻塞等 FDT 标志 + TDC, 替代逐字节轮询.
 *
 * 不使用任何中断 (避免与 FOC/CAN 高优先级 ISR 争用); 不启用 RT_USING_DEVICE.
 */

#define USART1_RX_DMA_BUF_SIZE   128u
#define USART1_TX_STAGE_BUF_SIZE 256u   /* RT_CONSOLEBUF_SIZE(128) + \n->\r\n 最坏翻倍 */

/* 初始化 DMA1 时钟 + USART1 TX/RX 两个通道 + DMAMUX 映射, 并启动 RX 循环.
 * 必须在 board_usart1_init() 完成 usart_enable(USART1, TRUE) 之后调用. */
void board_usart1_dma_init(void);

/* 阻塞 DMA 发送: 把 data[0..len-1] 一次性推到 USART1 TX, 等 DMA FDT + USART TDC 完成才返回.
 * len 上限 USART1_TX_STAGE_BUF_SIZE; 超出部分由调用者自行截断. */
void board_usart1_tx_dma_send(const uint8_t *data, uint16_t len);

/* 把 str (C 字符串) 做 \n -> \r\n 翻译后通过 DMA 发送; 翻译后超过 stage 上限会丢弃尾部. */
void board_usart1_tx_dma_send_str(const char *str);

/* 从 RX DMA 环形 buffer 取一个字节; 无数据返回 -1 (非阻塞). */
int board_usart1_rx_dma_getchar(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_USART1_DMA_H */
