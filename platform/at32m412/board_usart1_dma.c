#include "board_usart1_dma.h"
#include "at32m412_416.h"

/* DMA buffer
 * - s_rx_ring   : DMA1_CH3 循环写入, 大小 = USART1_RX_DMA_BUF_SIZE
 * - s_rx_read_idx: 软件读指针, getchar 内部递增并取模
 * - s_tx_stage  : TX 临时缓冲, 由 board_usart1_tx_dma_send_str 做 \n->\r\n 翻译后填充, DMA1_CH2 从中读取
 *
 * 注意: AT32 DMA CNDTR 在循环模式下到 0 时立即 reload 回 buffer_size, 所以
 *       write_idx = USART1_RX_DMA_BUF_SIZE - dma_data_number_get(...) 始终落在 [0, size) 区间, 不会爆.
 */
static volatile uint8_t  s_rx_ring[USART1_RX_DMA_BUF_SIZE];
static volatile uint16_t s_rx_read_idx;
static uint8_t           s_tx_stage[USART1_TX_STAGE_BUF_SIZE];

void board_usart1_dma_init(void)
{
    dma_init_type dma_init_struct;

    /* DMA1 时钟 (DMAMUX 时钟在 AT32M412 上随 DMA1 开启) */
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

    /* ============ RX: DMA1_CHANNEL3 循环模式 (P2M, 1B) ============ */
    s_rx_read_idx = 0;
    dma_reset(DMA1_CHANNEL3);
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.buffer_size           = USART1_RX_DMA_BUF_SIZE;
    dma_init_struct.direction             = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_base_addr      = (uint32_t)&s_rx_ring[0];
    dma_init_struct.memory_data_width     = DMA_MEMORY_DATA_WIDTH_BYTE;
    dma_init_struct.memory_inc_enable     = TRUE;
    dma_init_struct.peripheral_base_addr  = (uint32_t)&USART1->dt;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority              = DMA_PRIORITY_LOW;
    dma_init_struct.loop_mode_enable      = TRUE;
    dma_init(DMA1_CHANNEL3, &dma_init_struct);

    /* DMAMUX 必须先 enable, 再做 flexible 映射 */
    dmamux_enable(DMA1, TRUE);
    dma_flexible_config(DMA1, DMA1MUX_CHANNEL3, DMAMUX_DMAREQ_ID_USART1_RX);

    /* 关 DMA RX 所有中断 (本方案 0 中断, 仅靠 USART1 RX EN + DMA 循环搬运) */
    dma_interrupt_enable(DMA1_CHANNEL3, DMA_FDT_INT,   FALSE);
    dma_interrupt_enable(DMA1_CHANNEL3, DMA_HDT_INT,   FALSE);
    dma_interrupt_enable(DMA1_CHANNEL3, DMA_DTERR_INT, FALSE);

    /* ============ TX: DMA1_CHANNEL2 单次模式 (M2P, 1B) ============ */
    dma_reset(DMA1_CHANNEL2);
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.buffer_size           = 0;  /* 每次发送前 dma_data_number_set */
    dma_init_struct.direction             = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_base_addr      = (uint32_t)&s_tx_stage[0];
    dma_init_struct.memory_data_width     = DMA_MEMORY_DATA_WIDTH_BYTE;
    dma_init_struct.memory_inc_enable     = TRUE;
    dma_init_struct.peripheral_base_addr  = (uint32_t)&USART1->dt;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority              = DMA_PRIORITY_LOW;
    dma_init_struct.loop_mode_enable      = FALSE;
    dma_init(DMA1_CHANNEL2, &dma_init_struct);

    dma_flexible_config(DMA1, DMA1MUX_CHANNEL2, DMAMUX_DMAREQ_ID_USART1_TX);

    dma_interrupt_enable(DMA1_CHANNEL2, DMA_FDT_INT,   FALSE);
    dma_interrupt_enable(DMA1_CHANNEL2, DMA_HDT_INT,   FALSE);
    dma_interrupt_enable(DMA1_CHANNEL2, DMA_DTERR_INT, FALSE);

    /* 使能 USART1 内 DMA TX/RX 请求位 (此后 USART 收到字节会触发 DMA 搬运) */
    usart_dma_transmitter_enable(USART1, TRUE);
    usart_dma_receiver_enable   (USART1, TRUE);

    /* 启动 RX 循环搬运; TX 在每次发送时按需 enable */
    dma_channel_enable(DMA1_CHANNEL3, TRUE);
}

void board_usart1_tx_dma_send(const uint8_t *data, uint16_t len)
{
    if (data == 0 || len == 0)
    {
        return;
    }
    if (len > USART1_TX_STAGE_BUF_SIZE)
    {
        len = USART1_TX_STAGE_BUF_SIZE;
    }

    /* 关闭 -> 设地址/长度 -> 清完成标志 -> 重新使能 -> 等 FDT
     * (AT32 dma_channel_enable(.., FALSE) 时硬件会把 CNDTR 锁住, 可以安全改 maddr/CNDTR) */
    dma_channel_enable(DMA1_CHANNEL2, FALSE);
    DMA1_CHANNEL2->maddr = (uint32_t)data;
    dma_data_number_set(DMA1_CHANNEL2, len);
    dma_flag_clear(DMA1_GL2_FLAG);   /* GL2 = FDT2 | HDT2 | DTERR2 的全局位, 一次清掉 */
    dma_channel_enable(DMA1_CHANNEL2, TRUE);

    /* 等 DMA 完成: 最坏 USART1_TX_STAGE_BUF_SIZE * 87us @ 115200 = 22ms,
     * 实际 finsh / rt_kprintf 单行 << 1ms. */
    while (dma_flag_get(DMA1_FDT2_FLAG) == RESET)
    {
        ;
    }
    /* DMA 完成只表示数据全部塞进 TDR, 还得等最后一字节移出移位寄存器,
     * 否则紧接着的下一次发送可能在 USART 未空闲时就重置 maddr. */
    while (usart_flag_get(USART1, USART_TDC_FLAG) == RESET)
    {
        ;
    }

    dma_channel_enable(DMA1_CHANNEL2, FALSE);
}

void board_usart1_tx_dma_send_str(const char *str)
{
    uint16_t in = 0;
    uint16_t out = 0;

    if (str == 0)
    {
        return;
    }

    /* 做 \n -> \r\n 翻译; 超过 stage 上限时截断, 保证不越界 */
    while (str[in] != '\0')
    {
        char c = str[in++];
        if (c == '\n')
        {
            if (out + 2 > USART1_TX_STAGE_BUF_SIZE) break;
            s_tx_stage[out++] = (uint8_t)'\r';
            s_tx_stage[out++] = (uint8_t)'\n';
        }
        else
        {
            if (out + 1 > USART1_TX_STAGE_BUF_SIZE) break;
            s_tx_stage[out++] = (uint8_t)c;
        }
    }

    if (out > 0)
    {
        board_usart1_tx_dma_send(s_tx_stage, out);
    }
}

int board_usart1_rx_dma_getchar(void)
{
    uint16_t write_idx;
    int      ch;

    /* CNDTR 是"剩余待传数", 反推已写入 buffer 的下标 */
    write_idx = (uint16_t)(USART1_RX_DMA_BUF_SIZE - dma_data_number_get(DMA1_CHANNEL3));

    if (write_idx == s_rx_read_idx)
    {
        return -1;  /* 无新数据 */
    }

    ch = (int)s_rx_ring[s_rx_read_idx];
    s_rx_read_idx = (uint16_t)((s_rx_read_idx + 1u) % USART1_RX_DMA_BUF_SIZE);
    return ch;
}
