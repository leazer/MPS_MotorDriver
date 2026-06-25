/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-05-24                  the first version
 */

#include "rtthread_app.h"
#include "board_init_at32m412.h"
#include "board_usart1_dma.h"

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
/*
 * Please modify RT_HEAP_SIZE if you enable RT_USING_HEAP
 * the RT_HEAP_SIZE max value = (sram size - ZI size), 1024 means 1024 bytes
 */
#define RT_HEAP_SIZE (16 * 1024)
static rt_uint8_t rt_heap[RT_HEAP_SIZE];

RT_WEAK void *rt_heap_begin_get(void)
{
  return rt_heap;
}

RT_WEAK void *rt_heap_end_get(void)
{
  return rt_heap + RT_HEAP_SIZE;
}
#endif

#ifdef RT_USING_FINSH
#include <finsh.h>
static void reboot(uint8_t argc, char **argv)
{
  rt_hw_cpu_reset();
}
MSH_CMD_EXPORT(reboot, Reboot System);
#endif /* RT_USING_FINSH */

#ifdef RT_USING_CONSOLE
static int uart_init(void)
{
  board_usart1_init();
  return 0;
}
#ifndef RT_DEBUG_INIT
INIT_BOARD_EXPORT(uart_init);
#endif
#endif

/**
 * This function will initial your board.
 */
void rt_hw_board_init(void)
{
  /* system clock config. */
  wk_system_clock_config();

  /* 板级初始化: 外设时钟 + GPIO + NVIC (替代已清空的 wk_periph_clock_config/wk_nvic_config/wk_gpio_config)
   * 必须在 rt_components_board_init 之前, 否则 uart_init->board_usart1_init 时 USART1/GPIOB 时钟未开 */
  board_clock_init();
  board_gpio_init();
  board_nvic_init();

#if defined(RT_DEBUG_INIT) && defined(RT_USING_CONSOLE)
  uart_init();
#endif

  /* 
   * 1: OS Tick Configuration
   * Enable the hardware timer and call the rt_os_tick_callback function
   * periodically with the frequency RT_TICK_PER_SECOND. 
   */
  SysTick_Config(system_core_clock/RT_TICK_PER_SECOND);

  /* Call components board initial (use INIT_BOARD_EXPORT()) */
#ifdef RT_USING_COMPONENTS_INIT
  rt_components_board_init();
#endif

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
  rt_system_heap_init(rt_heap_begin_get(), rt_heap_end_get());
#endif
}

#ifdef RT_USING_CONSOLE
/* USART1 后端: DMA TX 单次 + DMA RX 循环环形 buffer.
 * 详见 platform/at32m412/board_usart1_dma.c.
 * \n -> \r\n 翻译在 board_usart1_tx_dma_send_str 内部完成. */
void rt_hw_console_output(const char *str)
{
  board_usart1_tx_dma_send_str(str);
}
#endif

#ifdef RT_USING_FINSH
/* 返回 int 而非 char: -1 表示无数据 (finsh_getchar 用 ch<0 判断).
 * 若返回 char, -1 经 char 截断变 0xFF, 提升回 int 变 255 (>0),
 * shell 误当有效字符处理, 破坏方向键 ESC 序列状态机 (ESC 后插入假 0xFF). */
int rt_hw_console_getchar(void)
{
  return board_usart1_rx_dma_getchar();
}
#endif
