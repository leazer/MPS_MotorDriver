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
void rt_hw_console_output(const char *str)
{
  rt_size_t i = 0, size = 0;
  char a = '\r';

  size = rt_strlen(str);

  for (i = 0; i < size; i++)
  {
    if (*(str + i) == '\n')
    {
      while(usart_flag_get(USART1, USART_TDBE_FLAG) == RESET);
      usart_data_transmit(USART1, (uint16_t)a);
      while(usart_flag_get(USART1, USART_TDC_FLAG) == RESET);
    }
    while(usart_flag_get(USART1, USART_TDBE_FLAG) == RESET);
    usart_data_transmit(USART1, (uint16_t)(*(str + i)));
    while(usart_flag_get(USART1, USART_TDC_FLAG) == RESET);
  }
}
#endif

#ifdef RT_USING_FINSH
char rt_hw_console_getchar(void)
{
  int ch = -1;
 
  if(usart_flag_get(USART1, USART_RDBF_FLAG) != RESET)
  {
    ch = usart_data_receive(USART1);
  }
 
  return ch;
}
#endif
