#ifndef BOARD_INIT_AT32M412_H
#define BOARD_INIT_AT32M412_H

#ifdef __cplusplus
extern "C" {
#endif

/* 板级初始化: 时钟 + GPIO + NVIC 优先级 (spec 方案 Y, 替代清空的 wk_*_init)
 * 调用顺序: board_clock_init -> board_gpio_init -> board_nvic_init
 *           -> 各外设模块 init (motor_pwm / current_sense / encoder / can ...)
 */

/* 外设时钟使能: GPIOA/GPIOB/TMR1/SCFG/PWC */
void board_clock_init(void);

/* 板级 GPIO: nFAULT 输入上拉 / LED 输出 / SPI2_CS 输出 / PWM_EN 输出(低, MP6540H 禁用) */
void board_gpio_init(void);

/* NVIC 优先级组 4 + 系统异常 + RT-Thread 调度优先级 + 外设中断优先级(仅设, 不使能) */
void board_nvic_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_INIT_AT32M412_H */
