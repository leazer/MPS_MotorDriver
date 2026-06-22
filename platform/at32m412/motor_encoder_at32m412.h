#ifndef MOTOR_ENCODER_AT32M412_H
#define MOTOR_ENCODER_AT32M412_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 初始化 MA600A: 内部完成 SPI2 硬件初始化 (时钟/GPIO MUX/spi_init) + ma600a_init.
 * 必须在 board_clock_init / board_gpio_init 之后调用 (SPI2 时钟由本函数开启). */
void motor_encoder_at32m412_init(void);

/* 读取角度 + 速度 (FOC ISR 内同步阻塞, spec §3.1 step 3)
 *   raw_angle_16: 12-bit MA600A 角度左移 4 位扩展为 16-bit
 *   raw_speed    : MA600A 速度寄存器原始值
 * retval 0=成功, 非0=故障码 (SPI 超时/总线错误)
 * 失败时递增内部错误计数, 连续失败会影响 motor_encoder_is_alive() */
int motor_encoder_read_angle_speed(uint16_t *raw_angle_16, int16_t *raw_speed);

/* 机械角 -> 电角度 (弧度 [0, 2π)), 含旁轴标定查表 + 零点修正 (spec §4.5.2 + §4.7.6)
 * 流程: raw_16 -> 查表校正 (若标定有效) -> 减零点 -> ×极对数 -> mod 2π */
float motor_encoder_to_electrical_angle(uint16_t raw_angle_16);

/* ===== 调试/状态接口 (供 shell 与标定状态机读取, 非 ISR 上下文) ===== */

/* 最近一次成功读取的原始角度 (16-bit 扩展) */
uint16_t motor_encoder_get_last_raw(void);

/* SPI 读取失败累计计数 (uint16 环绕) */
uint16_t motor_encoder_get_error_count(void);

/* 编码器是否存活: 最近 ENC_ALIVE_WINDOW 次读取中至少 1 次成功 */
bool motor_encoder_is_alive(void);

/* 设置/读取零点 (mech_zero_raw, 16-bit). 零点由标定或手动 mc_zero 写入. */
void motor_encoder_set_zero(uint16_t raw);
uint16_t motor_encoder_get_zero(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_ENCODER_AT32M412_H */
