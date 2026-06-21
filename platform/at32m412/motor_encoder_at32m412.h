#ifndef MOTOR_ENCODER_AT32M412_H
#define MOTOR_ENCODER_AT32M412_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 初始化 MA600A SPI2 适配 (封装 middlewares/msp/ma600) */
void motor_encoder_at32m412_init(void);

/* 读取角度 + 速度 (FOC ISR 内同步阻塞, spec §3.1 step 3)
 * retval 0=成功, 非0=故障码
 */
int motor_encoder_read_angle_speed(uint16_t *raw_angle_16, int16_t *raw_speed);

/* 机械角 -> 电角度 (弧度), 含旁轴标定查表 (spec §4.5.2 + §4.7.6) */
float motor_encoder_to_electrical_angle(uint16_t raw_angle_16);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_ENCODER_AT32M412_H */
