#ifndef MOTOR_PWM_AT32M412_H
#define MOTOR_PWM_AT32M412_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* TMR1 完整初始化: 中心对齐 16kHz 3 路 PWM + CH4(ADC 顶点触发预留)
 * spec §1.3: TWO_WAY_3, RCR=1, ARR=5624 (180MHz), prescaler=1
 * 初始占空比 50% 三相同电位, MP6540H EN 保持低 (禁用)
 */
void motor_pwm_at32m412_init(void);

/* 安全初始化: init + disable_output + 占空比归零 */
void motor_pwm_at32m412_safe_init(void);

/* MP6540H EN 引脚控制 (高=使能, 低=禁用) */
void motor_pwm_at32m412_disable_output(void);
void motor_pwm_at32m412_enable_output(void);

/* 设置三相占空比 ticks (0..TMR1_ARR), 内部硬限幅 PWM_DUTY_MAX (95%) */
void motor_pwm_at32m412_set_duty_ticks(uint16_t phase_u, uint16_t phase_v, uint16_t phase_w);

#ifdef __cplusplus
}
#endif

#endif
