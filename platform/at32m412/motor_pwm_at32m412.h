#ifndef MOTOR_PWM_AT32M412_H
#define MOTOR_PWM_AT32M412_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* TMR1 完整初始化: 中心对齐 16kHz 3 路 PWM + CH4(ADC 触发)
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

/* 设置 TMR1_CH4 ADC 注入触发比较点 (调试采样窗口用, 0..ARR) */
void motor_pwm_at32m412_set_adc_trigger_ticks(uint16_t ticks);

/* 使能 TMR1_OVF 中断 (16kHz, FOC ISR 触发). NVIC 优先级已在 board_nvic_init 设置.
 * Stage 2: motor_control_isr_open_loop_start 调用, 之后 ISR 每周期执行一次. */
void motor_pwm_at32m412_enable_ovf_irq(void);

/* 禁用 TMR1_OVF 中断 (停止 FOC ISR) */
void motor_pwm_at32m412_disable_ovf_irq(void);

#ifdef __cplusplus
}
#endif

#endif
