#ifndef MOTOR_PWM_AT32M412_H
#define MOTOR_PWM_AT32M412_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void motor_pwm_at32m412_safe_init(void);
void motor_pwm_at32m412_disable_output(void);
void motor_pwm_at32m412_enable_output(void);
void motor_pwm_at32m412_set_duty_ticks(uint16_t phase_u, uint16_t phase_v, uint16_t phase_w);

#ifdef __cplusplus
}
#endif

#endif
