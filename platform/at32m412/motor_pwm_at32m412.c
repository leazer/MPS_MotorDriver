#include "motor_pwm_at32m412.h"

#include "at32m412_416_wk_config.h"

void motor_pwm_at32m412_safe_init(void)
{
    motor_pwm_at32m412_disable_output();
    motor_pwm_at32m412_set_duty_ticks(0u, 0u, 0u);
}

void motor_pwm_at32m412_disable_output(void)
{
    gpio_bits_reset(PWM_EN_GPIO_PORT, PWM_EN_PIN);
}

void motor_pwm_at32m412_enable_output(void)
{
    gpio_bits_set(PWM_EN_GPIO_PORT, PWM_EN_PIN);
}

void motor_pwm_at32m412_set_duty_ticks(uint16_t phase_u, uint16_t phase_v, uint16_t phase_w)
{
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, phase_u);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, phase_v);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, phase_w);
}
