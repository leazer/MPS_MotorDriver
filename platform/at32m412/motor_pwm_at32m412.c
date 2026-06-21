#include "motor_pwm_at32m412.h"
#include "board_motor_pins.h"
#include "at32m412_416.h"

/* 内部: 限幅单通道占空比 */
static uint16_t pwm_clamp_duty(uint16_t duty)
{
    if (duty > PWM_DUTY_MAX) {
        return PWM_DUTY_MAX;
    }
    return duty;
}

void motor_pwm_at32m412_init(void)
{
    gpio_init_type gpio_init_struct;
    tmr_output_config_type tmr_output_struct;
    tmr_brkdt_config_type tmr_brkdt_struct;

    /* --- 1. PA8/PA9/PA10 配置为 TMR1 CH1/2/3 复用 (MUX_1) --- */
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;

    gpio_pin_mux_config(PWMA_GPIO_PORT, PWMA_PIN_SOURCE, PWMA_IOMUX);
    gpio_init_struct.gpio_pins = PWMA_PIN;
    gpio_init(PWMA_GPIO_PORT, &gpio_init_struct);

    gpio_pin_mux_config(PWMB_GPIO_PORT, PWMB_PIN_SOURCE, PWMB_IOMUX);
    gpio_init_struct.gpio_pins = PWMB_PIN;
    gpio_init(PWMB_GPIO_PORT, &gpio_init_struct);

    gpio_pin_mux_config(PWMC_GPIO_PORT, PWMC_PIN_SOURCE, PWMC_IOMUX);
    gpio_init_struct.gpio_pins = PWMC_PIN;
    gpio_init(PWMC_GPIO_PORT, &gpio_init_struct);

    /* --- 2. TMR1 基础配置: 中心对齐 16kHz --- */
    tmr_base_init(TMR1, TMR1_ARR, 0);                       /* ARR=5624, prescaler=1 (div+1=1) */
    tmr_cnt_dir_set(TMR1, TMR_COUNT_TWO_WAY_3);             /* 中心对齐模式3 (上下都置位) */
    tmr_repetition_counter_set(TMR1, 1);                    /* RCR=1 */
    tmr_clock_source_div_set(TMR1, TMR_CLOCK_DIV1);         /* DTS 分频 (死区/ETR 滤波用) */
    tmr_period_buffer_enable(TMR1, TRUE);                   /* ARR 预装载 */

    /* 主从模式: 独立运行 */
    tmr_sub_sync_mode_set(TMR1, FALSE);
    tmr_primary_mode_select(TMR1, TMR_PRIMARY_SEL_RESET);
    tmr_primary_mode2_select(TMR1, TMR_PRIMARY_SEL_RESET);

    /* --- 3. CH1/2/3 输出比较: PWM_MODE_A, 高电平有效 --- */
    tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_output_struct.oc_output_state = TRUE;                /* 主输出使能 */
    tmr_output_struct.occ_output_state = FALSE;              /* 互补不用 (MP6540H 高边特化) */
    tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_struct.oc_idle_state = FALSE;                 /* 空闲低 (MP6540H 关断) */
    tmr_output_struct.occ_idle_state = FALSE;

    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &tmr_output_struct);
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_2, &tmr_output_struct);
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_3, &tmr_output_struct);

    /* CCR 预装载 + 初始 50% (三相同电位, 不出力) */
    tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_1, TRUE);
    tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_2, TRUE);
    tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_3, TRUE);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, TMR1_ARR / 2u);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, TMR1_ARR / 2u);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, TMR1_ARR / 2u);

    /* --- 4. CH4: 比较值=ARR, 为 Stage 3 ADC 顶点触发预留 (Stage 1 不触发 ADC) --- */
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, TMR1_ARR);

    /* --- 5. 刹车/死区: 不使能刹车 (MP6540H nFAULT 走 EXINT, 不接 TMR1_BRK) --- */
    tmr_brkdt_struct.brk_enable = FALSE;
    tmr_brkdt_struct.auto_output_enable = FALSE;
    tmr_brkdt_struct.brk_polarity = TMR_BRK_INPUT_ACTIVE_LOW;
    tmr_brkdt_struct.fcsoen_state = FALSE;
    tmr_brkdt_struct.fcsodis_state = FALSE;
    tmr_brkdt_struct.wp_level = TMR_WP_OFF;
    tmr_brkdt_struct.deadtime = 0;                           /* MP6540H 内部驱动, 无死区 */
    tmr_brkdt_config(TMR1, &tmr_brkdt_struct);
    tmr_brk_filter_value_set(TMR1, 0);

    /* --- 6. 使能输出 + 启动计数 (MP6540H EN 仍为低, PWM 不驱动电机) --- */
    tmr_output_enable(TMR1, TRUE);
    tmr_counter_enable(TMR1, TRUE);
}

void motor_pwm_at32m412_safe_init(void)
{
    motor_pwm_at32m412_init();           /* TMR1 完整初始化 (50% 三相同电位) */
    motor_pwm_at32m412_disable_output(); /* MP6540H EN=低, 保持禁用 */
    motor_pwm_at32m412_set_duty_ticks(TMR1_ARR / 2u, TMR1_ARR / 2u, TMR1_ARR / 2u);
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
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, pwm_clamp_duty(phase_u));
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, pwm_clamp_duty(phase_v));
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, pwm_clamp_duty(phase_w));
}
