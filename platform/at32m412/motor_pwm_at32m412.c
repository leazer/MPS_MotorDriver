#include "motor_pwm_at32m412.h"
#include "board_motor_pins.h"
#include "at32m412_416.h"
#include "motor_control_isr.h"

/* 内部: 限幅单通道占空比 */
#define PWM_ADC_TRIGGER_TICKS (TMR1_ARR - 360u)

static current_sample_tracker_t s_sample_tracker;

typedef struct {
    volatile uint32_t sequence;
    volatile uint16_t a;
    volatile uint16_t b;
    volatile uint16_t c;
} pwm_duty_request_t;

typedef struct {
    volatile uint32_t sequence;
    volatile uint16_t ticks;
} pwm_trigger_request_t;

static pwm_duty_request_t s_thread_duty_request;
static pwm_trigger_request_t s_thread_trigger_request;
static uint32_t s_duty_applied_sequence;
static uint32_t s_trigger_applied_sequence;
static volatile bool s_ovf_irq_enabled;
static volatile bool s_in_update_handler;

static uint16_t pwm_clamp_duty(uint16_t duty)
{
    if (duty > PWM_DUTY_MAX) {
        return PWM_DUTY_MAX;
    }
    return duty;
}

static void pwm_apply_duty_ticks(uint16_t a, uint16_t b, uint16_t c)
{
    tmr_channel_value_set(TMR1, PWM_PHASE_U_TMR_CHANNEL, a);
    tmr_channel_value_set(TMR1, PWM_PHASE_V_TMR_CHANNEL, b);
    tmr_channel_value_set(TMR1, PWM_PHASE_W_TMR_CHANNEL, c);
    current_sample_tracker_stage_duty(&s_sample_tracker, a, b, c);
}

static void pwm_apply_adc_trigger_ticks(uint16_t ticks)
{
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, ticks);
    current_sample_tracker_stage_trigger(&s_sample_tracker, ticks);
}

static bool pwm_update_must_be_deferred(void)
{
    return s_ovf_irq_enabled && !s_in_update_handler;
}

static void pwm_queue_duty_request(uint16_t a, uint16_t b, uint16_t c)
{
    uint32_t sequence;

    /* Odd means being written; the final even sequence publishes the RAM snapshot.
     * No timer register is touched here, so an asynchronous UEV cannot split it. */
    sequence = s_thread_duty_request.sequence;
    s_thread_duty_request.sequence = sequence + 1u;
    __DMB();
    s_thread_duty_request.a = a;
    s_thread_duty_request.b = b;
    s_thread_duty_request.c = c;
    __DMB();
    s_thread_duty_request.sequence = sequence + 2u;
}

static void pwm_queue_trigger_request(uint16_t ticks)
{
    uint32_t sequence;

    /* Same publish protocol as the three-duty request above. */
    sequence = s_thread_trigger_request.sequence;
    s_thread_trigger_request.sequence = sequence + 1u;
    __DMB();
    s_thread_trigger_request.ticks = ticks;
    __DMB();
    s_thread_trigger_request.sequence = sequence + 2u;
}

static void pwm_apply_pending_thread_updates(void)
{
    uint32_t sequence;
    uint16_t a;
    uint16_t b;
    uint16_t c;
    uint16_t ticks;

    /* This runs once per UEV after the control tick. Stable requests become the
     * final hardware preload + tracker plan for the following PWM cycle. */
    sequence = s_thread_duty_request.sequence;
    if (((sequence & 1u) == 0u) && (sequence != s_duty_applied_sequence)) {
        __DMB();
        a = s_thread_duty_request.a;
        b = s_thread_duty_request.b;
        c = s_thread_duty_request.c;
        __DMB();
        if (sequence == s_thread_duty_request.sequence) {
            pwm_apply_duty_ticks(a, b, c);
            s_duty_applied_sequence = sequence;
        }
    }

    sequence = s_thread_trigger_request.sequence;
    if (((sequence & 1u) == 0u) && (sequence != s_trigger_applied_sequence)) {
        __DMB();
        ticks = s_thread_trigger_request.ticks;
        __DMB();
        if (sequence == s_thread_trigger_request.sequence) {
            pwm_apply_adc_trigger_ticks(ticks);
            s_trigger_applied_sequence = sequence;
        }
    }
}

static void pwm_discard_pending_thread_updates(void)
{
    s_duty_applied_sequence = s_thread_duty_request.sequence;
    s_trigger_applied_sequence = s_thread_trigger_request.sequence;
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
    tmr_channel_value_set(TMR1, PWM_PHASE_U_TMR_CHANNEL, TMR1_ARR / 2u);
    tmr_channel_value_set(TMR1, PWM_PHASE_V_TMR_CHANNEL, TMR1_ARR / 2u);
    tmr_channel_value_set(TMR1, PWM_PHASE_W_TMR_CHANNEL, TMR1_ARR / 2u);

    /* --- 4. CH4: 输出比较模式, 触发 ADC 注入序列 ---
     * MP6540H 电流镜采样窗口需避开开关沿，默认在 ARR 前 360 ticks 采样.
     * 必须调用 tmr_output_channel_config 配置 CH4 为输出比较, 否则不产生
     * 比较事件, ADC 注入序列无法被 TMR1_CH4 触发 (spec §3.2).
     * 参考工程 mc_hwio.c: tmr_output_channel_config(ADC_TIMER, CH4, ...) */
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_4, &tmr_output_struct);
    tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_4, TRUE);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, PWM_ADC_TRIGGER_TICKS);
    current_sample_tracker_init(&s_sample_tracker,
                                TMR1_ARR / 2u,
                                PWM_ADC_TRIGGER_TICKS);
    s_thread_duty_request.sequence = 0u;
    s_thread_trigger_request.sequence = 0u;
    s_duty_applied_sequence = 0u;
    s_trigger_applied_sequence = 0u;
    s_ovf_irq_enabled = false;
    s_in_update_handler = false;

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
    uint16_t a;
    uint16_t b;
    uint16_t c;

    a = pwm_clamp_duty(phase_u);
    b = pwm_clamp_duty(phase_v);
    c = pwm_clamp_duty(phase_w);
    if (pwm_update_must_be_deferred()) {
        pwm_queue_duty_request(a, b, c);
        return;
    }
    pwm_apply_duty_ticks(a, b, c);
}

void motor_pwm_at32m412_set_adc_trigger_ticks(uint16_t ticks)
{
    if (ticks > TMR1_ARR) {
        ticks = TMR1_ARR;
    }
    if (pwm_update_must_be_deferred()) {
        pwm_queue_trigger_request(ticks);
        return;
    }
    pwm_apply_adc_trigger_ticks(ticks);
}

void motor_pwm_at32m412_get_sample_plan(current_sample_plan_t *out)
{
    current_sample_tracker_get_sampled(&s_sample_tracker, out);
}

void motor_pwm_at32m412_enable_ovf_irq(void)
{
    /* 使能 NVIC (优先级已在 board_nvic_init 设为 0=PRIO_FOC_ISR) + TMR1 OVF 中断使能 */
    current_sample_tracker_rearm_from_next(&s_sample_tracker);
    pwm_discard_pending_thread_updates();
    s_ovf_irq_enabled = true;
    nvic_irq_enable(TMR1_OVF_TMR10_IRQn, 0, 0);
    tmr_interrupt_enable(TMR1, TMR_OVF_INT, TRUE);
}

void motor_pwm_at32m412_disable_ovf_irq(void)
{
    tmr_interrupt_enable(TMR1, TMR_OVF_INT, FALSE);
    NVIC_DisableIRQ(TMR1_OVF_TMR10_IRQn);
    s_ovf_irq_enabled = false;
    pwm_discard_pending_thread_updates();
}

/* TMR1 溢出中断 (16kHz, 中心对齐顶点): 触发 FOC ISR.
 * NVIC 优先级 0 (最高, board_nvic_init 设置), 抢占 RT-Thread 调度. */
void TMR1_OVF_TMR10_IRQHandler(void)
{
    if (tmr_flag_get(TMR1, TMR_OVF_FLAG) != RESET) {
        tmr_flag_clear(TMR1, TMR_OVF_FLAG);
        s_in_update_handler = true;
        current_sample_tracker_latch_update(&s_sample_tracker);
        motor_control_isr_tick();
        pwm_apply_pending_thread_updates();
        s_in_update_handler = false;
    }
}
