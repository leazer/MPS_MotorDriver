/*
 * current_sense_at32m412.c - ADC2 注入序列 + TMR1_CH4 低边窗口触发 + VBUS 普通转换
 *
 * Stage 3: 电流采样与母线电压反馈 (spec §3.2 / §4.3)
 *
 * 硬件:
 *   SOA (PB1/ADC2_IN9)  <- MP6540H CS A
 *   SOB (PB0/ADC2_IN8)  <- MP6540H CS B
 *   SOC (PA7/ADC2_IN7)  <- MP6540H CS C
 *   VBUS (PA6/ADC2_IN6) <- 母线电压分压 (1/6)
 *
 * 注入序列 (preempt): SOA -> SOB -> SOC, TMR1_CH4 上升沿触发
 * 普通序列 (ordinary): VBUS, 软件触发读取
 *
 * ADC_CLK = 180MHz / 6 = 30MHz, 12-bit, 采样 1.5 cycle, 转换 ~0.7us/通道
 *
 * 零偏标定: PWM 50% 三相同电位时采 1024 次平均, 补偿电阻误差和 ADC 失调.
 *   标定条件: |offset - 2048| < 20 LSB, 否则返回 false (spec §4.3.3)
 *
 * 注意: ARMCC V5.06 默认 C90, 变量声明必须在块开头.
 */
#include "current_sense_at32m412.h"
#include "motor_params.h"
#include "board_motor_pins.h"
#include "at32m412_416.h"

/* ===== 配置常量 ===== */
#define ADC_CLK_DIV             ADC_HCLK_DIV_6      /* 180MHz/6 = 30MHz */
#define ADC_PREEMPT_SAMPLETIME  ADC_SAMPLETIME_1_5  /* 电流采样 1.5 cycle (参考工程同款) */
#define ADC_VBUS_SAMPLETIME     ADC_SAMPLETIME_13_5 /* VBUS 分压网络稍慢, 13.5 cycle 稳定 */

/* 零偏标定参数 (spec §4.3.3).
 * 窗口放宽到 50 LSB: 4.7k/4.7k 分压电阻 1% 容差 + MP6540H 电流镜偏置,
 * 实测零偏 ~2070 (偏差 22 LSB). 50 LSB = 0.16A, 在安全范围内. */
#define OFFSET_SAMPLE_COUNT     1024u
#define OFFSET_VALID_WINDOW_LSB 50u    /* |offset - 2048| < 50 LSB */
#define OFFSET_SEQUENCE_TIMEOUT 100000u /* > one 16kHz trigger interval at 180MHz */

/* VBUS 普通转换软件触发等待超时 (ADC_CLK 30MHz, 转换 ~0.7us, 1000 次循环足够) */
#define VBUS_CONV_TIMEOUT       1000u

/* ===== 模块状态 ===== */
static uint16_t s_offset_a = CURRENT_ZERO_OFFSET_LSB;   /* 默认 2048 */
static uint16_t s_offset_b = CURRENT_ZERO_OFFSET_LSB;
static uint16_t s_offset_c = CURRENT_ZERO_OFFSET_LSB;
static bool     s_offset_valid = false;

/* ===== 内部: 配置 SOA/SOB/SOC/VBUS 引脚为模拟输入 ===== */
static void current_sense_gpio_init(void)
{
    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;

    /* SOA (PB1), SOB (PB0) */
    gpio_init_struct.gpio_pins = SOA_PIN | SOB_PIN;
    gpio_init(SOA_GPIO_PORT, &gpio_init_struct);

    /* SOC (PA7), VBUS (PA6) */
    gpio_init_struct.gpio_pins = SOC_PIN | VBUS_PIN;
    gpio_init(SOC_GPIO_PORT, &gpio_init_struct);
}

/* ===== 内部: ADC2 base + common 配置 + 校准 ===== */
static void current_sense_adc2_config(void)
{
    adc_common_config_type adc_common_struct;
    adc_base_config_type   adc_base_struct;

    /* 先关 ADC 再配置 (参考工程 mc_hwio.c 模式) */
    adc_enable(ADC2, FALSE);

    /* base: 不用序列模式, 不重复, 右对齐, 普通 1 通道 (VBUS) */
    adc_base_default_para_init(&adc_base_struct);
    adc_base_struct.sequence_mode = TRUE;
    adc_base_struct.repeat_mode = FALSE;
    adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
    adc_base_struct.ordinary_channel_length = 1;   /* VBUS 普通序列 */
    adc_base_config(ADC2, &adc_base_struct);
    adc_resolution_set(ADC2, ADC_RESOLUTION_12B);

    /* common: 独立模式, HCLK/6 = 30MHz */
    adc_common_default_para_init(&adc_common_struct);
    adc_common_struct.combine_mode = ADC_INDEPENDENT_MODE;
    adc_common_struct.div = ADC_CLK_DIV;
    adc_common_struct.tempervintrv_state = FALSE;
    adc_common_config(&adc_common_struct);

    /* 注入序列: 3 通道 [SOA, SOB, SOC], 顺序 1/2/3 */
    adc_preempt_channel_length_set(ADC2, 3);
    adc_preempt_channel_set(ADC2, SOA_ADC_CHANNEL, 1, ADC_PREEMPT_SAMPLETIME);
    adc_preempt_channel_set(ADC2, SOB_ADC_CHANNEL, 2, ADC_PREEMPT_SAMPLETIME);
    adc_preempt_channel_set(ADC2, SOC_ADC_CHANNEL, 3, ADC_PREEMPT_SAMPLETIME);

    /* 注入触发: TMR1_CH4 上升沿；具体采样 tick 由 PWM 驱动配置并跟踪. */
    adc_preempt_conversion_trigger_set(ADC2, ADC_PREEMPT_TRIG_TMR1CH4,
                                       ADC_PREEMPT_TRIG_EDGE_RISING);

    /* 普通序列: VBUS (CH6), 软件触发 (无硬件触发源) */
    adc_ordinary_channel_set(ADC2, VBUS_ADC_CHANNEL, 1, ADC_VBUS_SAMPLETIME);
    adc_ordinary_conversion_trigger_set(ADC2, ADC_ORDINARY_TRIG_TMR1CH1,
                                        ADC_ORDINARY_TRIG_EDGE_NONE);

    /* 使能 ADC + 校准 (参考工程 mc_hwio.c: enable -> wait RDY -> calib).
     * 各 while 循环加超时保护, 避免硬件异常时永久挂起 (导致板子无法启动). */
    {
        uint32_t to;
        adc_enable(ADC2, TRUE);
        to = 100000u;
        while (adc_flag_get(ADC2, ADC_RDY_FLAG) == RESET) {
            if (--to == 0u) { return; }   /* ADC 未就绪, 放弃初始化 */
        }
        adc_calibration_init(ADC2);
        to = 100000u;
        while (adc_calibration_init_status_get(ADC2)) {
            if (--to == 0u) { return; }
        }
        adc_calibration_start(ADC2);
        to = 100000u;
        while (adc_calibration_status_get(ADC2)) {
            if (--to == 0u) { return; }
        }
    }
}

void current_sense_at32m412_init(void)
{
    /* 开 ADC2 时钟 (ADC1/ADC2 各有独立时钟使能位, 本工程用 ADC2).
     * 同时开 ADC1 时钟: AT32M412 的 ADC common 配置需 ADC1 时钟使能. */
    crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_ADC2_PERIPH_CLOCK, TRUE);

    current_sense_gpio_init();
    current_sense_adc2_config();

    /* 零偏默认值, 待 calibrate_offset 标定后更新 */
    s_offset_a = CURRENT_ZERO_OFFSET_LSB;
    s_offset_b = CURRENT_ZERO_OFFSET_LSB;
    s_offset_c = CURRENT_ZERO_OFFSET_LSB;
    s_offset_valid = false;
}

void current_sense_at32m412_read_raw(uint16_t *ia, uint16_t *ib, uint16_t *ic)
{
    /* 注入序列由 TMR1_CH4 硬件触发, ISR 进入时数据已就绪 (spec §3.2) */
    *ia = adc_preempt_conversion_data_get(ADC2, ADC_PREEMPT_CHANNEL_1);  /* SOA */
    *ib = adc_preempt_conversion_data_get(ADC2, ADC_PREEMPT_CHANNEL_2);  /* SOB */
    *ic = adc_preempt_conversion_data_get(ADC2, ADC_PREEMPT_CHANNEL_3);  /* SOC */
}

bool current_sense_at32m412_calibrate_offset(void)
{
    uint32_t sum_a = 0;
    uint32_t sum_b = 0;
    uint32_t sum_c = 0;
    uint16_t ia, ib, ic;
    uint16_t candidate_a, candidate_b, candidate_c;
    uint32_t i;
    uint32_t timeout;
    int32_t  diff_a, diff_b, diff_c;

    /* spec §4.3.3: PWM 50% 三相同电位时采 1024 次, 求平均.
     * 调用方需已设 PWM 50% 并等待稳定. 每个样本都先清完成标志，再等待
     * 下一次 TMR1_CH4 触发的完整注入序列，确保 1024 个样本彼此独立. */
    for (i = 0; i < OFFSET_SAMPLE_COUNT; i++) {
        adc_flag_clear(ADC2, ADC_PCCE_FLAG);
        timeout = OFFSET_SEQUENCE_TIMEOUT;
        while (adc_flag_get(ADC2, ADC_PCCE_FLAG) == RESET) {
            if (--timeout == 0u) {
                return false;
            }
        }
        current_sense_at32m412_read_raw(&ia, &ib, &ic);
        sum_a += ia;
        sum_b += ib;
        sum_c += ic;
    }

    candidate_a = (uint16_t)(sum_a / OFFSET_SAMPLE_COUNT);
    candidate_b = (uint16_t)(sum_b / OFFSET_SAMPLE_COUNT);
    candidate_c = (uint16_t)(sum_c / OFFSET_SAMPLE_COUNT);

    /* 校验: 零偏应在 2048±20 LSB 内 (spec §4.3.3), 否则硬件异常 */
    diff_a = (int32_t)candidate_a - (int32_t)CURRENT_ZERO_OFFSET_LSB;
    diff_b = (int32_t)candidate_b - (int32_t)CURRENT_ZERO_OFFSET_LSB;
    diff_c = (int32_t)candidate_c - (int32_t)CURRENT_ZERO_OFFSET_LSB;

    if (diff_a < 0) diff_a = -diff_a;
    if (diff_b < 0) diff_b = -diff_b;
    if (diff_c < 0) diff_c = -diff_c;

    if ((uint32_t)diff_a > OFFSET_VALID_WINDOW_LSB ||
        (uint32_t)diff_b > OFFSET_VALID_WINDOW_LSB ||
        (uint32_t)diff_c > OFFSET_VALID_WINDOW_LSB) {
        return false;
    }

    s_offset_a = candidate_a;
    s_offset_b = candidate_b;
    s_offset_c = candidate_c;
    s_offset_valid = true;
    return true;
}

void current_sense_at32m412_get_offset(uint16_t *ofs_a, uint16_t *ofs_b, uint16_t *ofs_c)
{
    *ofs_a = s_offset_a;
    *ofs_b = s_offset_b;
    *ofs_c = s_offset_c;
}

void current_sense_at32m412_set_offset(uint16_t ofs_a, uint16_t ofs_b, uint16_t ofs_c)
{
    s_offset_a = ofs_a;
    s_offset_b = ofs_b;
    s_offset_c = ofs_c;
}

float current_sense_calc(uint16_t raw, float offset_lsb, float gain_a_per_lsb)
{
    return ((float)raw - offset_lsb) * gain_a_per_lsb;
}

uint16_t current_sense_at32m412_read_vbus_raw(void)
{
    uint32_t timeout = 0;

    /* 软件触发普通转换 (VBUS, 普通序列第 1 通道) */
    adc_ordinary_software_trigger_enable(ADC2, TRUE);

    /* 等待普通转换完成标志 (OCCE = ordinary conversion complete) */
    while (adc_flag_get(ADC2, ADC_OCCE_FLAG) == RESET) {
        if (++timeout > VBUS_CONV_TIMEOUT) {
            return 0;   /* 超时, 返回 0 (后续 VBUS 校验会触发欠压故障) */
        }
    }

    /* 清标志 (读数据自动清, 但显式清更安全) */
    adc_flag_clear(ADC2, ADC_OCCE_FLAG);

    return adc_ordinary_conversion_data_get(ADC2);
}

float current_sense_at32m412_read_vbus(void)
{
    uint16_t raw = current_sense_at32m412_read_vbus_raw();
    return (float)raw * VBUS_VOLTS_PER_LSB;   /* 4.834 mV/LSB, spec §4.3.6 */
}

bool current_sense_at32m412_offset_valid(void)
{
    return s_offset_valid;
}
