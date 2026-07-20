#ifndef CURRENT_SENSE_AT32M412_H
#define CURRENT_SENSE_AT32M412_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* current_sense_at32m412 - ADC2 注入序列 + TMR1_CH4 低边窗口触发 (spec §3.2/§4.3)
 *
 * 硬件: MP6540H 电流镜, SOA/SOB/SOC -> ADC2_IN9/8/7, VBUS -> ADC2_IN6
 *   - 注入序列 (preempt): 3 通道 [SOA, SOB, SOC], 由 TMR1_CH4 比较事件触发
 *   - 普通序列 (ordinary): 1 通道 [VBUS], 软件触发读取
 *   - ADC_CLK = 180MHz / 6 = 30MHz, 12-bit, 采样时间 1.5 cycle
 *
 * 采样原理: TMR1_CH4 比较匹配触发 ADC 注入转换, FOC ISR 读取上次转换结果.
 *   MP6540H 电流镜需要在有效导通窗口采样; 触发点由 PWM 模块设置并可用
 *   pwm_adc_trig 临时调整.
 */

/* ADC2 注入序列初始化 (GPIO + ADC2 base + preempt 3ch + TMR1_CH4 触发 + 校准) */
void current_sense_at32m412_init(void);

/* 读取三相电流 ADC 原始值 (FOC ISR 内调用, 注入序列已由硬件触发完成) */
void current_sense_at32m412_read_raw(uint16_t *ia, uint16_t *ib, uint16_t *ic);

/* 零偏标定: PWM 50% 时采 1024 次平均 (spec §4.3.3).
 * 返回 true 成功, false 失败 (转换超时或零偏离 2048 超过窗口).
 * 失败时保留此前的零偏和有效状态，不发布不完整的候选值.
 * 调用前需确保 PWM 已输出 50% 三相同电位且 MP6540H 已使能. */
bool current_sense_at32m412_calibrate_offset(void);

/* 获取零偏标定结果 (LSB) */
void current_sense_at32m412_get_offset(uint16_t *ofs_a, uint16_t *ofs_b, uint16_t *ofs_c);

/* 设置零偏 (供调试/恢复默认用) */
void current_sense_at32m412_set_offset(uint16_t ofs_a, uint16_t ofs_b, uint16_t ofs_c);

/* raw -> 安培 (spec §4.3.4) */
float current_sense_calc(uint16_t raw, float offset_lsb, float gain_a_per_lsb);

/* 读取 VBUS 母线电压 (软件触发普通转换, 非阻塞轮询).
 * 返回电压 (V). 调用频率不限, 典型 1kHz. */
float current_sense_at32m412_read_vbus(void);

/* 读取 VBUS ADC 原始值 (LSB) */
uint16_t current_sense_at32m412_read_vbus_raw(void);

/* 零偏标定是否已完成 */
bool current_sense_at32m412_offset_valid(void);

#ifdef __cplusplus
}
#endif

#endif /* CURRENT_SENSE_AT32M412_H */
