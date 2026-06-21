#ifndef CURRENT_SENSE_AT32M412_H
#define CURRENT_SENSE_AT32M412_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ADC2 注入序列初始化 (TMR1_CH4 顶点触发, spec §3.2) */
void current_sense_at32m412_init(void);

/* 读取三相电流 ADC 原始值 (FOC ISR 内调用) */
void current_sense_at32m412_read_raw(uint16_t *ia, uint16_t *ib, uint16_t *ic);

/* 零偏标定 (PWM 50% 时采 1024 次平均, spec §4.3.3) */
void current_sense_at32m412_calibrate_offset(void);

/* raw -> 安培 (spec §4.3.4) */
float current_sense_calc(uint16_t raw, float offset_lsb, float gain_a_per_lsb);

#ifdef __cplusplus
}
#endif

#endif /* CURRENT_SENSE_AT32M412_H */
