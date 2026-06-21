#ifndef FOC_CORE_H
#define FOC_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Clarke 变换: 三相电流 -> alpha/beta */
void foc_clarke(float ia, float ib, float ic, float *i_alpha, float *i_beta);

/* Park 变换: alpha/beta -> d/q (输入电角度 theta_e, 弧度) */
void foc_park(float i_alpha, float i_beta, float theta_e, float *id, float *iq);

/* 逆 Park 变换: d/q -> alpha/beta */
void foc_ipark(float vd, float vq, float theta_e, float *v_alpha, float *v_beta);

/* SVPWM 7 段法, 三相高边特化 (spec §4.2)
 * v_alpha, v_beta: 电压分量 (V)
 * vbus: 母线电压 (V)
 * ta, tb, tc: 输出占空比 ticks (0..TMR1_ARR, 已限幅 95%)
 */
void foc_svpwm_3phase_high_side(float v_alpha, float v_beta, float vbus,
                                 uint16_t *ta, uint16_t *tb, uint16_t *tc);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CORE_H */
