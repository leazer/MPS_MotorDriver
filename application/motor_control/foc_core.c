#include "foc_core.h"
#include "board_motor_pins.h"

void foc_clarke(float ia, float ib, float ic, float *i_alpha, float *i_beta)
{
    (void)ic;
    *i_alpha = ia;
    *i_beta = (ia + 2.0f * ib) * 0.57735026919f; /* 1/sqrt(3) */
}

void foc_park(float i_alpha, float i_beta, float theta_e, float *id, float *iq)
{
    (void)i_alpha; (void)i_beta; (void)theta_e;
    *id = 0.0f;
    *iq = 0.0f;
}

void foc_ipark(float vd, float vq, float theta_e, float *v_alpha, float *v_beta)
{
    (void)vd; (void)vq; (void)theta_e;
    *v_alpha = 0.0f;
    *v_beta = 0.0f;
}

void foc_svpwm_3phase_high_side(float v_alpha, float v_beta, float vbus,
                                 uint16_t *ta, uint16_t *tb, uint16_t *tc)
{
    (void)v_alpha; (void)v_beta; (void)vbus;
    /* stub: 50% 占空比, 三相同电位, 不出力 */
    *ta = (uint16_t)(TMR1_ARR / 2u);
    *tb = (uint16_t)(TMR1_ARR / 2u);
    *tc = (uint16_t)(TMR1_ARR / 2u);
}
