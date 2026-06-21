/* tests/current_sense/test_current_sense_calc.c */
#include "current_sense_at32m412.h"
#include "motor_params.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static int approx_eq(float a, float b, float eps) { return fabsf(a - b) < eps; }

int main(void)
{
    /* 零电流: raw=2048, offset=2048 -> 0 A */
    float i = current_sense_calc(CURRENT_ZERO_OFFSET_LSB, (float)CURRENT_ZERO_OFFSET_LSB, CURRENT_GAIN_DEFAULT_A_PER_LSB);
    assert(approx_eq(i, 0.0f, 1e-6f));

    /* 满量程正向: raw=4096, offset=2048, gain=3.16mA/LSB -> +6.48 A */
    i = current_sense_calc(4096, 2048.0f, CURRENT_GAIN_DEFAULT_A_PER_LSB);
    assert(approx_eq(i, 2048.0f * CURRENT_GAIN_DEFAULT_A_PER_LSB, 1e-3f));

    /* 负向: raw=0, offset=2048 -> -6.48 A */
    i = current_sense_calc(0, 2048.0f, CURRENT_GAIN_DEFAULT_A_PER_LSB);
    assert(approx_eq(i, -2048.0f * CURRENT_GAIN_DEFAULT_A_PER_LSB, 1e-3f));

    printf("test_current_sense_calc: 3 tests passed\n");
    return 0;
}
