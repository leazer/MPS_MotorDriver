/* tests/foc_core/test_foc_clarke.c */
#include "foc_core.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static int approx_eq(float a, float b, float eps)
{
    return fabsf(a - b) < eps;
}

int main(void)
{
    float ia, ib, ic, ialpha, ibeta;

    /* 测试 1: 三相平衡正序, ia=1A, ib=-0.5A, ic=-0.5A -> ialpha=1, ibeta=0 */
    ia = 1.0f; ib = -0.5f; ic = -0.5f;
    foc_clarke(ia, ib, ic, &ialpha, &ibeta);
    assert(approx_eq(ialpha, 1.0f, 1e-4f));
    assert(approx_eq(ibeta, 0.0f, 1e-4f));

    /* 测试 2: 全零 -> 全零 */
    foc_clarke(0, 0, 0, &ialpha, &ibeta);
    assert(approx_eq(ialpha, 0.0f, 1e-6f));
    assert(approx_eq(ibeta, 0.0f, 1e-6f));

    /* 测试 3: ia=0, ib=sqrt(3)/2, ic=-sqrt(3)/2 -> ialpha=0, ibeta=1 */
    ia = 0.0f; ib = 0.8660254f; ic = -0.8660254f;
    foc_clarke(ia, ib, ic, &ialpha, &ibeta);
    assert(approx_eq(ialpha, 0.0f, 1e-4f));
    assert(approx_eq(ibeta, 1.0f, 1e-4f));

    printf("test_foc_clarke: 3 tests passed\n");
    return 0;
}
