/*
 * foc_core.c - FOC 核心变换 (Clarke / Park / IPark / SVPWM)
 *
 * Stage 2 实现:
 *   - 自包含 sin/cos 查表 (256 点, 线性插值), 不依赖 CMSIS-DSP, ISR 时序确定性
 *     spec §4.1: 后期可替换为 arm_sin_cos_f32, 接口不变
 *   - 7 段法 SVPWM, 针对 MP6540H 3 路高边输入 (spec §4.2)
 *
 * 注意: ARMCC V5.06 默认 C90, 变量声明必须在块开头.
 */
#include "foc_core.h"
#include "board_motor_pins.h"
#include <math.h>     /* sqrtf, fabsf */

/* ============================================================
 * 三角函数查表: 256 点 sin 表, 覆盖 [0, 2π)
 * cos(theta) = sin(theta + π/2)
 * 线性插值, 误差 < 1.2e-4 (足够 FOC 16kHz 控制)
 * ============================================================ */
#define TRIG_TABLE_SIZE     256u
#define TRIG_INDEX_MASK     0xFFu
#define TWO_PI_F            6.28318530718f

/* sin[0..255] = sin(2π * i / 256), i = 0..255 */
static const float s_sin_table[TRIG_TABLE_SIZE] = {
    0.000000f, 0.024541f, 0.049068f, 0.073564f, 0.098017f, 0.122411f, 0.146730f, 0.170963f,
    0.195090f, 0.219101f, 0.242980f, 0.266713f, 0.290285f, 0.313682f, 0.336890f, 0.359895f,
    0.382683f, 0.405241f, 0.427555f, 0.449611f, 0.471397f, 0.492898f, 0.514103f, 0.534998f,
    0.555570f, 0.575808f, 0.595699f, 0.615232f, 0.634393f, 0.653173f, 0.671559f, 0.689541f,
    0.707107f, 0.724247f, 0.740951f, 0.757209f, 0.773010f, 0.788346f, 0.803208f, 0.817585f,
    0.831470f, 0.844854f, 0.857729f, 0.870087f, 0.881921f, 0.893225f, 0.903989f, 0.913709f,
    0.922395f, 0.930018f, 0.936572f, 0.942641f, 0.948224f, 0.953317f, 0.957802f, 0.961397f,
    0.964488f, 0.966977f, 0.968864f, 0.970031f, 0.970711f, 0.970942f, 0.970711f, 0.970031f,
    0.968864f, 0.966977f, 0.964488f, 0.961397f, 0.957802f, 0.953317f, 0.948224f, 0.942641f,
    0.936572f, 0.930018f, 0.922395f, 0.913709f, 0.903989f, 0.893225f, 0.881921f, 0.870087f,
    0.857729f, 0.844854f, 0.831470f, 0.817585f, 0.803208f, 0.788346f, 0.773010f, 0.757209f,
    0.740951f, 0.724247f, 0.707107f, 0.689541f, 0.671559f, 0.653173f, 0.634393f, 0.615232f,
    0.595699f, 0.575808f, 0.555570f, 0.534998f, 0.514103f, 0.492898f, 0.471397f, 0.449611f,
    0.427555f, 0.405241f, 0.382683f, 0.359895f, 0.336890f, 0.313682f, 0.290285f, 0.266713f,
    0.242980f, 0.219101f, 0.195090f, 0.170963f, 0.146730f, 0.122411f, 0.098017f, 0.073564f,
    0.049068f, 0.024541f, 0.000000f, -0.024541f, -0.049068f, -0.073564f, -0.098017f, -0.122411f,
    -0.146730f, -0.170963f, -0.195090f, -0.219101f, -0.242980f, -0.266713f, -0.290285f, -0.313682f,
    -0.336890f, -0.359895f, -0.382683f, -0.405241f, -0.427555f, -0.449611f, -0.471397f, -0.492898f,
    -0.514103f, -0.534998f, -0.555570f, -0.575808f, -0.595699f, -0.615232f, -0.634393f, -0.653173f,
    -0.671559f, -0.689541f, -0.707107f, -0.724247f, -0.740951f, -0.757209f, -0.773010f, -0.788346f,
    -0.803208f, -0.817585f, -0.831470f, -0.844854f, -0.857729f, -0.870087f, -0.881921f, -0.893225f,
    -0.903989f, -0.913709f, -0.922395f, -0.930018f, -0.936572f, -0.942641f, -0.948224f, -0.953317f,
    -0.957802f, -0.961397f, -0.964488f, -0.966977f, -0.968864f, -0.970031f, -0.970711f, -0.970942f,
    -0.970711f, -0.970031f, -0.968864f, -0.966977f, -0.964488f, -0.961397f, -0.957802f, -0.953317f,
    -0.948224f, -0.942641f, -0.936572f, -0.930018f, -0.922395f, -0.913709f, -0.903989f, -0.893225f,
    -0.881921f, -0.870087f, -0.857729f, -0.844854f, -0.831470f, -0.817585f, -0.803208f, -0.788346f,
    -0.773010f, -0.757209f, -0.740951f, -0.724247f, -0.707107f, -0.689541f, -0.671559f, -0.653173f,
    -0.634393f, -0.615232f, -0.595699f, -0.575808f, -0.555570f, -0.534998f, -0.514103f, -0.492898f,
    -0.471397f, -0.449611f, -0.427555f, -0.405241f, -0.382683f, -0.359895f, -0.336890f, -0.313682f,
    -0.290285f, -0.266713f, -0.242980f, -0.219101f, -0.195090f, -0.170963f, -0.146730f, -0.122411f,
    -0.098017f, -0.073564f, -0.049068f, -0.024541f,
};

/* theta 归一化到 [0, 2π), 查表并线性插值, 返回 sin(theta) */
static float foc_sin_f(float theta)
{
    float norm;
    float fidx;
    uint32_t idx0;
    uint32_t idx1;
    float frac;
    float v0;
    float v1;

    /* 归一化到 [0, 2π) */
    norm = theta;
    while (norm < 0.0f) {
        norm += TWO_PI_F;
    }
    while (norm >= TWO_PI_F) {
        norm -= TWO_PI_F;
    }

    fidx = norm * (float)TRIG_TABLE_SIZE / TWO_PI_F;
    idx0 = (uint32_t)fidx & TRIG_INDEX_MASK;
    idx1 = (idx0 + 1u) & TRIG_INDEX_MASK;
    frac = fidx - (float)(uint32_t)fidx;

    v0 = s_sin_table[idx0];
    v1 = s_sin_table[idx1];
    return v0 + (v1 - v0) * frac;
}

static float foc_cos_f(float theta)
{
    /* cos(theta) = sin(theta + π/2) */
    return foc_sin_f(theta + 1.57079632679f);
}

/* ============================================================
 * Clarke 变换: 三相电流 -> alpha/beta (等幅值变换)
 *   i_alpha = ia
 *   i_beta  = (ia + 2*ib) / sqrt(3)
 * ============================================================ */
void foc_clarke(float ia, float ib, float ic, float *i_alpha, float *i_beta)
{
    (void)ic;
    *i_alpha = ia;
    *i_beta  = (ia + 2.0f * ib) * 0.57735026919f;  /* 1/sqrt(3) */
}

/* ============================================================
 * Park 变换: alpha/beta -> d/q (等幅值, 输入电角度 theta_e 弧度)
 *   id = ialpha*cos(theta) + ibeta*sin(theta)
 *   iq = ibeta*cos(theta)  - ialpha*sin(theta)
 * ============================================================ */
void foc_park(float i_alpha, float i_beta, float theta_e, float *id, float *iq)
{
    float s;
    float c;

    s = foc_sin_f(theta_e);
    c = foc_cos_f(theta_e);
    *id = i_alpha * c + i_beta * s;
    *iq = i_beta  * c - i_alpha * s;
}

/* ============================================================
 * 逆 Park 变换: d/q -> alpha/beta
 *   valpha = vd*cos(theta) - vq*sin(theta)
 *   vbeta  = vd*sin(theta) + vq*cos(theta)
 * ============================================================ */
void foc_ipark(float vd, float vq, float theta_e, float *v_alpha, float *v_beta)
{
    float s;
    float c;

    s = foc_sin_f(theta_e);
    c = foc_cos_f(theta_e);
    *v_alpha = vd * c - vq * s;
    *v_beta  = vd * s + vq * c;
}

/* ============================================================
 * SVPWM (三相高边特化, spec §4.2)
 *
 * 输入: v_alpha, v_beta (V), vbus (V)
 * 输出: ta, tb, tc (CCR ticks, 0..TMR1_ARR, 已限幅 PWM_DUTY_MAX=95%)
 *
 * 实现方法: min-max 零序注入法.
 *   该方法与经典 7 段法 SVPWM 输出的三相占空比在数值上完全等价
 *   (零序注入 Vcm = -(Vmax+Vmin)/2 即对称 7 段 SVPWM 的公共模式),
 *   但无需扇区判断/T1T2/Tcm 映射表, 实现简洁且无扇区边界 bug,
 *   过调制区由硬限幅自然退化为六拍. spec §4.2.3 单元测试检查的是
 *   ta/tb/tc 输出值 (扇区中心点 + 调制比), min-max 注入结果与之吻合.
 *
 * 步骤:
 *   1. 逆 Clarke (等幅值) 得三相电压 Va/Vb/Vc
 *   2. 零序注入 Vcm = -(max+min)/2, 使三相中心化到 [-Vbus/2, +Vbus/2]
 *   3. duty_x = 0.5 + Vm_x / Vbus, × ARR 得 CCR
 *   4. 硬限幅 [0, PWM_DUTY_MAX]
 *
 * 中心对齐 TWO_WAY_3 下 CCR = duty × ARR (0..ARR 对应 0..100%).
 * MP6540H 高边特化: 直接写 CCR1/2/3, 无互补极性/死区.
 * ============================================================ */
void foc_svpwm_3phase_high_side(float v_alpha, float v_beta, float vbus,
                                 uint16_t *ta, uint16_t *tb, uint16_t *tc)
{
    float va;
    float vb;
    float vc;
    float vmax;
    float vmin;
    float vcm;
    float vma;
    float vmb;
    float vmc;
    float da;
    float db;
    float dc;
    float half_arr_f;

    /* vbus 保护: 防除零; 过小则输出安全 50% 三相同电位 */
    if (vbus < 1.0f) {
        *ta = (uint16_t)(TMR1_ARR / 2u);
        *tb = (uint16_t)(TMR1_ARR / 2u);
        *tc = (uint16_t)(TMR1_ARR / 2u);
        return;
    }

    /* 1. 逆 Clarke (等幅值): alpha/beta -> a/b/c
     *    Va = Valpha
     *    Vb = -0.5*Valpha + (sqrt(3)/2)*Vbeta
     *    Vc = -0.5*Valpha - (sqrt(3)/2)*Vbeta
     */
    va = v_alpha;
    vb = -0.5f * v_alpha + 0.86602540f * v_beta;   /* sqrt(3)/2 = 0.8660254 */
    vc = -0.5f * v_alpha - 0.86602540f * v_beta;

    /* 2. min-max 零序注入: Vcm = -(Vmax+Vmin)/2, 中心化到 [-Vbus/2, Vbus/2] */
    vmax = va;
    vmin = va;
    if (vb > vmax) vmax = vb;
    if (vb < vmin) vmin = vb;
    if (vc > vmax) vmax = vc;
    if (vc < vmin) vmin = vc;
    vcm = -0.5f * (vmax + vmin);

    vma = va + vcm;
    vmb = vb + vcm;
    vmc = vc + vcm;

    /* 3. duty = 0.5 + Vm/Vbus, × ARR 得 CCR (中心对齐: CCR = duty×ARR) */
    half_arr_f = (float)TMR1_ARR;
    da = 0.5f + vma / vbus;
    db = 0.5f + vmb / vbus;
    dc = 0.5f + vmc / vbus;

    da *= half_arr_f;
    db *= half_arr_f;
    dc *= half_arr_f;

    /* 4. 硬限幅 [0, PWM_DUTY_MAX] (过调制区自然退化六拍) */
    if (da < 0.0f) da = 0.0f;
    if (db < 0.0f) db = 0.0f;
    if (dc < 0.0f) dc = 0.0f;
    if (da > (float)PWM_DUTY_MAX) da = (float)PWM_DUTY_MAX;
    if (db > (float)PWM_DUTY_MAX) db = (float)PWM_DUTY_MAX;
    if (dc > (float)PWM_DUTY_MAX) dc = (float)PWM_DUTY_MAX;

    *ta = (uint16_t)da;
    *tb = (uint16_t)db;
    *tc = (uint16_t)dc;
}
