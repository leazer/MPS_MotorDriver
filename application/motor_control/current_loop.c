/*
 * current_loop.c - Id/Iq 双轴 PI 电流环 (Stage 5, spec-stage5 §4)
 *
 * 标准 PI + clamping 抗 windup:
 *   - 积分先限幅 (integral_limit), 再加比例, 再输出限幅 (out_limit)
 *   - ki 含 1/s 量纲, 积分时 × ISR_DT_S 转 per-tick 增量
 *   - kd=0 (Stage 5 不用微分, 结构体字段保留)
 *
 * ISR 安全: 无 RT-Thread API, 无 malloc, 纯浮点运算 (~1µs).
 *
 * 增益参数复用 motor_params.h 中既有的 PID_ID_KP/KI、PID_IQ_KP/KI
 * (值与 spec §4.6 一致: Kp=0.5 V/A, Ki=100 V/(A·s)), 不另建并行宏.
 */
#include "current_loop.h"
#include "motor_params.h"
#include "board_motor_pins.h"   /* PWM_FREQUENCY_HZ: 与 motor_control_isr.c 同一来源 */
#include <string.h>

/* ISR 周期. 与 motor_control_isr.c:27 同一定义式, 单一来源 PWM_FREQUENCY_HZ.
 * 不用 #ifndef 守护: 该宏仅在 .c 内可见, 守护无跨 TU 保护作用, 反而掩盖重复. */
#define ISR_DT_S   (1.0f / (float)PWM_FREQUENCY_HZ)

static pid_f32_t s_pid_d;
static pid_f32_t s_pid_q;
/* 目标值: shell/CAN (线程) 写, ISR 读 -> volatile (同 motor_control_isr.c 约定).
 * PID 积分由 ISR 累加, 仅在模式切换 (CURRENT 非激活) 时由 reset 清零, 不需 volatile. */
static volatile float s_id_ref = 0.0f;
static volatile float s_iq_ref = 0.0f;

float pid_f32_exec(pid_f32_t *pid, float error)
{
    float p_out;
    float out;

    p_out = pid->kp * error;
    pid->integral += pid->ki * error * ISR_DT_S;   /* 离散积分 */
    /* 积分限幅 (clamping 抗 windup) */
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    out = p_out + pid->integral;
    /* 输出限幅 */
    if (out >  pid->out_limit) out =  pid->out_limit;
    if (out < -pid->out_limit) out = -pid->out_limit;
    /* last_error 仅微分项用; kd=0 时不写, 省 ISR 每拍一次 store */
    if (pid->kd != 0.0f) pid->last_error = error;
    return out;
}

void current_loop_init(void)
{
    memset(&s_pid_d, 0, sizeof(s_pid_d));
    memset(&s_pid_q, 0, sizeof(s_pid_q));
    s_pid_d.kp = PID_ID_KP;  s_pid_d.ki = PID_ID_KI;  s_pid_d.kd = 0.0f;
    s_pid_d.integral_limit = PID_CURRENT_INTEGRAL_LIMIT;  s_pid_d.out_limit = PID_CURRENT_OUT_LIMIT;
    s_pid_q.kp = PID_IQ_KP;  s_pid_q.ki = PID_IQ_KI;  s_pid_q.kd = 0.0f;
    s_pid_q.integral_limit = PID_CURRENT_INTEGRAL_LIMIT;  s_pid_q.out_limit = PID_CURRENT_OUT_LIMIT;
    s_id_ref = 0.0f;
    s_iq_ref = 0.0f;
}

void current_loop_set_targets(float id_ref_A, float iq_ref_A)
{
    s_id_ref = id_ref_A;
    s_iq_ref = iq_ref_A;
}

void current_loop_reset(void)
{
    s_pid_d.integral = 0.0f;
    s_pid_q.integral = 0.0f;
    s_pid_d.last_error = 0.0f;
    s_pid_q.last_error = 0.0f;
}

void current_loop_run(float id, float iq, float *vd_ref, float *vq_ref)
{
    *vd_ref = pid_f32_exec(&s_pid_d, s_id_ref - id);
    *vq_ref = pid_f32_exec(&s_pid_q, s_iq_ref - iq);
}

float current_loop_get_id_ref_A(void)
{
    return s_id_ref;
}

float current_loop_get_iq_ref_A(void)
{
    return s_iq_ref;
}
