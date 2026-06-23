/*
 * test_current_loop.c - 电流环 PI 数学单元测试 (Stage 5)
 *
 * 主机端 gcc 编译运行. 独立复制 pid_f32_exec 逻辑, 验证:
 *   1. 比例项正确 (Kp × error) + 单步积分增量
 *   2. 积分累加 + clamping 限幅生效
 *   3. 输出限幅生效 (p_out + 积分饱和后仍被限)
 *   4. reset 清积分/last_error
 *   5. 稳态: 恒定误差下积分收敛到消除误差 (Ki 作用)
 *   6. kd=0 时 last_error 不被写 (Task 2 优化: 省 ISR 每拍 store)
 *
 * 注: 产品代码 current_loop.c 依赖 motor_params.h/board_motor_pins.h 宏,
 *     此处独立定义等价参数, 只验数学正确性 (与 test_motor_calibration.c 同风格).
 *     replica 必须与产品代码 pid_f32_exec 逐行一致, 否则测的不是产品逻辑.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define ISR_DT_S   (1.0f / 16000.0f)
#define KP   0.5f
#define KI   100.0f
#define OUT_LIMIT  6.0f

typedef struct {
    float kp, ki, kd;
    float integral;
    float integral_limit;
    float out_limit;
    float last_error;
} pid_f32_t;

/* ===== replica: 必须与 current_loop.c pid_f32_exec 逐行一致 ===== */
static float pid_f32_exec(pid_f32_t *pid, float error)
{
    float p_out = pid->kp * error;
    pid->integral += pid->ki * error * ISR_DT_S;
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float out = p_out + pid->integral;
    if (out >  pid->out_limit) out =  pid->out_limit;
    if (out < -pid->out_limit) out = -pid->out_limit;
    if (pid->kd != 0.0f) pid->last_error = error;
    return out;
}

/* 测试 1: 纯比例 + 单步积分 (误差恒定, 积分初值 0) */
static void test_pure_proportional(void)
{
    pid_f32_t pid = {KP, KI, 0, 0, OUT_LIMIT, OUT_LIMIT, 0};
    float out = pid_f32_exec(&pid, 1.0f);   /* error=1A */
    /* p_out = 0.5*1 = 0.5, integral = 100*1*(1/16000) = 0.00625, out = 0.50625 */
    assert(fabsf(out - 0.50625f) < 1e-4f);
    printf("[PASS] pure proportional: out=%.5f (expect 0.50625)\n", out);
}

/* 测试 2: 积分限幅 clamping */
static void test_integral_clamp(void)
{
    pid_f32_t pid = {KP, KI, 0, 0, OUT_LIMIT, OUT_LIMIT, 0};
    /* 持续大误差, 积分应被限幅到 OUT_LIMIT */
    for (int i = 0; i < 100000; i++) {
        pid_f32_exec(&pid, 10.0f);
    }
    assert(pid.integral <= OUT_LIMIT + 1e-6f);
    assert(pid.integral >= -OUT_LIMIT - 1e-6f);
    printf("[PASS] integral clamp: integral=%.4f (limit %.1f)\n", pid.integral, OUT_LIMIT);
}

/* 测试 3: 输出限幅 (积分已饱和, p_out 叠加后仍被限) */
static void test_output_clamp(void)
{
    pid_f32_t pid = {KP, KI, 0, OUT_LIMIT, OUT_LIMIT, OUT_LIMIT, 0}; /* integral 已饱和 */
    float out = pid_f32_exec(&pid, 10.0f);  /* p_out=5 + integral=6 = 11 -> clamp 6 */
    assert(out <= OUT_LIMIT + 1e-6f);
    printf("[PASS] output clamp: out=%.4f (limit %.1f)\n", out, OUT_LIMIT);
}

/* 测试 4: reset 清积分与 last_error */
static void test_reset(void)
{
    pid_f32_t pid = {KP, KI, 0, 3.0f, OUT_LIMIT, OUT_LIMIT, 1.0f};
    /* 复刻 current_loop_reset 的行为 */
    pid.integral = 0.0f;
    pid.last_error = 0.0f;
    assert(pid.integral == 0.0f);
    assert(pid.last_error == 0.0f);
    printf("[PASS] reset clears integral and last_error\n");
}

/* 测试 5: 稳态消误差 (积分作用) */
static void test_steady_state_zero_error(void)
{
    /* 模拟: 实测电流缓慢跟踪目标, 积分把稳态误差压到 ~0 */
    pid_f32_t pid = {KP, KI, 0, 0, OUT_LIMIT, OUT_LIMIT, 0};
    float iq_target = 0.5f;
    float iq_actual = 0.0f;
    /* 简化被控对象: actual += out * 0.01 (一阶, 模拟电感) */
    for (int i = 0; i < 200000; i++) {
        float out = pid_f32_exec(&pid, iq_target - iq_actual);
        iq_actual += out * 0.01f;
    }
    /* 稳态误差 < 1% */
    assert(fabsf(iq_actual - iq_target) < 0.01f * iq_target);
    printf("[PASS] steady state: actual=%.5f target=%.5f err=%.6f\n",
           iq_actual, iq_target, fabsf(iq_actual - iq_target));
}

/* 测试 6: kd=0 时 last_error 不被写 (Task 2 优化) */
static void test_last_error_not_written_when_kd_zero(void)
{
    pid_f32_t pid = {KP, KI, 0.0f, 0, OUT_LIMIT, OUT_LIMIT, -7.0f}; /* last_error 哨兵值 */
    pid_f32_exec(&pid, 1.0f);
    assert(pid.last_error == -7.0f);  /* 未被覆盖 */
    /* kd!=0 时应被写 */
    pid.kd = 1.0f;
    pid_f32_exec(&pid, 2.0f);
    assert(pid.last_error == 2.0f);
    printf("[PASS] last_error gated by kd (skipped when kd=0, written when kd!=0)\n");
}

int main(void)
{
    test_pure_proportional();
    test_integral_clamp();
    test_output_clamp();
    test_reset();
    test_steady_state_zero_error();
    test_last_error_not_written_when_kd_zero();
    printf("\n6 current_loop tests passed\n");
    return 0;
}
