/*
 * test_speed_loop.c - Stage 6 speed PI math unit tests.
 *
 * Host-side test mirrors speed_loop.c math so that tuning constants,
 * sign, ramp limiting, and output clamping stay explicit.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define ISR_DT_S                    (1.0f / 16000.0f)
#define SPEED_LOOP_DIV              16u
#define SPEED_DT_S                  (ISR_DT_S * (float)SPEED_LOOP_DIV)
#define PID_SPEED_KP                0.01f
#define PID_SPEED_KI                0.5f
#define SPEED_IQ_LIMIT_A            0.5f
#define PID_SPEED_INTEGRAL_LIMIT    SPEED_IQ_LIMIT_A
#define PID_SPEED_OUT_LIMIT         SPEED_IQ_LIMIT_A
#define SPEED_RAMP_RAD_S2           600.0f

typedef struct {
    float kp;
    float ki;
    float integral;
    float integral_limit;
    float out_limit;
} speed_pid_t;

static speed_pid_t s_pid;
static float s_target;
static float s_cmd;
static float s_measured;
static float s_iq_ref;

static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void speed_loop_init_replica(void)
{
    memset(&s_pid, 0, sizeof(s_pid));
    s_pid.kp = PID_SPEED_KP;
    s_pid.ki = PID_SPEED_KI;
    s_pid.integral_limit = PID_SPEED_INTEGRAL_LIMIT;
    s_pid.out_limit = PID_SPEED_OUT_LIMIT;
    s_target = 0.0f;
    s_cmd = 0.0f;
    s_measured = 0.0f;
    s_iq_ref = 0.0f;
}

static void speed_loop_set_target_replica(float target)
{
    s_target = target;
}

static float speed_loop_run_replica(float measured)
{
    float max_step;
    float delta;
    float error;
    float out;

    s_measured = measured;
    max_step = SPEED_RAMP_RAD_S2 * SPEED_DT_S;
    delta = clampf(s_target - s_cmd, -max_step, max_step);
    s_cmd += delta;

    error = s_cmd - s_measured;
    s_pid.integral += s_pid.ki * error * SPEED_DT_S;
    s_pid.integral = clampf(s_pid.integral, -s_pid.integral_limit, s_pid.integral_limit);
    out = s_pid.kp * error + s_pid.integral;
    s_iq_ref = clampf(out, -s_pid.out_limit, s_pid.out_limit);
    return s_iq_ref;
}

static void test_positive_speed_error_outputs_positive_iq(void)
{
    float iq;
    speed_loop_init_replica();
    speed_loop_set_target_replica(10.0f);
    iq = speed_loop_run_replica(0.0f);
    assert(iq > 0.0f);
    printf("[PASS] positive speed error outputs positive iq: %.5f\n", iq);
}

static void test_negative_speed_error_outputs_negative_iq(void)
{
    float iq;
    speed_loop_init_replica();
    speed_loop_set_target_replica(-10.0f);
    iq = speed_loop_run_replica(0.0f);
    assert(iq < 0.0f);
    printf("[PASS] negative speed error outputs negative iq: %.5f\n", iq);
}

static void test_output_clamps_to_iq_limit(void)
{
    float iq;
    int i;
    speed_loop_init_replica();
    speed_loop_set_target_replica(10000.0f);
    iq = 0.0f;
    for (i = 0; i < 20000; i++) {
        iq = speed_loop_run_replica(0.0f);
    }
    assert(fabsf(iq - SPEED_IQ_LIMIT_A) < 1e-5f);
    assert(fabsf(s_pid.integral) <= PID_SPEED_INTEGRAL_LIMIT + 1e-5f);
    printf("[PASS] output clamps to iq limit: %.4f\n", iq);
}

static void test_negative_output_clamps_to_iq_limit(void)
{
    float iq;
    int i;
    speed_loop_init_replica();
    speed_loop_set_target_replica(-10000.0f);
    iq = 0.0f;
    for (i = 0; i < 20000; i++) {
        iq = speed_loop_run_replica(0.0f);
    }
    assert(fabsf(iq + SPEED_IQ_LIMIT_A) < 1e-5f);
    assert(fabsf(s_pid.integral) <= PID_SPEED_INTEGRAL_LIMIT + 1e-5f);
    printf("[PASS] negative output clamps to iq limit: %.4f\n", iq);
}

static void test_target_ramp_limits_first_step(void)
{
    float expected_step;
    speed_loop_init_replica();
    speed_loop_set_target_replica(100.0f);
    speed_loop_run_replica(0.0f);
    expected_step = SPEED_RAMP_RAD_S2 * SPEED_DT_S;
    assert(fabsf(s_cmd - expected_step) < 1e-6f);
    printf("[PASS] target ramp first step: %.5f rad/s\n", s_cmd);
}

static void test_zero_target_resets_command_toward_zero(void)
{
    int i;
    speed_loop_init_replica();
    speed_loop_set_target_replica(100.0f);
    for (i = 0; i < 20; i++) {
        speed_loop_run_replica(0.0f);
    }
    assert(s_cmd > 0.0f);
    speed_loop_set_target_replica(0.0f);
    speed_loop_run_replica(0.0f);
    assert(s_cmd < 100.0f);
    printf("[PASS] zero target ramps command down: %.5f\n", s_cmd);
}

int main(void)
{
    test_positive_speed_error_outputs_positive_iq();
    test_negative_speed_error_outputs_negative_iq();
    test_output_clamps_to_iq_limit();
    test_negative_output_clamps_to_iq_limit();
    test_target_ramp_limits_first_step();
    test_zero_target_resets_command_toward_zero();
    printf("\n6 speed_loop tests passed\n");
    return 0;
}
