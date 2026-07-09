/*
 * speed_loop.c - Stage 6 speed PI loop.
 *
 * Runs from the 16kHz FOC ISR, but updates PI at 1kHz by internal divider.
 * Output is Iq_ref (A) for the existing current loop.
 */
#include "speed_loop.h"
#include "board_motor_pins.h"
#include "motor_params.h"
#include <string.h>

#define SPEED_LOOP_DIV          16u
#define SPEED_LOOP_DT_S         ((float)SPEED_LOOP_DIV / (float)PWM_FREQUENCY_HZ)
#define SPEED_RAMP_RAD_S2       600.0f

static pid_f32_t s_pid;
static volatile float s_target_rad_s;
static volatile float s_command_rad_s;
static volatile float s_measured_rad_s;
static volatile float s_iq_ref_A;
static uint8_t s_divider;

static float speed_loop_clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void speed_loop_init(void)
{
    memset(&s_pid, 0, sizeof(s_pid));
    s_pid.kp = PID_SPEED_KP;
    s_pid.ki = PID_SPEED_KI;
    s_pid.kd = 0.0f;
    s_pid.integral_limit = PID_SPEED_INTEGRAL_LIMIT;
    s_pid.out_limit = PID_SPEED_OUT_LIMIT;
    speed_loop_reset();
}

void speed_loop_reset(void)
{
    s_pid.integral = 0.0f;
    s_pid.last_error = 0.0f;
    s_target_rad_s = 0.0f;
    s_command_rad_s = 0.0f;
    s_measured_rad_s = 0.0f;
    s_iq_ref_A = 0.0f;
    s_divider = 0u;
}

void speed_loop_set_target_rad_s(float target_rad_s)
{
    float max_speed;

    max_speed = (float)RPM_MAX * 6.28318530718f / 60.0f;
    s_target_rad_s = speed_loop_clamp(target_rad_s, -max_speed, max_speed);
}

float speed_loop_run(float measured_rad_s)
{
    float max_step;
    float delta;
    float error;
    float out;

    s_measured_rad_s = measured_rad_s;
    s_divider++;
    if (s_divider < SPEED_LOOP_DIV) {
        return s_iq_ref_A;
    }
    s_divider = 0u;

    max_step = SPEED_RAMP_RAD_S2 * SPEED_LOOP_DT_S;
    delta = speed_loop_clamp(s_target_rad_s - s_command_rad_s, -max_step, max_step);
    s_command_rad_s += delta;

    error = s_command_rad_s - s_measured_rad_s;
    s_pid.integral += s_pid.ki * error * SPEED_LOOP_DT_S;
    s_pid.integral = speed_loop_clamp(s_pid.integral,
                                      -s_pid.integral_limit,
                                      s_pid.integral_limit);
    out = s_pid.kp * error + s_pid.integral;
    s_iq_ref_A = speed_loop_clamp(out, -s_pid.out_limit, s_pid.out_limit);
    return s_iq_ref_A;
}

float speed_loop_get_target_rad_s(void)
{
    return s_target_rad_s;
}

float speed_loop_get_command_rad_s(void)
{
    return s_command_rad_s;
}

float speed_loop_get_measured_rad_s(void)
{
    return s_measured_rad_s;
}

float speed_loop_get_iq_ref_A(void)
{
    return s_iq_ref_A;
}
