#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "current_loop.h"
#include "motor_params.h"
#include "motor_tuning.h"
#include "position_loop.h"
#include "speed_loop.h"

static float position_run_one_millisecond(int32_t sensor_mdeg)
{
    float output;
    unsigned i;

    output = 0.0f;
    for (i = 0u; i < POSITION_LOOP_DIV; ++i) {
        output = position_loop_run(sensor_mdeg);
    }
    return output;
}

static void test_defaults_are_loaded(void)
{
    motor_tuning_init();

    assert(g_motor_tuning.current.id_kp == PID_ID_KP);
    assert(g_motor_tuning.current.id_ki == PID_ID_KI);
    assert(g_motor_tuning.current.iq_kp == PID_IQ_KP);
    assert(g_motor_tuning.current.iq_ki == PID_IQ_KI);
    assert(g_motor_tuning.current.id_integral_limit_v ==
           PID_CURRENT_INTEGRAL_LIMIT);
    assert(g_motor_tuning.current.iq_integral_limit_v ==
           PID_CURRENT_INTEGRAL_LIMIT);
    assert(g_motor_tuning.current.id_output_limit_v ==
           PID_CURRENT_OUT_LIMIT);
    assert(g_motor_tuning.current.iq_output_limit_v ==
           PID_CURRENT_OUT_LIMIT);

    assert(g_motor_tuning.speed.kp == PID_SPEED_KP);
    assert(g_motor_tuning.speed.kp_brake == PID_SPEED_KP_BRAKE);
    assert(g_motor_tuning.speed.ki == PID_SPEED_KI);
    assert(g_motor_tuning.speed.integral_limit_A ==
           PID_SPEED_INTEGRAL_LIMIT);
    assert(g_motor_tuning.speed.output_limit_A == PID_SPEED_OUT_LIMIT);
    assert(g_motor_tuning.speed.friction_A == SPEED_IQ_FRICTION_A);
    assert(g_motor_tuning.speed.ramp_rad_s2 == 50.0f);

    assert(g_motor_tuning.position.kp == PID_POSITION_KP);
    assert(g_motor_tuning.position.speed_limit_elec_rad_s ==
           POSITION_SPEED_LIMIT_ELEC_RAD_S);
    assert(g_motor_tuning.position.max_velocity_mdeg_s ==
           POSITION_MAX_VELOCITY_MDEG_S);
    assert(g_motor_tuning.position.max_error_mdeg ==
           POSITION_MAX_ERROR_MDEG);
    assert(g_motor_tuning.position.command_limit_mdeg ==
           POSITION_COMMAND_LIMIT_MDEG);
    assert(g_motor_tuning.position.extrapolation_limit_ms ==
           POSITION_EXTRAPOLATION_LIMIT_MS);
    assert(g_motor_tuning.position.iq_friction_A ==
           POSITION_IQ_FRICTION_A);
    assert(g_motor_tuning.position.iq_friction_moving_A ==
           POSITION_IQ_FRICTION_MOVING_A);
    assert(g_motor_tuning.position.iq_friction_error_mdeg ==
           POSITION_IQ_FRICTION_ERROR_MDEG);

    assert(g_motor_tuning.protection.phase_overcurrent_A ==
           IQ_OVERCURRENT_A);
    assert(g_motor_tuning.protection.overcurrent_debounce_ticks ==
           OVERCURRENT_DEBOUNCE_TICKS);
    assert(g_motor_tuning.protection.imbalance_threshold_A ==
           IMBALANCE_THRESHOLD_A);
    assert(g_motor_tuning.protection.imbalance_debounce_ticks ==
           IMBALANCE_DEBOUNCE_TICKS);
    assert(g_motor_tuning.protection.sample_blanking_ticks ==
           CURRENT_SAMPLE_BLANKING_TICKS);
    assert(g_motor_tuning.protection.sample_invalid_limit ==
           CURRENT_SAMPLE_INVALID_LIMIT);
}

static void test_current_loop_reads_changed_ram_gain(void)
{
    float vd;
    float vq;

    motor_tuning_init();
    current_loop_init();
    g_motor_tuning.current.id_kp = 2.0f;
    g_motor_tuning.current.id_ki = 0.0f;
    g_motor_tuning.current.id_integral_limit_v = 10.0f;
    g_motor_tuning.current.id_output_limit_v = 10.0f;
    current_loop_set_targets(1.0f, 0.0f);

    current_loop_run(0.0f, 0.0f, &vd, &vq);

    assert(fabsf(vd - 2.0f) < 1.0e-6f);
    assert(fabsf(vq) < 1.0e-6f);
    assert(g_motor_loop_debug.current.id_error_A == 1.0f);
    assert(g_motor_loop_debug.current.id_integral_v == 0.0f);
    assert(g_motor_loop_debug.current.vd_output_v == vd);
    assert(g_motor_loop_debug.current.id_saturated == 0u);
}

static void test_speed_loop_reads_changed_ram_gain_and_limit(void)
{
    float iq;
    unsigned i;

    motor_tuning_init();
    speed_loop_init();
    g_motor_tuning.speed.kp = 0.1f;
    g_motor_tuning.speed.kp_brake = 0.1f;
    g_motor_tuning.speed.ki = 0.0f;
    g_motor_tuning.speed.integral_limit_A = 10.0f;
    g_motor_tuning.speed.output_limit_A = 10.0f;
    g_motor_tuning.speed.friction_A = 0.0f;
    g_motor_tuning.speed.ramp_rad_s2 = 100000.0f;
    speed_loop_set_target_rad_s(10.0f);

    iq = 0.0f;
    for (i = 0u; i < 16u; ++i) {
        iq = speed_loop_run(0.0f);
    }

    assert(fabsf(iq - 1.0f) < 1.0e-5f);
    assert(fabsf(g_motor_loop_debug.speed.error_rad_s - 10.0f) <
           1.0e-5f);
    assert(g_motor_loop_debug.speed.active_kp == 0.1f);
    assert(g_motor_loop_debug.speed.iq_output_A == iq);
    assert(g_motor_loop_debug.speed.saturated == 0u);
}

static void test_position_loop_reads_changed_ram_gain(void)
{
    position_setpoint_t setpoint;
    float first_speed_ref;
    float second_speed_ref;

    motor_tuning_init();
    position_loop_init();
    position_loop_set_origin(0, 0);
    g_motor_tuning.position.kp = 1.0f;
    g_motor_tuning.position.speed_limit_elec_rad_s = 1000.0f;
    g_motor_tuning.position.max_error_mdeg = 100000;
    setpoint.position_mdeg = 1000;
    setpoint.velocity_mdeg_s = 0;
    setpoint.sequence = 1u;
    setpoint.lease_ms = 0u;
    assert(position_loop_submit(&setpoint));

    first_speed_ref = position_run_one_millisecond(0);
    g_motor_tuning.position.kp = 2.0f;
    second_speed_ref = position_run_one_millisecond(0);

    assert(second_speed_ref > first_speed_ref);
    assert(g_motor_loop_debug.position.error_mdeg == 1000);
    assert(g_motor_loop_debug.position.proportional_velocity_mdeg_s ==
           2000.0f);
    assert(g_motor_loop_debug.position.speed_output_elec_rad_s ==
           second_speed_ref);
}

int main(void)
{
    test_defaults_are_loaded();
    test_current_loop_reads_changed_ram_gain();
    test_speed_loop_reads_changed_ram_gain_and_limit();
    test_position_loop_reads_changed_ram_gain();
    printf("motor tuning tests passed\n");
    return 0;
}
