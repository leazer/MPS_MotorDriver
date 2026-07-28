#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "motor_params.h"
#include "motor_tuning.h"
#include "position_loop.h"

static float run_one_millisecond(int32_t sensor_mdeg)
{
    float output;
    unsigned i;

    output = 0.0f;
    for (i = 0u; i < POSITION_LOOP_DIV; ++i) {
        output = position_loop_run(sensor_mdeg);
    }
    return output;
}

static void run_milliseconds(int32_t sensor_mdeg, unsigned milliseconds)
{
    unsigned i;
    for (i = 0u; i < milliseconds; ++i) {
        (void)run_one_millisecond(sensor_mdeg);
    }
}

static position_setpoint_t point(int32_t position_mdeg,
                                 int32_t velocity_mdeg_s,
                                 uint16_t sequence,
                                 uint16_t lease_ms)
{
    position_setpoint_t value;
    value.position_mdeg = position_mdeg;
    value.velocity_mdeg_s = velocity_mdeg_s;
    value.sequence = sequence;
    value.lease_ms = lease_ms;
    return value;
}

static position_loop_snapshot_t snapshot(void)
{
    position_loop_snapshot_t value;
    assert(position_loop_get_snapshot(&value));
    return value;
}

static void test_origin_and_first_setpoint(void)
{
    position_setpoint_t setpoint;
    position_loop_snapshot_t snap;
    float output;

    setpoint = point(30000, 10000, 1u, 100u);
    position_loop_init();
    assert(!position_loop_origin_valid());
    assert(!position_loop_submit(&setpoint));
    position_loop_set_origin(120000, 25000);
    assert(position_loop_origin_valid());
    assert(position_loop_sensor_to_joint_mdeg(120000) == 25000);
    assert(position_loop_sensor_to_joint_mdeg(121000) == 26000);
    assert(position_loop_submit(&setpoint));

    output = run_one_millisecond(120000);
    snap = snapshot();
    assert(snap.active == 1u);
    assert(snap.target_position_mdeg == 30000);
    assert(snap.velocity_ff_mdeg_s == 10000);
    assert(snap.reference_position_mdeg == 30000);
    assert(snap.measured_position_mdeg == 25000);
    assert(snap.error_mdeg == 5000);
    assert(snap.speed_ref_elec_mrad_s > 0);
    assert(output > 0.0f);
}

static void test_negative_joint_direction_end_to_end(void)
{
    position_setpoint_t setpoint;
    position_loop_snapshot_t snap;
    float output;

    position_loop_init();
    assert(position_loop_set_joint_origin(100000, 20000, -1));
    assert(position_loop_joint_direction() == -1);
    assert(position_loop_sensor_to_joint_mdeg(100000) == 20000);
    assert(position_loop_sensor_to_joint_mdeg(101000) == 19000);
    assert(position_loop_sensor_to_joint_mdeg(99000) == 21000);
    assert(position_loop_control_to_joint_velocity_mdeg_s(5000) == -5000);
    assert(position_loop_control_to_joint_velocity_mdeg_s(INT32_MIN) ==
           INT32_MAX);

    /* Correct current joint position is 0; offset-only math would produce
     * 40000 and falsely accept the 31000 target as a 9000 mdeg move. */
    assert(position_loop_first_target_safe(120000, 30000));
    assert(!position_loop_first_target_safe(120000, 31000));

    setpoint = point(21000, 0, 1u, 0u);
    assert(position_loop_submit(&setpoint));
    output = run_one_millisecond(100000);
    snap = snapshot();
    assert(snap.measured_position_mdeg == 20000);
    assert(snap.error_mdeg == 1000);
    assert(output < 0.0f); /* positive joint error => negative control speed */

    setpoint = point(20000, 10000, 2u, 100u);
    assert(position_loop_submit(&setpoint));
    output = run_one_millisecond(100000);
    snap = snapshot();
    assert(snap.velocity_ff_mdeg_s == 10000);
    assert(output < 0.0f); /* positive joint velocity => negative control speed */

    setpoint = point(20000, -10000, 3u, 100u);
    assert(position_loop_submit(&setpoint));
    output = run_one_millisecond(100000);
    assert(output > 0.0f);

    assert(!position_loop_set_joint_origin(0, 0, 0));
    assert(position_loop_joint_direction() == -1);

    assert(position_loop_set_joint_origin(INT32_MIN, INT32_MAX, 1));
    assert(position_loop_sensor_to_joint_mdeg(INT32_MIN) == INT32_MAX);
    assert(position_loop_set_joint_origin(INT32_MAX, INT32_MIN, -1));
    assert(position_loop_sensor_to_joint_mdeg(INT32_MAX) == INT32_MIN);
}

static void test_extrapolation_limit_and_timeout_hold(void)
{
    position_setpoint_t setpoint;
    position_loop_snapshot_t snap;
    int32_t frozen_reference;

    position_loop_init();
    position_loop_set_origin(0, 0);
    setpoint = point(0, 10000, 2u, 100u);
    assert(position_loop_submit(&setpoint));
    (void)run_one_millisecond(0);

    run_milliseconds(0, 10u);
    snap = snapshot();
    assert(snap.reference_position_mdeg == 100);
    assert(snap.age_ms == 10u);

    run_milliseconds(0, 10u);
    snap = snapshot();
    assert(snap.reference_position_mdeg == 200);
    run_milliseconds(0, 30u);
    snap = snapshot();
    assert(snap.reference_position_mdeg == 200);
    assert(snap.stream_timeout == 0u);

    run_milliseconds(0, 50u);
    snap = snapshot();
    assert(snap.age_ms == 100u);
    assert(snap.stream_timeout == 1u);
    assert(snap.velocity_ff_mdeg_s == 0);
    frozen_reference = snap.reference_position_mdeg;
    run_milliseconds(0, 20u);
    snap = snapshot();
    assert(snap.reference_position_mdeg == frozen_reference);

    setpoint = point(300, -5000, 3u, 100u);
    assert(position_loop_submit(&setpoint));
    (void)run_one_millisecond(0);
    snap = snapshot();
    assert(snap.stream_timeout == 0u);
    assert(snap.velocity_ff_mdeg_s == -5000);
    assert(snap.reference_position_mdeg == 300);
}

static void test_sequence_rules_and_wrap(void)
{
    position_setpoint_t setpoint;

    position_loop_init();
    position_loop_set_origin(0, 0);
    setpoint = point(0, 0, 10u, 0u);
    assert(position_loop_submit(&setpoint));
    assert(position_loop_submit(&setpoint));
    setpoint.position_mdeg = 1;
    assert(!position_loop_submit(&setpoint));
    setpoint.position_mdeg = 0;
    setpoint.sequence = 9u;
    assert(!position_loop_submit(&setpoint));
    setpoint.sequence = (uint16_t)(10u + 0x8000u);
    assert(!position_loop_submit(&setpoint));
    setpoint.sequence = 11u;
    assert(position_loop_submit(&setpoint));

    position_loop_init();
    position_loop_set_origin(0, 0);
    setpoint.sequence = 65535u;
    assert(position_loop_submit(&setpoint));
    setpoint.sequence = 0u;
    assert(position_loop_submit(&setpoint));
}

static void test_static_hold_never_times_out(void)
{
    position_setpoint_t setpoint;
    position_loop_snapshot_t snap;

    position_loop_init();
    position_loop_set_origin(0, 0);
    setpoint = point(1000, 0, 1u, 0u);
    assert(position_loop_submit(&setpoint));
    (void)run_one_millisecond(0);
    run_milliseconds(0, 250u);
    snap = snapshot();
    assert(snap.stream_timeout == 0u);
    assert(snap.reference_position_mdeg == 1000);
}

static void test_sign_limit_fault_and_reset(void)
{
    position_setpoint_t setpoint;
    position_loop_snapshot_t snap;
    float saved_speed_limit;
    float output;

    position_loop_init();
    position_loop_set_origin(0, 0);
    setpoint = point(5000, 0, 1u, 0u);
    assert(position_loop_submit(&setpoint));
    output = run_one_millisecond(0);
    assert(output > 0.0f);

    setpoint = point(-5000, 0, 2u, 0u);
    assert(position_loop_submit(&setpoint));
    output = run_one_millisecond(0);
    assert(output < 0.0f);

    saved_speed_limit =
        g_motor_tuning.position.speed_limit_elec_rad_s;
    g_motor_tuning.position.speed_limit_elec_rad_s = 10.0f;
    setpoint = point(29999, POSITION_MAX_VELOCITY_MDEG_S, 3u, 0u);
    assert(position_loop_submit(&setpoint));
    output = run_one_millisecond(0);
    assert(fabsf(output - 10.0f) < 1.0e-5f);
    g_motor_tuning.position.speed_limit_elec_rad_s =
        saved_speed_limit;

    setpoint = point(POSITION_MAX_ERROR_MDEG + 1, 0, 4u, 0u);
    assert(position_loop_submit(&setpoint));
    output = run_one_millisecond(0);
    snap = snapshot();
    assert(output == 0.0f);
    assert(snap.tracking_fault == 1u);

    position_loop_reset();
    assert(position_loop_origin_valid());
    snap = snapshot();
    assert(snap.active == 0u);
    assert(snap.tracking_fault == 0u);
    position_loop_init();
    assert(!position_loop_origin_valid());
}

static void test_position_friction_feedforward_and_deadband(void)
{
    position_setpoint_t setpoint;

    position_loop_init();
    position_loop_set_origin(0, 0);

    setpoint = point(5000, 0, 1u, 0u);
    assert(position_loop_submit(&setpoint));
    (void)run_one_millisecond(0);
    assert(fabsf(position_loop_get_iq_feedforward_A() -
                 POSITION_IQ_FRICTION_A) < 1.0e-6f);

    setpoint = point(POSITION_IQ_FRICTION_ERROR_MDEG, 0, 2u, 0u);
    assert(position_loop_submit(&setpoint));
    (void)run_one_millisecond(0);
    assert(position_loop_get_iq_feedforward_A() == 0.0f);

    setpoint = point(0, 10000, 3u, 100u);
    assert(position_loop_submit(&setpoint));
    (void)run_one_millisecond(0);
    assert(fabsf(position_loop_get_iq_feedforward_A() -
                 POSITION_IQ_FRICTION_MOVING_A) < 1.0e-6f);

    setpoint = point(-5000, 0, 4u, 0u);
    assert(position_loop_submit(&setpoint));
    (void)run_one_millisecond(0);
    assert(fabsf(position_loop_get_iq_feedforward_A() +
                 POSITION_IQ_FRICTION_A) < 1.0e-6f);
}

int main(void)
{
    motor_tuning_init();
    test_origin_and_first_setpoint();
    test_negative_joint_direction_end_to_end();
    test_extrapolation_limit_and_timeout_hold();
    test_sequence_rules_and_wrap();
    test_static_hold_never_times_out();
    test_sign_limit_fault_and_reset();
    test_position_friction_feedforward_and_deadband();
    printf("position loop tests passed\n");
    return 0;
}
