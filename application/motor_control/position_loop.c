#include "position_loop.h"
#include "motor_params.h"
#include "motor_tuning.h"
#include <limits.h>
#include <string.h>

#if defined(__CC_ARM) || defined(__arm__) || defined(__thumb__)
#include "at32m412_416.h"
#else
#include <stdatomic.h>
#endif

#define POSITION_MDEG_TO_RAD (3.14159265359f / 180000.0f)

#if !defined(__CC_ARM) && !defined(__arm__) && !defined(__thumb__)
static atomic_flag s_position_origin_host_lock = ATOMIC_FLAG_INIT;
#endif

static uint32_t position_loop_origin_lock(void)
{
#if defined(__CC_ARM) || defined(__arm__) || defined(__thumb__)
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    return primask;
#else
    while (atomic_flag_test_and_set_explicit(&s_position_origin_host_lock,
                                             memory_order_acquire)) {
    }
    return 0u;
#endif
}

static void position_loop_origin_unlock(uint32_t primask)
{
#if defined(__CC_ARM) || defined(__arm__) || defined(__thumb__)
    __DMB();
    if (primask == 0u) {
        __enable_irq();
    }
#else
    (void)primask;
    atomic_flag_clear_explicit(&s_position_origin_host_lock,
                               memory_order_release);
#endif
}

static volatile uint32_t s_publish_generation;
static volatile int32_t s_publish_position_mdeg;
static volatile int32_t s_publish_velocity_mdeg_s;
static volatile uint16_t s_publish_sequence;
static volatile uint16_t s_publish_lease_ms;
static uint32_t s_consumed_generation;
static uint16_t s_last_submitted_sequence;
static uint8_t s_has_submitted_sequence;

static volatile int32_t s_sensor_anchor_mdeg;
static volatile int32_t s_joint_anchor_mdeg;
static volatile int8_t s_joint_direction;
static volatile uint8_t s_origin_valid;

static position_setpoint_t s_active_setpoint;
static position_loop_snapshot_t s_position_snapshot;
static volatile uint32_t s_snapshot_generation;
static int32_t s_frozen_reference_mdeg;
static float s_speed_ref_elec_rad_s;
static float s_iq_feedforward_A;
static uint8_t s_divider;

static int32_t position_saturate_i64(int64_t value)
{
    if (value > (int64_t)INT32_MAX) {
        return INT32_MAX;
    }
    if (value < (int64_t)INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)value;
}

static float position_clamp(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static bool position_sequence_newer(uint16_t candidate, uint16_t previous)
{
    uint16_t delta;
    delta = (uint16_t)(candidate - previous);
    return delta != 0u && delta < 0x8000u;
}

static uint32_t position_snapshot_begin(void)
{
    uint32_t generation;

    generation = s_snapshot_generation;
    if ((generation & 1u) != 0u) {
        generation++;
    }
    s_snapshot_generation = generation + 1u;
    return generation;
}

static void position_snapshot_end(uint32_t generation)
{
    s_position_snapshot.origin_valid = s_origin_valid;
    s_snapshot_generation = generation + 2u;
}

static bool position_consume_setpoint(position_setpoint_t *out)
{
    uint32_t generation_before;
    uint32_t generation_after;

    generation_before = s_publish_generation;
    if ((generation_before & 1u) != 0u ||
        generation_before == s_consumed_generation) {
        return false;
    }
    out->position_mdeg = s_publish_position_mdeg;
    out->velocity_mdeg_s = s_publish_velocity_mdeg_s;
    out->sequence = s_publish_sequence;
    out->lease_ms = s_publish_lease_ms;
    generation_after = s_publish_generation;
    if (generation_before != generation_after ||
        (generation_after & 1u) != 0u) {
        return false;
    }
    s_consumed_generation = generation_after;
    return true;
}

void position_loop_init(void)
{
    uint32_t primask;

    position_loop_reset();
    primask = position_loop_origin_lock();
    s_sensor_anchor_mdeg = 0;
    s_joint_anchor_mdeg = 0;
    s_joint_direction = 1;
    s_origin_valid = 0u;
    s_position_snapshot.origin_valid = 0u;
    position_loop_origin_unlock(primask);
}

void position_loop_reset(void)
{
    uint8_t origin_valid;

    origin_valid = s_origin_valid;
    s_publish_generation = 0u;
    s_publish_position_mdeg = 0;
    s_publish_velocity_mdeg_s = 0;
    s_publish_sequence = 0u;
    s_publish_lease_ms = 0u;
    s_consumed_generation = 0u;
    s_last_submitted_sequence = 0u;
    s_has_submitted_sequence = 0u;
    memset(&s_active_setpoint, 0, sizeof(s_active_setpoint));
    memset(&s_position_snapshot, 0, sizeof(s_position_snapshot));
    s_snapshot_generation = 0u;
    s_frozen_reference_mdeg = 0;
    s_speed_ref_elec_rad_s = 0.0f;
    s_iq_feedforward_A = 0.0f;
    s_divider = 0u;
    s_position_snapshot.origin_valid = origin_valid;
    memset((void *)&g_motor_loop_debug.position, 0,
           sizeof(g_motor_loop_debug.position));
}

bool position_loop_set_joint_origin(int32_t sensor_mdeg, int32_t joint_mdeg,
                                    int8_t joint_direction)
{
    uint32_t primask;

    if (joint_direction != 1 && joint_direction != -1) {
        return false;
    }

    position_loop_reset();
    primask = position_loop_origin_lock();
    s_sensor_anchor_mdeg = sensor_mdeg;
    s_joint_anchor_mdeg = joint_mdeg;
    s_joint_direction = joint_direction;
    s_origin_valid = 1u;
    s_position_snapshot.origin_valid = 1u;
    position_loop_origin_unlock(primask);
    return true;
}

void position_loop_set_origin(int32_t sensor_mdeg, int32_t joint_mdeg)
{
    (void)position_loop_set_joint_origin(sensor_mdeg, joint_mdeg, 1);
}

int8_t position_loop_joint_direction(void)
{
    return s_joint_direction;
}

bool position_loop_origin_valid(void)
{
    return s_origin_valid != 0u;
}

int32_t position_loop_sensor_to_joint_mdeg(int32_t sensor_mdeg)
{
    return position_saturate_i64(
        (int64_t)s_joint_anchor_mdeg +
        ((int64_t)s_joint_direction *
         ((int64_t)sensor_mdeg - (int64_t)s_sensor_anchor_mdeg)));
}

int32_t position_loop_control_to_joint_velocity_mdeg_s(
    int32_t control_velocity_mdeg_s)
{
    return position_saturate_i64((int64_t)s_joint_direction *
                                 (int64_t)control_velocity_mdeg_s);
}

bool position_loop_first_target_safe(int32_t sensor_mdeg,
                                     int32_t target_joint_mdeg)
{
    int32_t current_joint_mdeg;
    int64_t error_mdeg;

    if (s_origin_valid == 0u) {
        return false;
    }
    current_joint_mdeg = position_loop_sensor_to_joint_mdeg(sensor_mdeg);
    error_mdeg = (int64_t)target_joint_mdeg - (int64_t)current_joint_mdeg;
    return error_mdeg >= -(int64_t)g_motor_tuning.position.max_error_mdeg &&
           error_mdeg <= (int64_t)g_motor_tuning.position.max_error_mdeg;
}

bool position_loop_submit(const position_setpoint_t *setpoint)
{
    uint32_t generation;

    if (setpoint == 0 || s_origin_valid == 0u) {
        return false;
    }
    if (setpoint->velocity_mdeg_s >
            g_motor_tuning.position.max_velocity_mdeg_s ||
        setpoint->velocity_mdeg_s <
            -g_motor_tuning.position.max_velocity_mdeg_s) {
        return false;
    }
    if (s_has_submitted_sequence != 0u) {
        if (setpoint->sequence == s_last_submitted_sequence) {
            return setpoint->position_mdeg == s_publish_position_mdeg &&
                   setpoint->velocity_mdeg_s == s_publish_velocity_mdeg_s &&
                   setpoint->lease_ms == s_publish_lease_ms;
        }
        if (!position_sequence_newer(setpoint->sequence,
                                     s_last_submitted_sequence)) {
            return false;
        }
    }

    generation = s_publish_generation;
    if ((generation & 1u) != 0u) {
        generation++;
    }
    s_publish_generation = generation + 1u;
    s_publish_position_mdeg = setpoint->position_mdeg;
    s_publish_velocity_mdeg_s = setpoint->velocity_mdeg_s;
    s_publish_sequence = setpoint->sequence;
    s_publish_lease_ms = setpoint->lease_ms;
    s_publish_generation = generation + 2u;
    s_last_submitted_sequence = setpoint->sequence;
    s_has_submitted_sequence = 1u;
    return true;
}

float position_loop_run(int32_t sensor_mdeg)
{
    position_setpoint_t received;
    uint16_t extrapolation_ms;
    int32_t measured_mdeg;
    int64_t reference_i64;
    int64_t error_i64;
    int32_t velocity_ff_mdeg_s;
    float proportional_velocity_mdeg_s;
    float speed_mech_rad_s;
    float speed_unlimited_elec_rad_s;
    uint32_t snapshot_generation;

    s_divider++;
    if (s_divider < POSITION_LOOP_DIV) {
        return s_speed_ref_elec_rad_s;
    }
    s_divider = 0u;
    snapshot_generation = position_snapshot_begin();

    if (position_consume_setpoint(&received)) {
        s_active_setpoint = received;
        s_position_snapshot.active = 1u;
        s_position_snapshot.stream_timeout = 0u;
        s_position_snapshot.tracking_fault = 0u;
        s_position_snapshot.age_ms = 0u;
        s_frozen_reference_mdeg = received.position_mdeg;
    } else if (s_position_snapshot.active != 0u &&
               s_position_snapshot.age_ms < UINT16_MAX) {
        s_position_snapshot.age_ms++;
    }

    if (s_position_snapshot.active == 0u) {
        s_speed_ref_elec_rad_s = 0.0f;
        s_iq_feedforward_A = 0.0f;
        g_motor_loop_debug.position.speed_unlimited_elec_rad_s = 0.0f;
        g_motor_loop_debug.position.speed_output_elec_rad_s = 0.0f;
        g_motor_loop_debug.position.iq_feedforward_A = 0.0f;
        position_snapshot_end(snapshot_generation);
        return 0.0f;
    }

    extrapolation_ms = s_position_snapshot.age_ms;
    if (extrapolation_ms >
        g_motor_tuning.position.extrapolation_limit_ms) {
        extrapolation_ms =
            g_motor_tuning.position.extrapolation_limit_ms;
    }
    if (s_position_snapshot.stream_timeout == 0u) {
        reference_i64 = (int64_t)s_active_setpoint.position_mdeg +
            ((int64_t)s_active_setpoint.velocity_mdeg_s *
             (int64_t)extrapolation_ms) / 1000LL;
        s_frozen_reference_mdeg = position_saturate_i64(reference_i64);
    }
    if (s_active_setpoint.lease_ms != 0u &&
        s_position_snapshot.age_ms >= s_active_setpoint.lease_ms) {
        s_position_snapshot.stream_timeout = 1u;
    }

    velocity_ff_mdeg_s = s_position_snapshot.stream_timeout != 0u ?
        0 : s_active_setpoint.velocity_mdeg_s;
    measured_mdeg = position_loop_sensor_to_joint_mdeg(sensor_mdeg);
    error_i64 = (int64_t)s_frozen_reference_mdeg - (int64_t)measured_mdeg;

    s_position_snapshot.target_position_mdeg =
        s_active_setpoint.position_mdeg;
    s_position_snapshot.velocity_ff_mdeg_s = velocity_ff_mdeg_s;
    s_position_snapshot.reference_position_mdeg =
        s_frozen_reference_mdeg;
    s_position_snapshot.measured_position_mdeg = measured_mdeg;
    s_position_snapshot.error_mdeg = position_saturate_i64(error_i64);
    s_position_snapshot.sequence = s_active_setpoint.sequence;
    proportional_velocity_mdeg_s =
        g_motor_tuning.position.kp *
        (float)s_position_snapshot.error_mdeg;
    speed_mech_rad_s =
        ((float)velocity_ff_mdeg_s + proportional_velocity_mdeg_s) *
        POSITION_MDEG_TO_RAD;
    speed_unlimited_elec_rad_s =
        speed_mech_rad_s * (float)s_joint_direction *
        (float)MOTOR_POLE_PAIRS;

    if (error_i64 > g_motor_tuning.position.max_error_mdeg ||
        error_i64 < -g_motor_tuning.position.max_error_mdeg) {
        s_position_snapshot.tracking_fault = 1u;
        s_speed_ref_elec_rad_s = 0.0f;
        s_iq_feedforward_A = 0.0f;
    } else {
        s_speed_ref_elec_rad_s = position_clamp(
            speed_unlimited_elec_rad_s,
            g_motor_tuning.position.speed_limit_elec_rad_s);
        s_iq_feedforward_A = 0.0f;
        if (s_speed_ref_elec_rad_s != 0.0f) {
            if (velocity_ff_mdeg_s != 0) {
                s_iq_feedforward_A = s_speed_ref_elec_rad_s > 0.0f ?
                    g_motor_tuning.position.iq_friction_moving_A :
                    -g_motor_tuning.position.iq_friction_moving_A;
            } else if (
                s_position_snapshot.error_mdeg >
                    g_motor_tuning.position.iq_friction_error_mdeg ||
                s_position_snapshot.error_mdeg <
                    -g_motor_tuning.position.iq_friction_error_mdeg) {
                s_iq_feedforward_A = s_speed_ref_elec_rad_s > 0.0f ?
                    g_motor_tuning.position.iq_friction_A :
                    -g_motor_tuning.position.iq_friction_A;
            }
        }
    }
    s_position_snapshot.speed_ref_elec_mrad_s =
        (int32_t)(s_speed_ref_elec_rad_s * 1000.0f);
    g_motor_loop_debug.position.target_position_mdeg =
        s_position_snapshot.target_position_mdeg;
    g_motor_loop_debug.position.reference_position_mdeg =
        s_position_snapshot.reference_position_mdeg;
    g_motor_loop_debug.position.measured_position_mdeg =
        s_position_snapshot.measured_position_mdeg;
    g_motor_loop_debug.position.error_mdeg =
        s_position_snapshot.error_mdeg;
    g_motor_loop_debug.position.velocity_ff_mdeg_s =
        s_position_snapshot.velocity_ff_mdeg_s;
    g_motor_loop_debug.position.proportional_velocity_mdeg_s =
        proportional_velocity_mdeg_s;
    g_motor_loop_debug.position.speed_unlimited_elec_rad_s =
        speed_unlimited_elec_rad_s;
    g_motor_loop_debug.position.speed_output_elec_rad_s =
        s_speed_ref_elec_rad_s;
    g_motor_loop_debug.position.iq_feedforward_A =
        s_iq_feedforward_A;
    position_snapshot_end(snapshot_generation);
    return s_speed_ref_elec_rad_s;
}

float position_loop_get_iq_feedforward_A(void)
{
    return s_iq_feedforward_A;
}

bool position_loop_get_snapshot(position_loop_snapshot_t *out)
{
    uint32_t generation_before;
    uint32_t generation_after;

    if (out == 0) {
        return false;
    }
    for (;;) {
        generation_before = s_snapshot_generation;
        if ((generation_before & 1u) != 0u) {
            continue;
        }
        *out = s_position_snapshot;
        generation_after = s_snapshot_generation;
        if (generation_before == generation_after &&
            (generation_after & 1u) == 0u) {
            return true;
        }
    }
}
