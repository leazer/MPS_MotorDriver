#include "position_loop.h"
#include "motor_params.h"
#include <limits.h>
#include <string.h>

#define POSITION_MDEG_TO_RAD (3.14159265359f / 180000.0f)

static volatile uint32_t s_publish_generation;
static volatile int32_t s_publish_position_mdeg;
static volatile int32_t s_publish_velocity_mdeg_s;
static volatile uint16_t s_publish_sequence;
static volatile uint16_t s_publish_lease_ms;
static uint32_t s_consumed_generation;
static uint16_t s_last_submitted_sequence;
static uint8_t s_has_submitted_sequence;

static volatile int32_t s_joint_offset_mdeg;
static volatile uint8_t s_origin_valid;

static position_setpoint_t s_active_setpoint;
static position_loop_snapshot_t s_snapshot;
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
    s_snapshot.origin_valid = s_origin_valid;
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
    s_joint_offset_mdeg = 0;
    s_origin_valid = 0u;
    position_loop_reset();
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
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_snapshot_generation = 0u;
    s_frozen_reference_mdeg = 0;
    s_speed_ref_elec_rad_s = 0.0f;
    s_iq_feedforward_A = 0.0f;
    s_divider = 0u;
    s_snapshot.origin_valid = origin_valid;
}

void position_loop_set_origin(int32_t sensor_mdeg, int32_t joint_mdeg)
{
    position_loop_reset();
    s_joint_offset_mdeg = position_saturate_i64(
        (int64_t)joint_mdeg - (int64_t)sensor_mdeg);
    s_origin_valid = 1u;
    s_snapshot.origin_valid = 1u;
}

bool position_loop_origin_valid(void)
{
    return s_origin_valid != 0u;
}

int32_t position_loop_sensor_to_joint_mdeg(int32_t sensor_mdeg)
{
    return position_saturate_i64((int64_t)sensor_mdeg +
                                 (int64_t)s_joint_offset_mdeg);
}

bool position_loop_submit(const position_setpoint_t *setpoint)
{
    uint32_t generation;

    if (setpoint == 0 || s_origin_valid == 0u) {
        return false;
    }
    if (setpoint->velocity_mdeg_s > POSITION_MAX_VELOCITY_MDEG_S ||
        setpoint->velocity_mdeg_s < -POSITION_MAX_VELOCITY_MDEG_S) {
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
    float speed_mech_rad_s;
    uint32_t snapshot_generation;

    s_divider++;
    if (s_divider < POSITION_LOOP_DIV) {
        return s_speed_ref_elec_rad_s;
    }
    s_divider = 0u;
    snapshot_generation = position_snapshot_begin();

    if (position_consume_setpoint(&received)) {
        s_active_setpoint = received;
        s_snapshot.active = 1u;
        s_snapshot.stream_timeout = 0u;
        s_snapshot.tracking_fault = 0u;
        s_snapshot.age_ms = 0u;
        s_frozen_reference_mdeg = received.position_mdeg;
    } else if (s_snapshot.active != 0u && s_snapshot.age_ms < UINT16_MAX) {
        s_snapshot.age_ms++;
    }

    if (s_snapshot.active == 0u) {
        s_speed_ref_elec_rad_s = 0.0f;
        s_iq_feedforward_A = 0.0f;
        position_snapshot_end(snapshot_generation);
        return 0.0f;
    }

    extrapolation_ms = s_snapshot.age_ms;
    if (extrapolation_ms > POSITION_EXTRAPOLATION_LIMIT_MS) {
        extrapolation_ms = POSITION_EXTRAPOLATION_LIMIT_MS;
    }
    if (s_snapshot.stream_timeout == 0u) {
        reference_i64 = (int64_t)s_active_setpoint.position_mdeg +
            ((int64_t)s_active_setpoint.velocity_mdeg_s *
             (int64_t)extrapolation_ms) / 1000LL;
        s_frozen_reference_mdeg = position_saturate_i64(reference_i64);
    }
    if (s_active_setpoint.lease_ms != 0u &&
        s_snapshot.age_ms >= s_active_setpoint.lease_ms) {
        s_snapshot.stream_timeout = 1u;
    }

    velocity_ff_mdeg_s = s_snapshot.stream_timeout != 0u ?
        0 : s_active_setpoint.velocity_mdeg_s;
    measured_mdeg = position_loop_sensor_to_joint_mdeg(sensor_mdeg);
    error_i64 = (int64_t)s_frozen_reference_mdeg - (int64_t)measured_mdeg;

    s_snapshot.target_position_mdeg = s_active_setpoint.position_mdeg;
    s_snapshot.velocity_ff_mdeg_s = velocity_ff_mdeg_s;
    s_snapshot.reference_position_mdeg = s_frozen_reference_mdeg;
    s_snapshot.measured_position_mdeg = measured_mdeg;
    s_snapshot.error_mdeg = position_saturate_i64(error_i64);
    s_snapshot.sequence = s_active_setpoint.sequence;

    if (error_i64 > POSITION_MAX_ERROR_MDEG ||
        error_i64 < -POSITION_MAX_ERROR_MDEG) {
        s_snapshot.tracking_fault = 1u;
        s_speed_ref_elec_rad_s = 0.0f;
        s_iq_feedforward_A = 0.0f;
    } else {
        speed_mech_rad_s =
            ((float)velocity_ff_mdeg_s +
             PID_POSITION_KP * (float)s_snapshot.error_mdeg) *
            POSITION_MDEG_TO_RAD;
        s_speed_ref_elec_rad_s = position_clamp(
            speed_mech_rad_s * (float)MOTOR_POLE_PAIRS,
            POSITION_SPEED_LIMIT_ELEC_RAD_S);
        s_iq_feedforward_A = 0.0f;
        if (s_speed_ref_elec_rad_s != 0.0f) {
            if (velocity_ff_mdeg_s != 0) {
                s_iq_feedforward_A = s_speed_ref_elec_rad_s > 0.0f ?
                    POSITION_IQ_FRICTION_MOVING_A :
                    -POSITION_IQ_FRICTION_MOVING_A;
            } else if (
                s_snapshot.error_mdeg > POSITION_IQ_FRICTION_ERROR_MDEG ||
                s_snapshot.error_mdeg < -POSITION_IQ_FRICTION_ERROR_MDEG) {
                s_iq_feedforward_A = s_speed_ref_elec_rad_s > 0.0f ?
                    POSITION_IQ_FRICTION_A : -POSITION_IQ_FRICTION_A;
            }
        }
    }
    s_snapshot.speed_ref_elec_mrad_s =
        (int32_t)(s_speed_ref_elec_rad_s * 1000.0f);
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
        *out = s_snapshot;
        generation_after = s_snapshot_generation;
        if (generation_before == generation_after &&
            (generation_after & 1u) == 0u) {
            return true;
        }
    }
}
