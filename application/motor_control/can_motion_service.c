#include "can_motion_service.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#include "fault_manager.h"
#include "motor_params.h"

#define CAN_MOTION_RX_BATCH_MAX       4u
#define CAN_MOTION_PENDING_TIMEOUT_MS 30u
#define CAN_MOTION_HOLD_TIMEOUT_MS    50u
#define CAN_MOTION_FAULT_TIMEOUT_MS   500u
#define CAN_MOTION_FEEDBACK_PERIOD_MS 10u
#define CAN_MOTION_HEALTH_PERIOD_MS   50u
#define CAN_MOTION_POSITION_LEASE_MS  50u

typedef struct {
    can_motion_ops_t ops;
    can_node_state_t state;
    uint8_t node_id;
    bool joint_ready;
    uint16_t session;
    uint16_t expected_sequence;
    can_trajectory_t pending;
    bool pending_valid;
    uint16_t pending_age_ms;
    can_trajectory_t last_received;
    bool last_received_valid;
    can_trajectory_t applied_point;
    uint16_t applied_sequence;
    bool applied_valid;
    uint16_t sync_age_ms;
    bool position_active;
    uint8_t feedback_ticks;
    uint8_t health_ticks;
    bool feedback_due;
    bool health_due;
    uint32_t rx_frames;
    uint32_t tx_frames;
    uint32_t tx_failures;
    uint32_t protocol_errors;
    uint32_t fault_bits;
} can_motion_service_t;

static can_motion_service_t s_service;

static void increment_u32(uint32_t *value)
{
    if (*value != UINT32_MAX) {
        ++*value;
    }
}

static uint16_t increment_u16(uint16_t value)
{
    if (value != UINT16_MAX) {
        ++value;
    }
    return value;
}

static bool valid_node_id(uint8_t node_id)
{
    return (node_id == 1u) || (node_id == 2u);
}

static void set_state(can_node_state_t state)
{
    if (s_service.state != state) {
        s_service.state = state;
        s_service.health_due = true;
    }
}

static void clear_pending(void)
{
    memset(&s_service.pending, 0, sizeof(s_service.pending));
    s_service.pending_valid = false;
    s_service.pending_age_ms = 0u;
}

static void clear_sequence_window(void)
{
    clear_pending();
    memset(&s_service.last_received, 0, sizeof(s_service.last_received));
    s_service.last_received_valid = false;
    memset(&s_service.applied_point, 0, sizeof(s_service.applied_point));
    s_service.applied_sequence = 0u;
    s_service.applied_valid = false;
    s_service.sync_age_ms = 0u;
}

static uint32_t read_faults(void)
{
    uint32_t current;

    if (s_service.ops.fault_get != NULL) {
        current = s_service.ops.fault_get();
    } else {
        current = FAULT_NONE;
    }
    if (current != s_service.fault_bits) {
        s_service.fault_bits = current;
        s_service.health_due = true;
    }
    return s_service.fault_bits;
}

static void stop_position(void)
{
    if (s_service.ops.position_stop != NULL) {
        s_service.ops.position_stop();
    }
    s_service.position_active = false;
}

static void enter_fault(void)
{
    clear_pending();
    if (s_service.state != CAN_NODE_STATE_FAULT) {
        stop_position();
        set_state(CAN_NODE_STATE_FAULT);
    }
}

static void latch_fault_and_stop(uint32_t bits)
{
    if (s_service.ops.fault_set != NULL) {
        s_service.ops.fault_set(bits);
    }
    read_faults();
    enter_fault();
}

static bool trajectory_equal(const can_trajectory_t *first,
                             const can_trajectory_t *second)
{
    return (first->position_mdeg == second->position_mdeg) &&
           (first->velocity_mdeg_s == second->velocity_mdeg_s) &&
           (first->sequence == second->sequence);
}

static bool trajectory_in_range(const can_trajectory_t *point)
{
    return (point->position_mdeg >= -POSITION_COMMAND_LIMIT_MDEG) &&
           (point->position_mdeg <= POSITION_COMMAND_LIMIT_MDEG) &&
           (point->velocity_mdeg_s >= -POSITION_MAX_VELOCITY_MDEG_S) &&
           (point->velocity_mdeg_s <= POSITION_MAX_VELOCITY_MDEG_S);
}

static bool first_target_safe(int32_t target_mdeg, int32_t actual_mdeg)
{
    int64_t difference = (int64_t)target_mdeg - (int64_t)actual_mdeg;

    return (difference >= -(int64_t)POSITION_MAX_ERROR_MDEG) &&
           (difference <= (int64_t)POSITION_MAX_ERROR_MDEG);
}

static void accept_pending(const can_trajectory_t *point)
{
    s_service.pending = *point;
    s_service.pending_valid = true;
    s_service.pending_age_ms = 0u;
    s_service.last_received = *point;
    s_service.last_received_valid = true;
}

static void process_trajectory(const can_frame_t *frame)
{
    can_trajectory_t point;

    if (!s_service.joint_ready ||
        (s_service.state != CAN_NODE_STATE_ARMED &&
         s_service.state != CAN_NODE_STATE_RUNNING &&
         s_service.state != CAN_NODE_STATE_HOLD) ||
        !can_protocol_decode_trajectory(frame, s_service.node_id, &point) ||
        !trajectory_in_range(&point)) {
        increment_u32(&s_service.protocol_errors);
        return;
    }

    if (s_service.applied_valid &&
        point.sequence == s_service.applied_sequence) {
        if (!trajectory_equal(&point, &s_service.applied_point)) {
            increment_u32(&s_service.protocol_errors);
        }
        return;
    }
    if (s_service.last_received_valid &&
        point.sequence == s_service.last_received.sequence) {
        if (!trajectory_equal(&point, &s_service.last_received)) {
            increment_u32(&s_service.protocol_errors);
        } else {
            accept_pending(&point);
        }
        return;
    }
    if (!s_service.applied_valid) {
        if (point.sequence != s_service.expected_sequence) {
            increment_u32(&s_service.protocol_errors);
            return;
        }
    } else if (!can_protocol_sequence_newer(
                   point.sequence,
                   s_service.last_received_valid
                       ? s_service.last_received.sequence
                       : s_service.applied_sequence)) {
        increment_u32(&s_service.protocol_errors);
        return;
    }

    accept_pending(&point);
}

static void apply_pending(uint16_t sequence, uint16_t session)
{
    position_setpoint_t setpoint;
    int result;

    if ((s_service.state != CAN_NODE_STATE_ARMED &&
         s_service.state != CAN_NODE_STATE_RUNNING &&
         s_service.state != CAN_NODE_STATE_HOLD) ||
        session != s_service.session) {
        return;
    }
    if (s_service.applied_valid && sequence == s_service.applied_sequence) {
        return;
    }
    if (!s_service.pending_valid || sequence != s_service.pending.sequence) {
        return;
    }

    setpoint.position_mdeg = s_service.pending.position_mdeg;
    setpoint.velocity_mdeg_s = s_service.pending.velocity_mdeg_s;
    setpoint.sequence = s_service.pending.sequence;
    setpoint.lease_ms = CAN_MOTION_POSITION_LEASE_MS;

    if (!s_service.applied_valid) {
        int32_t actual = 0;

        if (s_service.ops.position_mdeg != NULL) {
            actual = s_service.ops.position_mdeg();
        }
        if (!first_target_safe(setpoint.position_mdeg, actual)) {
            latch_fault_and_stop(FAULT_POSITION_TRACKING);
            return;
        }
        if (s_service.ops.position_start == NULL) {
            return;
        }
        result = s_service.ops.position_start(&setpoint);
    } else {
        if (s_service.ops.position_submit == NULL) {
            return;
        }
        result = s_service.ops.position_submit(&setpoint);
    }

    if (result != 0) {
        return;
    }
    s_service.applied_point = s_service.pending;
    s_service.applied_sequence = setpoint.sequence;
    s_service.applied_valid = true;
    s_service.position_active = true;
    s_service.sync_age_ms = 0u;
    clear_pending();
    set_state(CAN_NODE_STATE_RUNNING);
}

static void process_arm(const can_broadcast_t *control)
{
    if (!s_service.joint_ready ||
        (s_service.state != CAN_NODE_STATE_READY &&
         s_service.state != CAN_NODE_STATE_ARMED) ||
        ((read_faults() & FAULT_FATAL_MASK) != 0u)) {
        increment_u32(&s_service.protocol_errors);
        return;
    }
    s_service.session = control->session;
    s_service.expected_sequence = control->sequence;
    clear_sequence_window();
    set_state(CAN_NODE_STATE_ARMED);
}

static void process_clear_fault(void)
{
    if (s_service.state != CAN_NODE_STATE_FAULT) {
        increment_u32(&s_service.protocol_errors);
        return;
    }
    if (s_service.ops.fault_clear_can != NULL) {
        s_service.ops.fault_clear_can();
    }
    if ((read_faults() & FAULT_FATAL_MASK) != 0u) {
        return;
    }
    clear_pending();
    s_service.position_active = false;
    set_state(s_service.joint_ready ? CAN_NODE_STATE_READY
                                    : CAN_NODE_STATE_UNCONFIGURED);
}

static void process_control(const can_broadcast_t *control)
{
    switch (control->opcode) {
    case CAN_OPCODE_ARM:
        process_arm(control);
        break;
    case CAN_OPCODE_SYNC:
        apply_pending(control->sequence, control->session);
        break;
    case CAN_OPCODE_CLEAR_FAULT:
        process_clear_fault();
        break;
    case CAN_OPCODE_DISCOVER:
        if (s_service.joint_ready) {
            s_service.health_due = true;
        }
        break;
    case CAN_OPCODE_STOP:
    default:
        break;
    }
}

static void process_frame(const can_frame_t *frame)
{
    can_broadcast_t control;

    if (frame->id == CAN_ID_BROADCAST) {
        if (!can_protocol_decode_broadcast(frame, &control)) {
            increment_u32(&s_service.protocol_errors);
            return;
        }
        process_control(&control);
        return;
    }
    if (s_service.joint_ready &&
        frame->id == CAN_ID_TRAJECTORY(s_service.node_id)) {
        process_trajectory(frame);
        return;
    }
    increment_u32(&s_service.protocol_errors);
}

static void process_stop(void)
{
    clear_pending();
    stop_position();
    if ((read_faults() & FAULT_FATAL_MASK) != 0u) {
        set_state(CAN_NODE_STATE_FAULT);
    } else {
        set_state(s_service.joint_ready ? CAN_NODE_STATE_READY
                                        : CAN_NODE_STATE_UNCONFIGURED);
    }
}

static bool batch_contains_stop(const can_frame_t *frames, uint8_t count)
{
    uint8_t index;

    for (index = 0u; index < count; ++index) {
        can_broadcast_t control;

        if (frames[index].id == CAN_ID_BROADCAST &&
            can_protocol_decode_broadcast(&frames[index], &control) &&
            control.opcode == CAN_OPCODE_STOP) {
            return true;
        }
    }
    return false;
}

static void advance_time(void)
{
    if (s_service.pending_valid) {
        s_service.pending_age_ms = increment_u16(s_service.pending_age_ms);
    }
    if (s_service.applied_valid) {
        s_service.sync_age_ms = increment_u16(s_service.sync_age_ms);
    }

    ++s_service.feedback_ticks;
    if (s_service.feedback_ticks >= CAN_MOTION_FEEDBACK_PERIOD_MS) {
        s_service.feedback_ticks = 0u;
        s_service.feedback_due = true;
    }
    ++s_service.health_ticks;
    if (s_service.health_ticks >= CAN_MOTION_HEALTH_PERIOD_MS) {
        s_service.health_ticks = 0u;
        s_service.health_due = true;
    }
}

static void expire_pending_and_watchdog(void)
{
    if (s_service.pending_valid &&
        s_service.pending_age_ms >= CAN_MOTION_PENDING_TIMEOUT_MS) {
        clear_pending();
    }

    if (!s_service.applied_valid ||
        (s_service.state != CAN_NODE_STATE_RUNNING &&
         s_service.state != CAN_NODE_STATE_HOLD)) {
        return;
    }
    if (s_service.sync_age_ms >= CAN_MOTION_FAULT_TIMEOUT_MS) {
        latch_fault_and_stop(FAULT_CAN_TIMEOUT);
        return;
    }
    if (s_service.state == CAN_NODE_STATE_RUNNING &&
        s_service.sync_age_ms >= CAN_MOTION_HOLD_TIMEOUT_MS) {
        set_state(CAN_NODE_STATE_HOLD);
    }
}

void can_motion_service_init(const can_motion_ops_t *ops)
{
    memset(&s_service, 0, sizeof(s_service));
    if (ops != NULL) {
        s_service.ops = *ops;
    }
    s_service.state = CAN_NODE_STATE_UNCONFIGURED;
    s_service.health_due = true;
    read_faults();
}

void can_motion_service_set_joint_config(bool ready, uint8_t node_id)
{
    bool valid = ready && valid_node_id(node_id);
    bool changed = (valid != s_service.joint_ready) ||
                   (valid && node_id != s_service.node_id);

    if (!changed) {
        return;
    }
    if (s_service.position_active) {
        stop_position();
    }
    clear_sequence_window();
    s_service.session = 0u;
    s_service.expected_sequence = 0u;
    s_service.joint_ready = valid;
    s_service.node_id = valid ? node_id : 0u;
    if ((read_faults() & FAULT_FATAL_MASK) != 0u) {
        set_state(CAN_NODE_STATE_FAULT);
    } else {
        set_state(valid ? CAN_NODE_STATE_READY : CAN_NODE_STATE_UNCONFIGURED);
    }
}

void can_motion_service_tick_1ms(void)
{
    can_frame_t frames[CAN_MOTION_RX_BATCH_MAX];
    uint8_t count = 0u;
    uint8_t index;

    advance_time();
    if ((read_faults() & FAULT_FATAL_MASK) != 0u) {
        enter_fault();
    }

    if (s_service.ops.rx_pop != NULL) {
        while (count < CAN_MOTION_RX_BATCH_MAX &&
               s_service.ops.rx_pop(&frames[count])) {
            ++count;
            increment_u32(&s_service.rx_frames);
        }
    }

    if (batch_contains_stop(frames, count)) {
        process_stop();
    } else {
        for (index = 0u; index < count; ++index) {
            process_frame(&frames[index]);
        }
    }

    if ((read_faults() & FAULT_FATAL_MASK) != 0u) {
        enter_fault();
    }
    expire_pending_and_watchdog();
}

static void push_feedback(void)
{
    can_feedback_t feedback;
    can_frame_t frame;

    feedback.actual_position_mdeg = s_service.ops.position_mdeg != NULL
                                        ? s_service.ops.position_mdeg()
                                        : 0;
    feedback.actual_velocity_mdeg_s = s_service.ops.velocity_mdeg_s != NULL
                                          ? s_service.ops.velocity_mdeg_s()
                                          : 0;
    feedback.applied_sequence = s_service.applied_valid
                                    ? s_service.applied_sequence
                                    : 0u;
    if (can_protocol_encode_feedback(s_service.node_id, &feedback, &frame) &&
        s_service.ops.tx_push != NULL && s_service.ops.tx_push(&frame)) {
        increment_u32(&s_service.tx_frames);
    } else {
        increment_u32(&s_service.tx_failures);
    }
}

static void push_health(void)
{
    can_health_t health;
    can_frame_t frame;

    health.protocol_version = CAN_PROTOCOL_VERSION;
    health.node_state = (uint8_t)s_service.state;
    health.fault_bits = (uint16_t)read_faults();
    health.session = s_service.session;
    health.vbus_10mv = s_service.ops.vbus_10mv != NULL
                           ? s_service.ops.vbus_10mv()
                           : 0u;
    if (can_protocol_encode_health(s_service.node_id, &health, &frame) &&
        s_service.ops.tx_push != NULL && s_service.ops.tx_push(&frame)) {
        increment_u32(&s_service.tx_frames);
    } else {
        increment_u32(&s_service.tx_failures);
    }
}

void can_motion_service_poll_tx(void)
{
    bool feedback_due;
    bool health_due;

    (void)read_faults();
    feedback_due = s_service.feedback_due;
    health_due = s_service.health_due;
    s_service.feedback_due = false;
    s_service.health_due = false;
    if (!s_service.joint_ready) {
        return;
    }
    if (feedback_due) {
        push_feedback();
    }
    if (health_due) {
        push_health();
    }
}

bool can_motion_service_get_snapshot(can_motion_snapshot_t *out)
{
    if (out == NULL) {
        return false;
    }
    out->node_id = s_service.node_id;
    out->state = s_service.state;
    out->session = s_service.session;
    out->pending_sequence = s_service.pending_valid
                                ? s_service.pending.sequence
                                : 0u;
    out->applied_sequence = s_service.applied_valid
                                ? s_service.applied_sequence
                                : 0u;
    out->pending_age_ms = s_service.pending_valid
                              ? s_service.pending_age_ms
                              : 0u;
    out->sync_age_ms = s_service.applied_valid ? s_service.sync_age_ms : 0u;
    out->rx_frames = s_service.rx_frames;
    out->tx_frames = s_service.tx_frames;
    out->tx_failures = s_service.tx_failures;
    out->protocol_errors = s_service.protocol_errors;
    out->fault_bits = read_faults();
    out->joint_ready = s_service.joint_ready;
    out->pending_valid = s_service.pending_valid;
    out->applied_valid = s_service.applied_valid;
    out->position_active = s_service.position_active;
    return true;
}

void can_motion_service_force_stop(void)
{
    latch_fault_and_stop(FAULT_CAN_BUS);
}
