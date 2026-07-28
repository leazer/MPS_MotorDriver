#include "can_motion_service.h"

#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "fault_manager.h"
#include "motor_params.h"
#include "motor_tuning.h"

#define RX_CAPACITY 32u
#define TX_CAPACITY 32u

#ifdef CAN_MOTION_SERVICE_TEST
extern void can_motion_service_test_seed_counters(uint32_t rx_frames,
                                                  uint32_t tx_frames,
                                                  uint32_t tx_failures,
                                                  uint32_t protocol_errors);
#endif

static can_frame_t s_rx[RX_CAPACITY];
static unsigned s_rx_head;
static unsigned s_rx_tail;
static can_frame_t s_tx[TX_CAPACITY];
static unsigned s_tx_count;
static bool s_tx_accept;

static int s_start_result;
static int s_submit_result;
static unsigned s_start_calls;
static unsigned s_submit_calls;
static unsigned s_stop_calls;
static position_setpoint_t s_last_start;
static position_setpoint_t s_last_submit;
static int32_t s_position_mdeg;
static int32_t s_velocity_mdeg_s;
static uint16_t s_vbus_10mv;
static uint32_t s_faults;
static uint32_t s_fault_set_bits;
static unsigned s_fault_clear_calls;
static bool s_fault_clear_blocked;

static bool fake_rx_pop(can_frame_t *out)
{
    if (s_rx_head == s_rx_tail) {
        return false;
    }
    *out = s_rx[s_rx_head % RX_CAPACITY];
    ++s_rx_head;
    return true;
}

static bool fake_tx_push(const can_frame_t *frame)
{
    if (!s_tx_accept) {
        return false;
    }
    assert(s_tx_count < TX_CAPACITY);
    s_tx[s_tx_count++] = *frame;
    return true;
}

static int fake_position_start(const position_setpoint_t *setpoint)
{
    ++s_start_calls;
    s_last_start = *setpoint;
    return s_start_result;
}

static int fake_position_submit(const position_setpoint_t *setpoint)
{
    ++s_submit_calls;
    s_last_submit = *setpoint;
    return s_submit_result;
}

static void fake_position_stop(void)
{
    ++s_stop_calls;
}

static int32_t fake_position_mdeg(void)
{
    return s_position_mdeg;
}

static int32_t fake_velocity_mdeg_s(void)
{
    return s_velocity_mdeg_s;
}

static uint16_t fake_vbus_10mv(void)
{
    return s_vbus_10mv;
}

static uint32_t fake_fault_get(void)
{
    return s_faults;
}

static void fake_fault_set(uint32_t bits)
{
    s_fault_set_bits |= bits;
    s_faults |= bits;
}

static void fake_fault_clear_can(void)
{
    ++s_fault_clear_calls;
    if (!s_fault_clear_blocked) {
        s_faults &= ~(FAULT_CAN_TIMEOUT | FAULT_CAN_BUS |
                      FAULT_POSITION_TRACKING);
    }
}

static const can_motion_ops_t s_ops = {
    fake_rx_pop,
    fake_tx_push,
    fake_position_start,
    fake_position_submit,
    fake_position_stop,
    fake_position_mdeg,
    fake_velocity_mdeg_s,
    fake_vbus_10mv,
    fake_fault_get,
    fake_fault_set,
    fake_fault_clear_can
};

static void reset_service(void)
{
    memset(s_rx, 0, sizeof(s_rx));
    memset(s_tx, 0, sizeof(s_tx));
    s_rx_head = 0u;
    s_rx_tail = 0u;
    s_tx_count = 0u;
    s_tx_accept = true;
    s_start_result = 0;
    s_submit_result = 0;
    s_start_calls = 0u;
    s_submit_calls = 0u;
    s_stop_calls = 0u;
    memset(&s_last_start, 0, sizeof(s_last_start));
    memset(&s_last_submit, 0, sizeof(s_last_submit));
    s_position_mdeg = 0;
    s_velocity_mdeg_s = 0;
    s_vbus_10mv = 1200u;
    s_faults = FAULT_NONE;
    s_fault_set_bits = FAULT_NONE;
    s_fault_clear_calls = 0u;
    s_fault_clear_blocked = false;
    can_motion_service_init(&s_ops);
}

static can_motion_snapshot_t snapshot(void)
{
    can_motion_snapshot_t value;

    memset(&value, 0xa5, sizeof(value));
    assert(can_motion_service_get_snapshot(&value));
    return value;
}

static void enqueue(can_frame_t frame)
{
    assert((s_rx_tail - s_rx_head) < RX_CAPACITY);
    s_rx[s_rx_tail % RX_CAPACITY] = frame;
    ++s_rx_tail;
}

static can_frame_t trajectory(uint8_t node_id,
                              int32_t position_mdeg,
                              int32_t velocity_mdeg_s,
                              uint16_t sequence)
{
    can_frame_t frame;
    uint32_t position = (uint32_t)position_mdeg;
    int16_t velocity_wire = (int16_t)(velocity_mdeg_s / 10);
    uint16_t velocity = (uint16_t)velocity_wire;

    memset(&frame, 0, sizeof(frame));
    frame.id = CAN_ID_TRAJECTORY(node_id);
    frame.dlc = 8u;
    frame.data[0] = (uint8_t)position;
    frame.data[1] = (uint8_t)(position >> 8u);
    frame.data[2] = (uint8_t)(position >> 16u);
    frame.data[3] = (uint8_t)(position >> 24u);
    frame.data[4] = (uint8_t)velocity;
    frame.data[5] = (uint8_t)(velocity >> 8u);
    frame.data[6] = (uint8_t)sequence;
    frame.data[7] = (uint8_t)(sequence >> 8u);
    return frame;
}

static can_frame_t broadcast(can_opcode_t opcode,
                             uint16_t sequence,
                             uint16_t session)
{
    can_broadcast_t value;
    can_frame_t frame;

    value.opcode = opcode;
    value.protocol_version = CAN_PROTOCOL_VERSION;
    value.sequence = sequence;
    value.session = session;
    value.flags = 0u;
    assert(can_protocol_encode_broadcast(&value, &frame));
    return frame;
}

static void tick_empty(unsigned count)
{
    while (count-- != 0u) {
        can_motion_service_tick_1ms();
    }
}

static void configure_and_arm(uint16_t sequence, uint16_t session)
{
    can_motion_service_set_joint_config(true, 1u);
    enqueue(broadcast(CAN_OPCODE_ARM, sequence, session));
    can_motion_service_tick_1ms();
    assert(snapshot().state == CAN_NODE_STATE_ARMED);
}

static void apply_first(int32_t position,
                        int32_t velocity,
                        uint16_t sequence,
                        uint16_t session)
{
    enqueue(trajectory(1u, position, velocity, sequence));
    enqueue(broadcast(CAN_OPCODE_SYNC, sequence, session));
    can_motion_service_tick_1ms();
    assert(snapshot().state == CAN_NODE_STATE_RUNNING);
}

static void test_frozen_wire_state_values(void)
{
    assert(CAN_NODE_STATE_UNCONFIGURED == 0u);
    assert(CAN_NODE_STATE_READY == 1u);
    assert(CAN_NODE_STATE_ARMED == 2u);
    assert(CAN_NODE_STATE_RUNNING == 3u);
    assert(CAN_NODE_STATE_HOLD == 4u);
    assert(CAN_NODE_STATE_FAULT == 5u);
}

static void test_configuration_and_discover(void)
{
    can_motion_snapshot_t state;

    reset_service();
    state = snapshot();
    assert(!state.joint_ready);
    assert(state.node_id == 0u);
    assert(state.state == CAN_NODE_STATE_UNCONFIGURED);
    assert(!state.pending_valid);
    assert(!state.applied_valid);
    assert(!can_motion_service_get_snapshot(NULL));

    can_motion_service_set_joint_config(true, 0u);
    assert(snapshot().state == CAN_NODE_STATE_UNCONFIGURED);
    can_motion_service_set_joint_config(true, 3u);
    assert(snapshot().state == CAN_NODE_STATE_UNCONFIGURED);
    enqueue(broadcast(CAN_OPCODE_DISCOVER, 0u, 0u));
    can_motion_service_tick_1ms();
    can_motion_service_poll_tx();
    assert(s_tx_count == 0u);

    can_motion_service_set_joint_config(true, 1u);
    state = snapshot();
    assert(state.joint_ready);
    assert(state.node_id == 1u);
    assert(state.state == CAN_NODE_STATE_READY);
    can_motion_service_poll_tx();
    assert(s_tx_count == 1u);
    assert(s_tx[0].id == CAN_ID_HEALTH(1u));
    assert(s_tx[0].data[0] == CAN_PROTOCOL_VERSION);
    assert(s_tx[0].data[1] == CAN_NODE_STATE_READY);

    s_tx_count = 0u;
    enqueue(broadcast(CAN_OPCODE_DISCOVER, 99u, 77u));
    can_motion_service_tick_1ms();
    assert(snapshot().state == CAN_NODE_STATE_READY);
    can_motion_service_poll_tx();
    assert(s_tx_count == 1u);
    assert(s_tx[0].id == CAN_ID_HEALTH(1u));
}

static void test_arm_preload_sync_and_idempotence(void)
{
    can_motion_snapshot_t state;
    can_frame_t first;
    uint32_t errors;

    reset_service();
    configure_and_arm(10u, 0x1234u);
    state = snapshot();
    assert(state.session == 0x1234u);
    assert(!state.pending_valid);
    assert(!state.applied_valid);
    assert(s_start_calls == 0u);

    enqueue(broadcast(CAN_OPCODE_SYNC, 10u, 0x1234u));
    can_motion_service_tick_1ms();
    assert(s_start_calls == 0u);
    assert(snapshot().state == CAN_NODE_STATE_ARMED);

    first = trajectory(1u, 12000, -3210, 10u);
    enqueue(first);
    can_motion_service_tick_1ms();
    state = snapshot();
    assert(state.pending_valid);
    assert(state.pending_sequence == 10u);
    assert(state.pending_age_ms == 0u);
    assert(s_start_calls == 0u);

    enqueue(first);
    can_motion_service_tick_1ms();
    assert(snapshot().pending_age_ms == 0u);
    errors = snapshot().protocol_errors;
    enqueue(trajectory(1u, 12001, -3210, 10u));
    can_motion_service_tick_1ms();
    assert(snapshot().protocol_errors == errors + 1u);
    assert(snapshot().pending_sequence == 10u);

    enqueue(broadcast(CAN_OPCODE_SYNC, 10u, 0x9999u));
    enqueue(broadcast(CAN_OPCODE_SYNC, 11u, 0x1234u));
    can_motion_service_tick_1ms();
    assert(s_start_calls == 0u);
    assert(snapshot().pending_valid);

    enqueue(broadcast(CAN_OPCODE_SYNC, 10u, 0x1234u));
    can_motion_service_tick_1ms();
    state = snapshot();
    assert(s_start_calls == 1u);
    assert(s_submit_calls == 0u);
    assert(s_last_start.position_mdeg == 12000);
    assert(s_last_start.velocity_mdeg_s == -3210);
    assert(s_last_start.sequence == 10u);
    assert(s_last_start.lease_ms == 50u);
    assert(state.state == CAN_NODE_STATE_RUNNING);
    assert(state.position_active);
    assert(!state.pending_valid);
    assert(state.applied_valid);
    assert(state.applied_sequence == 10u);
    assert(state.sync_age_ms == 0u);

    enqueue(first);
    enqueue(broadcast(CAN_OPCODE_SYNC, 10u, 0x1234u));
    can_motion_service_tick_1ms();
    assert(s_start_calls == 1u);
    assert(s_submit_calls == 0u);
    assert(!snapshot().pending_valid);

    enqueue(trajectory(1u, -5000, 60000, 11u));
    can_motion_service_tick_1ms();
    assert(s_submit_calls == 0u);
    enqueue(broadcast(CAN_OPCODE_SYNC, 11u, 0x1234u));
    can_motion_service_tick_1ms();
    assert(s_start_calls == 1u);
    assert(s_submit_calls == 1u);
    assert(s_last_submit.position_mdeg == -5000);
    assert(s_last_submit.velocity_mdeg_s == 60000);
    assert(s_last_submit.sequence == 11u);
    assert(s_last_submit.lease_ms == 50u);
    assert(snapshot().applied_sequence == 11u);
}

static void test_range_state_and_decode_rejections(void)
{
    can_frame_t bad;
    uint32_t errors;

    reset_service();
    can_motion_service_set_joint_config(true, 1u);
    errors = snapshot().protocol_errors;
    enqueue(trajectory(1u, 0, 0, 1u));
    enqueue(trajectory(2u, 0, 0, 1u));
    bad = trajectory(1u, 0, 0, 1u);
    bad.dlc = 7u;
    enqueue(bad);
    bad = broadcast(CAN_OPCODE_ARM, 1u, 1u);
    bad.data[7] ^= 1u;
    enqueue(bad);
    can_motion_service_tick_1ms();
    assert(snapshot().protocol_errors == errors + 4u);
    assert(snapshot().state == CAN_NODE_STATE_READY);
    assert(!snapshot().pending_valid);

    configure_and_arm(1u, 2u);
    errors = snapshot().protocol_errors;
    enqueue(trajectory(1u, POSITION_COMMAND_LIMIT_MDEG + 1, 0, 1u));
    enqueue(trajectory(1u, 0, POSITION_MAX_VELOCITY_MDEG_S + 10, 1u));
    enqueue(trajectory(1u, -POSITION_COMMAND_LIMIT_MDEG - 1, 0, 1u));
    enqueue(trajectory(1u, 0, -POSITION_MAX_VELOCITY_MDEG_S - 10, 1u));
    can_motion_service_tick_1ms();
    assert(snapshot().protocol_errors == errors + 4u);
    assert(!snapshot().pending_valid);
}

static void test_pending_expiry_exact_boundary(void)
{
    reset_service();
    configure_and_arm(7u, 8u);
    enqueue(trajectory(1u, 1000, 0, 7u));
    can_motion_service_tick_1ms();
    tick_empty(29u);
    assert(snapshot().pending_valid);
    assert(snapshot().pending_age_ms == 29u);
    tick_empty(1u);
    assert(!snapshot().pending_valid);
    assert(snapshot().pending_age_ms == 0u);
    enqueue(broadcast(CAN_OPCODE_SYNC, 7u, 8u));
    can_motion_service_tick_1ms();
    assert(s_start_calls == 0u);
}

static void test_stop_priority_and_bounded_drain(void)
{
    can_motion_snapshot_t state;

    reset_service();
    configure_and_arm(1u, 10u);
    enqueue(trajectory(1u, 1000, 0, 1u));
    enqueue(broadcast(CAN_OPCODE_SYNC, 1u, 10u));
    enqueue(broadcast(CAN_OPCODE_STOP, 0u, 0xffffu));
    enqueue(broadcast(CAN_OPCODE_ARM, 5u, 20u));
    enqueue(broadcast(CAN_OPCODE_ARM, 6u, 30u));
    can_motion_service_tick_1ms();
    state = snapshot();
    assert(state.state == CAN_NODE_STATE_READY);
    assert(!state.pending_valid);
    assert(!state.position_active);
    assert(s_start_calls == 0u);
    assert(s_stop_calls == 1u);
    assert(state.rx_frames == 5u); /* ARM setup plus exactly four this tick. */
    assert((s_rx_tail - s_rx_head) == 1u);

    can_motion_service_tick_1ms();
    state = snapshot();
    assert(state.state == CAN_NODE_STATE_ARMED);
    assert(state.session == 30u);
    assert(state.rx_frames == 6u);
}

static void test_watchdog_hold_resume_timeout_and_clear(void)
{
    can_motion_snapshot_t state;
    unsigned submit_before;

    reset_service();
    configure_and_arm(20u, 44u);
    s_position_mdeg = 1000;
    apply_first(1500, 100, 20u, 44u);
    tick_empty(49u);
    assert(snapshot().state == CAN_NODE_STATE_RUNNING);
    assert(snapshot().sync_age_ms == 49u);
    submit_before = s_submit_calls;
    s_position_mdeg = 1777;
    tick_empty(1u);
    state = snapshot();
    assert(state.state == CAN_NODE_STATE_HOLD);
    assert(state.position_active);
    assert(state.sync_age_ms == 50u);
    assert(s_stop_calls == 0u);
    assert(s_submit_calls == submit_before);

    enqueue(trajectory(1u, 1800, -200, 21u));
    enqueue(broadcast(CAN_OPCODE_SYNC, 21u, 44u));
    can_motion_service_tick_1ms();
    state = snapshot();
    assert(state.state == CAN_NODE_STATE_RUNNING);
    assert(state.sync_age_ms == 0u);
    assert(state.applied_sequence == 21u);
    assert(s_last_submit.position_mdeg == 1800);
    assert(s_last_submit.velocity_mdeg_s == -200);

    tick_empty(499u);
    assert(snapshot().state == CAN_NODE_STATE_HOLD);
    assert(snapshot().sync_age_ms == 499u);
    tick_empty(1u);
    state = snapshot();
    assert(state.state == CAN_NODE_STATE_FAULT);
    assert(!state.position_active);
    assert(state.sync_age_ms == 500u);
    assert(s_stop_calls == 1u);
    assert((s_fault_set_bits & FAULT_CAN_TIMEOUT) != 0u);
    assert((state.fault_bits & FAULT_CAN_TIMEOUT) != 0u);

    enqueue(trajectory(1u, 1900, 0, 22u));
    enqueue(broadcast(CAN_OPCODE_SYNC, 22u, 44u));
    can_motion_service_tick_1ms();
    assert(snapshot().state == CAN_NODE_STATE_FAULT);
    assert(snapshot().applied_sequence == 21u);

    s_fault_clear_blocked = true;
    enqueue(broadcast(CAN_OPCODE_CLEAR_FAULT, 0u, 0u));
    can_motion_service_tick_1ms();
    assert(s_fault_clear_calls == 1u);
    assert(snapshot().state == CAN_NODE_STATE_FAULT);

    s_fault_clear_blocked = false;
    enqueue(broadcast(CAN_OPCODE_CLEAR_FAULT, 0u, 0u));
    can_motion_service_tick_1ms();
    assert(s_fault_clear_calls == 2u);
    assert(snapshot().state == CAN_NODE_STATE_READY);
    assert(snapshot().applied_valid);
    assert(snapshot().applied_sequence == 21u);
    assert(!snapshot().position_active);
}

static void test_stop_is_session_independent_and_fault_latched(void)
{
    reset_service();
    configure_and_arm(1u, 2u);
    apply_first(0, 0, 1u, 2u);
    enqueue(broadcast(CAN_OPCODE_STOP, 999u, 0xffffu));
    can_motion_service_tick_1ms();
    assert(s_stop_calls == 1u);
    assert(snapshot().state == CAN_NODE_STATE_READY);
    assert(snapshot().applied_valid);
    assert(snapshot().applied_sequence == 1u);

    s_faults = FAULT_DRIVER;
    can_motion_service_tick_1ms();
    assert(snapshot().state == CAN_NODE_STATE_FAULT);
    enqueue(broadcast(CAN_OPCODE_STOP, 0u, 0x1111u));
    can_motion_service_tick_1ms();
    assert(snapshot().state == CAN_NODE_STATE_FAULT);
    assert((snapshot().fault_bits & FAULT_DRIVER) != 0u);
    assert(s_stop_calls == 3u);
}

static void test_stop_cannot_exit_latched_fault_after_condition_clears(void)
{
    reset_service();
    configure_and_arm(1u, 2u);
    apply_first(0, 0, 1u, 2u);
    can_motion_service_force_stop();
    assert(snapshot().state == CAN_NODE_STATE_FAULT);
    assert(s_stop_calls == 1u);

    s_faults = FAULT_NONE;
    enqueue(broadcast(CAN_OPCODE_STOP, 0u, 0xffffu));
    can_motion_service_tick_1ms();
    assert(snapshot().state == CAN_NODE_STATE_FAULT);
    assert(!snapshot().pending_valid);
    assert(!snapshot().position_active);
    assert(s_stop_calls == 2u);

    enqueue(broadcast(CAN_OPCODE_CLEAR_FAULT, 0u, 0u));
    can_motion_service_tick_1ms();
    assert(snapshot().state == CAN_NODE_STATE_READY);
}

static void test_config_change_cannot_exit_latched_fault(void)
{
    reset_service();
    configure_and_arm(1u, 2u);
    can_motion_service_force_stop();
    assert(snapshot().state == CAN_NODE_STATE_FAULT);
    s_faults = FAULT_NONE;

    can_motion_service_set_joint_config(false, 0u);
    assert(snapshot().state == CAN_NODE_STATE_FAULT);
    assert(!snapshot().joint_ready);
    can_motion_service_set_joint_config(true, 2u);
    assert(snapshot().state == CAN_NODE_STATE_FAULT);
    assert(snapshot().joint_ready);
    assert(snapshot().node_id == 2u);

    enqueue(broadcast(CAN_OPCODE_CLEAR_FAULT, 0u, 0u));
    can_motion_service_tick_1ms();
    assert(snapshot().state == CAN_NODE_STATE_READY);

    can_motion_service_force_stop();
    s_faults = FAULT_NONE;
    can_motion_service_set_joint_config(false, 0u);
    assert(snapshot().state == CAN_NODE_STATE_FAULT);
    enqueue(broadcast(CAN_OPCODE_CLEAR_FAULT, 0u, 0u));
    can_motion_service_tick_1ms();
    assert(snapshot().state == CAN_NODE_STATE_UNCONFIGURED);
}

static void test_session_change_sequence_wrap_and_stale_pending(void)
{
    reset_service();
    configure_and_arm(0xffffu, 11u);
    enqueue(trajectory(1u, 100, 0, 0xffffu));
    can_motion_service_tick_1ms();
    assert(snapshot().pending_valid);
    enqueue(broadcast(CAN_OPCODE_ARM, 0xffffu, 12u));
    can_motion_service_tick_1ms();
    assert(snapshot().session == 12u);
    assert(!snapshot().pending_valid);
    enqueue(broadcast(CAN_OPCODE_SYNC, 0xffffu, 12u));
    can_motion_service_tick_1ms();
    assert(s_start_calls == 0u);

    apply_first(200, 0, 0xffffu, 12u);
    enqueue(trajectory(1u, 300, 0, 0u));
    enqueue(broadcast(CAN_OPCODE_SYNC, 0u, 12u));
    can_motion_service_tick_1ms();
    assert(s_submit_calls == 1u);
    assert(snapshot().applied_sequence == 0u);

    enqueue(trajectory(1u, 400, 0, 0xffffu));
    can_motion_service_tick_1ms();
    assert(!snapshot().pending_valid);
    enqueue(trajectory(1u, 500, 0, 0x8000u));
    can_motion_service_tick_1ms();
    assert(!snapshot().pending_valid);
}

static void test_first_target_gate_and_callback_failures(void)
{
    uint32_t errors;

    reset_service();
    configure_and_arm(1u, 1u);
    s_position_mdeg = 0;
    enqueue(trajectory(1u, POSITION_MAX_ERROR_MDEG + 1, 0, 1u));
    enqueue(broadcast(CAN_OPCODE_SYNC, 1u, 1u));
    can_motion_service_tick_1ms();
    assert(s_start_calls == 0u);
    assert(s_stop_calls == 1u);
    assert((s_fault_set_bits & FAULT_POSITION_TRACKING) != 0u);
    assert(snapshot().state == CAN_NODE_STATE_FAULT);
    assert(!snapshot().applied_valid);

    reset_service();
    configure_and_arm(1u, 1u);
    s_start_result = -3;
    enqueue(trajectory(1u, 0, 0, 1u));
    enqueue(broadcast(CAN_OPCODE_SYNC, 1u, 1u));
    can_motion_service_tick_1ms();
    assert(s_start_calls == 1u);
    assert(snapshot().state == CAN_NODE_STATE_ARMED);
    assert(snapshot().pending_valid);
    assert(!snapshot().applied_valid);
    errors = snapshot().protocol_errors;
    s_start_result = 0;
    enqueue(broadcast(CAN_OPCODE_SYNC, 1u, 1u));
    can_motion_service_tick_1ms();
    assert(s_start_calls == 2u);
    assert(snapshot().state == CAN_NODE_STATE_RUNNING);
    assert(snapshot().protocol_errors == errors);

    enqueue(trajectory(1u, 100, 0, 2u));
    can_motion_service_tick_1ms();
    s_submit_result = -2;
    enqueue(broadcast(CAN_OPCODE_SYNC, 2u, 1u));
    can_motion_service_tick_1ms();
    assert(s_submit_calls == 1u);
    assert(snapshot().applied_sequence == 1u);
    assert(snapshot().pending_valid);
    s_submit_result = 0;
    enqueue(broadcast(CAN_OPCODE_SYNC, 2u, 1u));
    can_motion_service_tick_1ms();
    assert(s_submit_calls == 2u);
    assert(snapshot().applied_sequence == 2u);
    assert(!snapshot().pending_valid);
}

static void test_force_stop_and_fault_authority(void)
{
    reset_service();
    configure_and_arm(1u, 1u);
    apply_first(0, 0, 1u, 1u);
    can_motion_service_force_stop();
    assert(s_stop_calls == 1u);
    assert((s_fault_set_bits & FAULT_CAN_BUS) != 0u);
    assert(snapshot().state == CAN_NODE_STATE_FAULT);
    assert(!snapshot().position_active);
    assert(!snapshot().pending_valid);

    enqueue(broadcast(CAN_OPCODE_ARM, 2u, 2u));
    can_motion_service_tick_1ms();
    assert(snapshot().state == CAN_NODE_STATE_FAULT);

    s_faults = FAULT_POSITION_TRACKING;
    enqueue(broadcast(CAN_OPCODE_CLEAR_FAULT, 0u, 0u));
    can_motion_service_tick_1ms();
    assert(snapshot().state == CAN_NODE_STATE_READY);
    assert((snapshot().fault_bits & FAULT_POSITION_TRACKING) == 0u);

    can_motion_service_force_stop();
    s_faults = FAULT_OVERCURRENT;
    enqueue(broadcast(CAN_OPCODE_CLEAR_FAULT, 0u, 0u));
    can_motion_service_tick_1ms();
    assert(snapshot().state == CAN_NODE_STATE_FAULT);
    assert((snapshot().fault_bits & FAULT_OVERCURRENT) != 0u);

    s_faults = FAULT_CAN_BUS;
    can_motion_service_set_joint_config(false, 0u);
    enqueue(broadcast(CAN_OPCODE_CLEAR_FAULT, 0u, 0u));
    can_motion_service_tick_1ms();
    assert(snapshot().state == CAN_NODE_STATE_UNCONFIGURED);
    assert(!snapshot().joint_ready);
}

static void test_feedback_health_schedule_and_tx_failure(void)
{
    can_motion_snapshot_t state;
    unsigned tx_before;

    reset_service();
    configure_and_arm(0x3456u, 0x789au);
    s_tx_count = 0u;
    s_position_mdeg = -123456;
    s_velocity_mdeg_s = 12340;
    s_vbus_10mv = 1555u;
    apply_first(-120000, 1000, 0x3456u, 0x789au);
    can_motion_service_poll_tx();
    assert(s_tx_count == 1u);
    assert(s_tx[0].id == CAN_ID_HEALTH(1u));
    assert(s_tx[0].data[0] == CAN_PROTOCOL_VERSION);
    assert(s_tx[0].data[1] == CAN_NODE_STATE_RUNNING);
    assert(s_tx[0].data[2] == 0u && s_tx[0].data[3] == 0u);
    assert(s_tx[0].data[4] == 0x9au && s_tx[0].data[5] == 0x78u);
    assert(s_tx[0].data[6] == 0x13u && s_tx[0].data[7] == 0x06u);

    tick_empty(8u); /* ARM tick + apply tick + eight = feedback tick 10. */
    can_motion_service_poll_tx();
    assert(s_tx_count == 2u);
    assert(s_tx[1].id == CAN_ID_FEEDBACK(1u));
    assert(s_tx[1].dlc == 8u);
    assert(s_tx[1].data[0] == 0xc0u && s_tx[1].data[1] == 0x1du);
    assert(s_tx[1].data[2] == 0xfeu && s_tx[1].data[3] == 0xffu);
    assert(s_tx[1].data[4] == 0xd2u && s_tx[1].data[5] == 0x04u);
    assert(s_tx[1].data[6] == 0x56u && s_tx[1].data[7] == 0x34u);

    tick_empty(40u); /* Service tick 50: feedback and health are both due. */
    can_motion_service_poll_tx();
    assert(s_tx_count == 4u);
    assert(s_tx[2].id == CAN_ID_FEEDBACK(1u));
    assert(s_tx[3].id == CAN_ID_HEALTH(1u));

    state = snapshot();
    assert(state.rx_frames == 3u);
    assert(state.tx_frames == 4u);
    assert(state.tx_failures == 0u);
    assert(state.fault_bits == FAULT_NONE);

    enqueue(trajectory(1u, -119000, 1000, 0x3457u));
    enqueue(broadcast(CAN_OPCODE_SYNC, 0x3457u, 0x789au));
    can_motion_service_tick_1ms();
    tick_empty(9u);
    s_tx_accept = false;
    tx_before = s_tx_count;
    can_motion_service_poll_tx();
    state = snapshot();
    assert(s_tx_count == tx_before);
    assert(state.tx_failures == 1u);
    can_motion_service_poll_tx();
    assert(snapshot().tx_failures == 1u); /* Failed old snapshot was discarded. */
}

static void test_fault_change_sends_immediate_health(void)
{
    reset_service();
    can_motion_service_set_joint_config(true, 1u);
    can_motion_service_poll_tx();
    s_tx_count = 0u;

    s_faults = FAULT_CAL_INVALID;
    can_motion_service_tick_1ms();
    assert(snapshot().state == CAN_NODE_STATE_READY);
    can_motion_service_poll_tx();
    assert(s_tx_count == 1u);
    assert(s_tx[0].id == CAN_ID_HEALTH(1u));
    assert(s_tx[0].data[1] == CAN_NODE_STATE_READY);
    assert(s_tx[0].data[2] == (uint8_t)FAULT_CAL_INVALID);
    assert(s_tx[0].data[3] == 0u);

    s_tx_count = 0u;
    s_faults = FAULT_NONE;
    can_motion_service_poll_tx();
    assert(s_tx_count == 1u);
    assert(s_tx[0].id == CAN_ID_HEALTH(1u));
    assert(s_tx[0].data[2] == 0u && s_tx[0].data[3] == 0u);
}

static void test_ages_saturate(void)
{
    reset_service();
    configure_and_arm(1u, 1u);
    apply_first(0, 0, 1u, 1u);
    tick_empty((unsigned)UINT16_MAX + 25u);
    assert(snapshot().sync_age_ms == UINT16_MAX);
    assert(snapshot().state == CAN_NODE_STATE_FAULT);
}

#ifdef CAN_MOTION_SERVICE_TEST
static void test_diagnostic_counters_saturate(void)
{
    can_motion_snapshot_t state;
    can_frame_t invalid;

    reset_service();
    can_motion_service_set_joint_config(true, 1u);
    can_motion_service_poll_tx();
    s_tx_count = 0u;

    can_motion_service_test_seed_counters(UINT32_MAX, 0u, 0u, 0u);
    enqueue(broadcast(CAN_OPCODE_DISCOVER, 0u, 0u));
    can_motion_service_tick_1ms();
    assert(snapshot().rx_frames == UINT32_MAX);

    can_motion_service_test_seed_counters(UINT32_MAX, UINT32_MAX, 0u, 0u);
    can_motion_service_poll_tx();
    assert(s_tx_count == 1u);
    assert(snapshot().tx_frames == UINT32_MAX);

    can_motion_service_test_seed_counters(UINT32_MAX, UINT32_MAX,
                                          UINT32_MAX, 0u);
    s_tx_accept = false;
    enqueue(broadcast(CAN_OPCODE_DISCOVER, 0u, 0u));
    can_motion_service_tick_1ms();
    can_motion_service_poll_tx();
    assert(snapshot().tx_failures == UINT32_MAX);

    can_motion_service_test_seed_counters(UINT32_MAX, UINT32_MAX,
                                          UINT32_MAX, UINT32_MAX);
    memset(&invalid, 0, sizeof(invalid));
    invalid.id = 0x7ffu;
    invalid.dlc = 8u;
    enqueue(invalid);
    can_motion_service_tick_1ms();
    state = snapshot();
    assert(state.rx_frames == UINT32_MAX);
    assert(state.tx_frames == UINT32_MAX);
    assert(state.tx_failures == UINT32_MAX);
    assert(state.protocol_errors == UINT32_MAX);
}

static void test_diagnostic_reset_preserves_motion_state(void)
{
    can_motion_snapshot_t before;
    can_motion_snapshot_t after;

    reset_service();
    configure_and_arm(17u, 0x3456u);
    apply_first(1000, 20, 17u, 0x3456u);
    enqueue(trajectory(1u, 1100, 30, 18u));
    can_motion_service_tick_1ms();
    can_motion_service_test_seed_counters(11u, 22u, 33u, 44u);
    before = snapshot();
    assert(before.state == CAN_NODE_STATE_RUNNING);
    assert(before.pending_valid);
    assert(before.applied_valid);
    assert(before.position_active);
    can_motion_service_reset_diagnostics();
    after = snapshot();
    assert(after.rx_frames == 0u);
    assert(after.tx_frames == 0u);
    assert(after.tx_failures == 0u);
    assert(after.protocol_errors == 0u);
    assert(after.node_id == before.node_id);
    assert(after.state == before.state);
    assert(after.session == before.session);
    assert(after.pending_sequence == before.pending_sequence);
    assert(after.pending_age_ms == before.pending_age_ms);
    assert(after.pending_valid == before.pending_valid);
    assert(after.applied_sequence == before.applied_sequence);
    assert(after.applied_valid == before.applied_valid);
    assert(after.sync_age_ms == before.sync_age_ms);
    assert(after.position_active == before.position_active);
    assert(after.joint_ready == before.joint_ready);
    assert(after.fault_bits == before.fault_bits);
}
#endif

int main(void)
{
    motor_tuning_init();
    test_frozen_wire_state_values();
    test_configuration_and_discover();
    test_arm_preload_sync_and_idempotence();
    test_range_state_and_decode_rejections();
    test_pending_expiry_exact_boundary();
    test_stop_priority_and_bounded_drain();
    test_watchdog_hold_resume_timeout_and_clear();
    test_stop_is_session_independent_and_fault_latched();
    test_stop_cannot_exit_latched_fault_after_condition_clears();
    test_config_change_cannot_exit_latched_fault();
    test_session_change_sequence_wrap_and_stale_pending();
    test_first_target_gate_and_callback_failures();
    test_force_stop_and_fault_authority();
    test_feedback_health_schedule_and_tx_failure();
    test_fault_change_sends_immediate_health();
    test_ages_saturate();
#ifdef CAN_MOTION_SERVICE_TEST
    test_diagnostic_counters_saturate();
    test_diagnostic_reset_preserves_motion_state();
#endif
    puts("can motion service: PASS");
    return 0;
}
