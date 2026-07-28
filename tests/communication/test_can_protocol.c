#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "can_protocol.h"

static void assert_frame_data(const can_frame_t *frame, const uint8_t expected[8])
{
    assert(frame->dlc == 8u);
    assert(memcmp(frame->data, expected, 8u) == 0);
}

static void assert_broadcast_equal(const can_broadcast_t *actual,
                                   const can_broadcast_t *expected)
{
    assert(actual->opcode == expected->opcode);
    assert(actual->protocol_version == expected->protocol_version);
    assert(actual->sequence == expected->sequence);
    assert(actual->session == expected->session);
    assert(actual->flags == expected->flags);
}

static void test_trajectory_decode(void)
{
    can_frame_t frame = {0};
    can_trajectory_t trajectory;

    frame.id = 0x101u;
    frame.dlc = 8u;
    frame.data[0] = 0x10u; frame.data[1] = 0x27u; /* 10000 mdeg */
    frame.data[4] = 0x2cu; frame.data[5] = 0x01u; /* 300 * 10 mdeg/s */
    frame.data[6] = 0x34u; frame.data[7] = 0x12u;
    assert(can_protocol_decode_trajectory(&frame, 1u, &trajectory));
    assert(trajectory.position_mdeg == 10000);
    assert(trajectory.velocity_mdeg_s == 3000);
    assert(trajectory.sequence == 0x1234u);
    assert(!can_protocol_decode_trajectory(&frame, 2u, &trajectory));

    frame.data[0] = 0xc0u; frame.data[1] = 0x1du;
    frame.data[2] = 0xfeu; frame.data[3] = 0xffu; /* -123456 mdeg */
    frame.data[4] = 0xbfu; frame.data[5] = 0xfeu; /* -321 * 10 mdeg/s */
    frame.data[6] = 0xfeu; frame.data[7] = 0xcau;
    assert(can_protocol_decode_trajectory(&frame, 1u, &trajectory));
    assert(trajectory.position_mdeg == -123456);
    assert(trajectory.velocity_mdeg_s == -3210);
    assert(trajectory.sequence == 0xcafeu);

    frame.dlc = 7u;
    assert(!can_protocol_decode_trajectory(&frame, 1u, &trajectory));
    frame.dlc = 8u;
    frame.id = 0x901u;
    assert(!can_protocol_decode_trajectory(&frame, 1u, &trajectory));
    assert(!can_protocol_decode_trajectory(NULL, 1u, &trajectory));
    assert(!can_protocol_decode_trajectory(&frame, 1u, NULL));
}

static void test_broadcast_round_trips_and_rejections(void)
{
    static const uint8_t expected[5][8] = {
        {0x01u, 0x01u, 0x34u, 0x12u, 0x78u, 0x56u, 0x00u, 0x20u},
        {0x02u, 0x01u, 0x02u, 0x10u, 0x22u, 0x20u, 0x00u, 0x08u},
        {0x03u, 0x01u, 0x03u, 0x10u, 0x23u, 0x20u, 0x00u, 0xdeu},
        {0x04u, 0x01u, 0x04u, 0x10u, 0x24u, 0x20u, 0x00u, 0xf2u},
        {0x05u, 0x01u, 0x05u, 0x10u, 0x25u, 0x20u, 0x00u, 0x24u},
    };
    can_frame_t frame = {0};
    can_broadcast_t value = {
        .opcode = CAN_OPCODE_ARM,
        .protocol_version = CAN_PROTOCOL_VERSION,
        .sequence = 0x1234u,
        .session = 0x5678u,
        .flags = 0u,
    };
    can_broadcast_t decoded;
    uint8_t index;

    assert(can_protocol_crc8((const uint8_t[]){
        0x01u, 0x01u, 0x34u, 0x12u, 0x78u, 0x56u, 0x00u
    }, 7u) == 0x20u);
    assert(can_protocol_encode_broadcast(&value, &frame));
    assert(frame.id == CAN_ID_BROADCAST);
    assert_frame_data(&frame, expected[0]);
    assert(can_protocol_decode_broadcast(&frame, &decoded));
    assert_broadcast_equal(&decoded, &value);

    for (index = 1u; index < 5u; ++index) {
        value.opcode = (can_opcode_t)(index + 1u);
        value.sequence = (uint16_t)(0x1001u + index);
        value.session = (uint16_t)(0x2021u + index);
        assert(can_protocol_encode_broadcast(&value, &frame));
        assert_frame_data(&frame, expected[index]);
        assert(can_protocol_decode_broadcast(&frame, &decoded));
        assert_broadcast_equal(&decoded, &value);
    }

    frame.dlc = 7u;
    assert(!can_protocol_decode_broadcast(&frame, &decoded));
    frame.dlc = 8u;
    frame.id = 0x880u;
    assert(!can_protocol_decode_broadcast(&frame, &decoded));
    frame.id = CAN_ID_BROADCAST;
    memcpy(frame.data, expected[0], 8u);
    frame.data[6] = 1u;
    frame.data[7] = can_protocol_crc8(frame.data, 7u);
    assert(!can_protocol_decode_broadcast(&frame, &decoded));
    memcpy(frame.data, expected[0], 8u);
    frame.data[1] = 2u;
    frame.data[7] = can_protocol_crc8(frame.data, 7u);
    assert(!can_protocol_decode_broadcast(&frame, &decoded));
    memcpy(frame.data, expected[0], 8u);
    frame.data[7] ^= 0x01u;
    assert(!can_protocol_decode_broadcast(&frame, &decoded));
    memcpy(frame.data, expected[0], 8u);
    frame.data[0] = 0u;
    frame.data[7] = can_protocol_crc8(frame.data, 7u);
    assert(!can_protocol_decode_broadcast(&frame, &decoded));
    value.opcode = (can_opcode_t)0u;
    assert(!can_protocol_encode_broadcast(&value, &frame));
    value.opcode = CAN_OPCODE_ARM;
    value.flags = 1u;
    assert(!can_protocol_encode_broadcast(&value, &frame));
    value.flags = 0u;
    value.protocol_version = 2u;
    assert(!can_protocol_encode_broadcast(&value, &frame));
    value.protocol_version = CAN_PROTOCOL_VERSION;
    assert(!can_protocol_encode_broadcast(NULL, &frame));
    assert(!can_protocol_encode_broadcast(&value, NULL));
}

static void test_feedback_and_health_encoders(void)
{
    static const uint8_t feedback_expected[8] = {
        0xc0u, 0x1du, 0xfeu, 0xffu, 0xbfu, 0xfeu, 0xfeu, 0xcau
    };
    static const uint8_t feedback_saturated_expected[8] = {
        0x00u, 0x00u, 0x00u, 0x00u, 0xffu, 0x7fu, 0x00u, 0x00u
    };
    static const uint8_t health_expected[8] = {
        0x01u, 0x05u, 0x34u, 0x12u, 0x78u, 0x56u, 0xbcu, 0x9au
    };
    can_feedback_t feedback = {
        .actual_position_mdeg = -123456,
        .actual_velocity_mdeg_s = -3210,
        .applied_sequence = 0xcafeu,
    };
    can_health_t health = {
        .protocol_version = CAN_PROTOCOL_VERSION,
        .node_state = CAN_NODE_STATE_FAULT,
        .fault_bits = 0x1234u,
        .session = 0x5678u,
        .vbus_10mv = 0x9abcu,
    };
    can_frame_t frame = {0};

    assert(can_protocol_encode_feedback(1u, &feedback, &frame));
    assert(frame.id == 0x181u);
    assert_frame_data(&frame, feedback_expected);
    assert(can_protocol_encode_feedback(2u, &feedback, &frame));
    assert(frame.id == 0x182u);
    feedback.actual_position_mdeg = 0;
    feedback.actual_velocity_mdeg_s = 327680;
    feedback.applied_sequence = 0u;
    assert(can_protocol_encode_feedback(1u, &feedback, &frame));
    assert_frame_data(&frame, feedback_saturated_expected);
    assert(!can_protocol_encode_feedback(0u, &feedback, &frame));
    assert(!can_protocol_encode_feedback(3u, &feedback, &frame));
    assert(!can_protocol_encode_feedback(1u, NULL, &frame));
    assert(!can_protocol_encode_feedback(1u, &feedback, NULL));

    assert(can_protocol_encode_health(1u, &health, &frame));
    assert(frame.id == 0x281u);
    assert_frame_data(&frame, health_expected);
    assert(can_protocol_encode_health(2u, &health, &frame));
    assert(frame.id == 0x282u);
    health.protocol_version = 2u;
    assert(!can_protocol_encode_health(1u, &health, &frame));
    health.protocol_version = CAN_PROTOCOL_VERSION;
    health.node_state = 6u;
    assert(!can_protocol_encode_health(1u, &health, &frame));
    health.node_state = CAN_NODE_STATE_FAULT;
    assert(!can_protocol_encode_health(0u, &health, &frame));
    assert(!can_protocol_encode_health(1u, NULL, &frame));
    assert(!can_protocol_encode_health(1u, &health, NULL));
}

static void test_sequence_helper(void)
{
    assert(can_protocol_sequence_newer(11u, 10u));
    assert(!can_protocol_sequence_newer(10u, 10u));
    assert(!can_protocol_sequence_newer(9u, 10u));
    assert(!can_protocol_sequence_newer(0x8000u, 0u));
    assert(can_protocol_sequence_newer(0u, 0xffffu));
}

int main(void)
{
    test_trajectory_decode();
    test_broadcast_round_trips_and_rejections();
    test_feedback_and_health_encoders();
    test_sequence_helper();
    puts("can protocol: PASS");
    return 0;
}
