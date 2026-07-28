#include "can_protocol.h"

#include <limits.h>
#include <stddef.h>

static bool can_protocol_valid_node(uint8_t node_id)
{
    return (node_id >= 1u) && (node_id <= 2u);
}

static bool can_protocol_valid_standard_frame(const can_frame_t *frame,
                                              uint16_t expected_id)
{
    return (frame != NULL) && (frame->id <= 0x7ffu) &&
           (frame->id == expected_id) && (frame->dlc == 8u);
}

static bool can_protocol_valid_opcode(can_opcode_t opcode)
{
    return ((uint8_t)opcode >= (uint8_t)CAN_OPCODE_ARM) &&
           ((uint8_t)opcode <= (uint8_t)CAN_OPCODE_DISCOVER);
}

static int16_t can_protocol_velocity_to_wire(int32_t velocity_mdeg_s)
{
    int32_t velocity_10mdeg_s = velocity_mdeg_s / 10;

    if (velocity_10mdeg_s > INT16_MAX) {
        return INT16_MAX;
    }
    if (velocity_10mdeg_s < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)velocity_10mdeg_s;
}

uint8_t can_protocol_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0u;
    uint8_t index;

    if ((data == NULL) && (len != 0u)) {
        return 0u;
    }

    for (index = 0u; index < len; ++index) {
        uint8_t bit;

        crc ^= data[index];
        for (bit = 0u; bit < 8u; ++bit) {
            if ((crc & 0x80u) != 0u) {
                crc = (uint8_t)((crc << 1u) ^ 0x07u);
            } else {
                crc = (uint8_t)(crc << 1u);
            }
        }
    }

    return crc;
}

bool can_protocol_sequence_newer(uint16_t candidate, uint16_t previous)
{
    uint16_t difference = (uint16_t)(candidate - previous);

    return (difference != 0u) && (difference < 0x8000u);
}

bool can_protocol_decode_trajectory(const can_frame_t *frame,
                                    uint8_t node_id,
                                    can_trajectory_t *out)
{
    uint32_t position;
    uint16_t velocity;

    if ((out == NULL) || !can_protocol_valid_node(node_id) ||
        !can_protocol_valid_standard_frame(frame, CAN_ID_TRAJECTORY(node_id))) {
        return false;
    }

    position = (uint32_t)frame->data[0] |
               ((uint32_t)frame->data[1] << 8u) |
               ((uint32_t)frame->data[2] << 16u) |
               ((uint32_t)frame->data[3] << 24u);
    velocity = (uint16_t)frame->data[4] |
               ((uint16_t)frame->data[5] << 8u);
    out->position_mdeg = (int32_t)position;
    out->velocity_mdeg_s = (int32_t)(int16_t)velocity * 10;
    out->sequence = (uint16_t)frame->data[6] |
                    ((uint16_t)frame->data[7] << 8u);
    return true;
}

bool can_protocol_decode_broadcast(const can_frame_t *frame,
                                   can_broadcast_t *out)
{
    can_opcode_t opcode;

    if ((out == NULL) ||
        !can_protocol_valid_standard_frame(frame, CAN_ID_BROADCAST) ||
        (frame->data[1] != CAN_PROTOCOL_VERSION) ||
        (frame->data[6] != 0u) ||
        (can_protocol_crc8(frame->data, 7u) != frame->data[7])) {
        return false;
    }

    opcode = (can_opcode_t)frame->data[0];
    if (!can_protocol_valid_opcode(opcode)) {
        return false;
    }

    out->opcode = opcode;
    out->protocol_version = frame->data[1];
    out->sequence = (uint16_t)frame->data[2] |
                    ((uint16_t)frame->data[3] << 8u);
    out->session = (uint16_t)frame->data[4] |
                   ((uint16_t)frame->data[5] << 8u);
    out->flags = frame->data[6];
    return true;
}

bool can_protocol_encode_feedback(uint8_t node_id,
                                  const can_feedback_t *value,
                                  can_frame_t *out)
{
    uint32_t position;
    uint16_t velocity;

    if ((value == NULL) || (out == NULL) || !can_protocol_valid_node(node_id)) {
        return false;
    }

    position = (uint32_t)value->actual_position_mdeg;
    velocity = (uint16_t)can_protocol_velocity_to_wire(value->actual_velocity_mdeg_s);
    out->id = CAN_ID_FEEDBACK(node_id);
    out->dlc = 8u;
    out->data[0] = (uint8_t)position;
    out->data[1] = (uint8_t)(position >> 8u);
    out->data[2] = (uint8_t)(position >> 16u);
    out->data[3] = (uint8_t)(position >> 24u);
    out->data[4] = (uint8_t)velocity;
    out->data[5] = (uint8_t)(velocity >> 8u);
    out->data[6] = (uint8_t)value->applied_sequence;
    out->data[7] = (uint8_t)(value->applied_sequence >> 8u);
    return true;
}

bool can_protocol_encode_health(uint8_t node_id,
                                const can_health_t *value,
                                can_frame_t *out)
{
    if ((value == NULL) || (out == NULL) || !can_protocol_valid_node(node_id) ||
        (value->protocol_version != CAN_PROTOCOL_VERSION) ||
        (value->node_state > (uint8_t)CAN_NODE_STATE_FAULT)) {
        return false;
    }

    out->id = CAN_ID_HEALTH(node_id);
    out->dlc = 8u;
    out->data[0] = value->protocol_version;
    out->data[1] = value->node_state;
    out->data[2] = (uint8_t)value->fault_bits;
    out->data[3] = (uint8_t)(value->fault_bits >> 8u);
    out->data[4] = (uint8_t)value->session;
    out->data[5] = (uint8_t)(value->session >> 8u);
    out->data[6] = (uint8_t)value->vbus_10mv;
    out->data[7] = (uint8_t)(value->vbus_10mv >> 8u);
    return true;
}

bool can_protocol_encode_broadcast(const can_broadcast_t *value,
                                   can_frame_t *out)
{
    if ((value == NULL) || (out == NULL) || !can_protocol_valid_opcode(value->opcode) ||
        (value->protocol_version != CAN_PROTOCOL_VERSION) || (value->flags != 0u)) {
        return false;
    }

    out->id = CAN_ID_BROADCAST;
    out->dlc = 8u;
    out->data[0] = (uint8_t)value->opcode;
    out->data[1] = value->protocol_version;
    out->data[2] = (uint8_t)value->sequence;
    out->data[3] = (uint8_t)(value->sequence >> 8u);
    out->data[4] = (uint8_t)value->session;
    out->data[5] = (uint8_t)(value->session >> 8u);
    out->data[6] = value->flags;
    out->data[7] = can_protocol_crc8(out->data, 7u);
    return true;
}
