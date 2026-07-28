#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "can_frame.h"

#define CAN_PROTOCOL_VERSION       1u
#define CAN_ID_BROADCAST           0x080u
#define CAN_ID_TRAJECTORY(node)    (0x100u + (uint16_t)(node))
#define CAN_ID_FEEDBACK(node)      (0x180u + (uint16_t)(node))
#define CAN_ID_HEALTH(node)        (0x280u + (uint16_t)(node))

typedef enum {
    CAN_OPCODE_ARM = 0x01u,
    CAN_OPCODE_SYNC = 0x02u,
    CAN_OPCODE_STOP = 0x03u,
    CAN_OPCODE_CLEAR_FAULT = 0x04u,
    CAN_OPCODE_DISCOVER = 0x05u
} can_opcode_t;

typedef enum {
    CAN_NODE_STATE_UNCONFIGURED = 0u,
    CAN_NODE_STATE_READY = 1u,
    CAN_NODE_STATE_ARMED = 2u,
    CAN_NODE_STATE_RUNNING = 3u,
    CAN_NODE_STATE_HOLD = 4u,
    CAN_NODE_STATE_FAULT = 5u
} can_node_state_t;

typedef struct {
    int32_t position_mdeg;
    int32_t velocity_mdeg_s;
    uint16_t sequence;
} can_trajectory_t;

typedef struct {
    can_opcode_t opcode;
    uint8_t protocol_version;
    uint16_t sequence;
    uint16_t session;
    uint8_t flags;
} can_broadcast_t;

typedef struct {
    int32_t actual_position_mdeg;
    int32_t actual_velocity_mdeg_s;
    uint16_t applied_sequence;
} can_feedback_t;

typedef struct {
    uint8_t protocol_version;
    uint8_t node_state;
    uint16_t fault_bits;
    uint16_t session;
    uint16_t vbus_10mv;
} can_health_t;

uint8_t can_protocol_crc8(const uint8_t *data, uint8_t len);
bool can_protocol_sequence_newer(uint16_t candidate, uint16_t previous);
bool can_protocol_decode_trajectory(const can_frame_t *frame,
                                    uint8_t node_id,
                                    can_trajectory_t *out);
bool can_protocol_decode_broadcast(const can_frame_t *frame,
                                   can_broadcast_t *out);
bool can_protocol_encode_feedback(uint8_t node_id,
                                  const can_feedback_t *value,
                                  can_frame_t *out);
bool can_protocol_encode_health(uint8_t node_id,
                                const can_health_t *value,
                                can_frame_t *out);
bool can_protocol_encode_broadcast(const can_broadcast_t *value,
                                   can_frame_t *out);

#ifdef __cplusplus
}
#endif

#endif /* CAN_PROTOCOL_H */
