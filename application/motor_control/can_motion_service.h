#ifndef CAN_MOTION_SERVICE_H
#define CAN_MOTION_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "can_protocol.h"
#include "position_loop.h"

typedef struct {
    bool (*rx_pop)(can_frame_t *out);
    bool (*tx_push)(const can_frame_t *frame);
    int (*position_start)(const position_setpoint_t *setpoint);
    int (*position_submit)(const position_setpoint_t *setpoint);
    void (*position_stop)(void);
    int32_t (*position_mdeg)(void);
    int32_t (*velocity_mdeg_s)(void);
    uint16_t (*vbus_10mv)(void);
    uint32_t (*fault_get)(void);
    void (*fault_set)(uint32_t bits);
    void (*fault_clear_can)(void);
} can_motion_ops_t;

typedef struct {
    uint8_t node_id;
    can_node_state_t state;
    uint16_t session;
    uint16_t pending_sequence;
    uint16_t applied_sequence;
    uint16_t pending_age_ms;
    uint16_t sync_age_ms;
    uint32_t rx_frames;
    uint32_t tx_frames;
    uint32_t tx_failures;
    uint32_t protocol_errors;
    uint32_t fault_bits;
    bool joint_ready;
    bool pending_valid;
    bool applied_valid;
    bool position_active;
} can_motion_snapshot_t;

void can_motion_service_init(const can_motion_ops_t *ops);
void can_motion_service_set_joint_config(bool ready, uint8_t node_id);
void can_motion_service_tick_1ms(void);
void can_motion_service_poll_tx(void);
bool can_motion_service_get_snapshot(can_motion_snapshot_t *out);
/* Clears cumulative transport/protocol counters only. */
void can_motion_service_reset_diagnostics(void);
void can_motion_service_force_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_MOTION_SERVICE_H */
