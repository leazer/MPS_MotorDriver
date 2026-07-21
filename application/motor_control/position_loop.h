#ifndef POSITION_LOOP_H
#define POSITION_LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int32_t position_mdeg;
    int32_t velocity_mdeg_s;
    uint16_t sequence;
    uint16_t lease_ms;
} position_setpoint_t;

typedef struct {
    int32_t target_position_mdeg;
    int32_t velocity_ff_mdeg_s;
    int32_t reference_position_mdeg;
    int32_t measured_position_mdeg;
    int32_t error_mdeg;
    int32_t speed_ref_elec_mrad_s;
    uint16_t sequence;
    uint16_t age_ms;
    uint8_t origin_valid;
    uint8_t active;
    uint8_t stream_timeout;
    uint8_t tracking_fault;
} position_loop_snapshot_t;

void position_loop_init(void);
void position_loop_reset(void);
void position_loop_set_origin(int32_t sensor_mdeg, int32_t joint_mdeg);
bool position_loop_origin_valid(void);
int32_t position_loop_sensor_to_joint_mdeg(int32_t sensor_mdeg);
bool position_loop_submit(const position_setpoint_t *setpoint);
float position_loop_run(int32_t sensor_mdeg);
float position_loop_get_iq_feedforward_A(void);
bool position_loop_get_snapshot(position_loop_snapshot_t *out);

#ifdef __cplusplus
}
#endif

#endif /* POSITION_LOOP_H */
