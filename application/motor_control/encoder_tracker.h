#ifndef ENCODER_TRACKER_H
#define ENCODER_TRACKER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t raw16;
    int32_t  elec_mrad;
    int32_t  speed_mrad_s;
    uint32_t sample_count;
    uint32_t stale_ticks;
    uint8_t  valid;
} encoder_tracker_snapshot_t;

void encoder_tracker_init(void);
void encoder_tracker_reset(void);
void encoder_tracker_update_sample(uint16_t raw16, uint8_t valid);
void encoder_tracker_tick(void);
float encoder_tracker_get_electrical_angle_rad(void);
float encoder_tracker_get_speed_rad_s(void);
uint32_t encoder_tracker_get_sample_age_ticks(void);
bool encoder_tracker_get_snapshot(encoder_tracker_snapshot_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_TRACKER_H */
