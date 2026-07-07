#ifndef ENCODER_SERVICE_H
#define ENCODER_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t raw16;
    int32_t  raw_unwrapped;
    int16_t  raw_delta;
    int32_t  mech_mdeg;
    int32_t  elec_mrad;
    int16_t  speed_raw;
    uint16_t last_rejected_raw16;
    int16_t  last_rejected_delta;
    uint8_t  valid;
    uint8_t  fresh;
    uint8_t  spike_rejected;
    uint32_t sample_count;
    uint32_t accept_count;
    uint32_t bus_error_count;
    uint32_t spike_count;
    uint32_t stale_count;
} encoder_snapshot_t;

void encoder_service_init(void);
int  encoder_service_update_from_isr(void);
int  encoder_service_update_sample(uint16_t raw, int16_t speed, uint8_t bus_ok);
int  encoder_service_acquire_once(void);
int  encoder_service_poll_once_thread(void);
bool encoder_service_get_snapshot(encoder_snapshot_t *out);
float encoder_service_get_electrical_angle_rad(void);
uint16_t encoder_service_get_raw16(void);
void encoder_service_set_zero(uint16_t raw);
uint16_t encoder_service_get_zero(void);
void encoder_service_set_calibration_table(const int16_t *table, bool valid);
void encoder_service_reset_diagnostics(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_SERVICE_H */
