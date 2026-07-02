#include "encoder_service.h"
#include "motor_encoder_at32m412.h"
#include "motor_params.h"
#include <string.h>

#define ENC_MAX_DELTA_PER_TICK      1024
#define ENC_CONSEC_ERROR_THRESHOLD  32u
#define TWO_PI_F                    6.28318530718f

static volatile encoder_snapshot_t s_snapshot;
static uint16_t s_zero_raw;
static bool s_cal_valid;
static bool s_has_prev;
static uint32_t s_consec_error_count;
static int16_t s_cal_table[CAL_TABLE_POINTS];

static int16_t encoder_raw_delta(uint16_t prev, uint16_t now)
{
    return (int16_t)(now - prev);
}

static uint16_t encoder_apply_calibration(uint16_t raw)
{
    uint16_t idx;
    uint16_t frac;
    int16_t off0;
    int16_t off1;
    int32_t off_mdeg;
    int32_t off_raw;
    uint32_t idx_frac_q24;

    if (!s_cal_valid) {
        return raw;
    }

    idx_frac_q24 = (uint32_t)raw * 256u;
    idx = (uint16_t)(idx_frac_q24 >> 16);
    frac = (uint16_t)(idx_frac_q24 & 0xFFFFu);
    off0 = s_cal_table[idx];
    off1 = s_cal_table[(uint16_t)((idx + 1u) & 0xFFu)];
    off_mdeg = (int32_t)off0 + (((int32_t)(off1 - off0) * (int32_t)frac) >> 16);
    off_raw = (off_mdeg * 65536) / 360000;
    return (uint16_t)((int32_t)raw - off_raw);
}

static int32_t encoder_mech_mdeg_from_raw(uint16_t raw)
{
    return (int32_t)((uint32_t)raw * 360000u / 65536u);
}

static int32_t encoder_elec_mrad_from_raw(uint16_t raw)
{
    uint16_t corrected;
    uint16_t mech_diff;
    float theta;

    corrected = encoder_apply_calibration(raw);
    mech_diff = (uint16_t)(corrected - s_zero_raw);
    theta = ((float)mech_diff * (float)MOTOR_POLE_PAIRS * TWO_PI_F) / 65536.0f;
    while (theta >= TWO_PI_F) {
        theta -= TWO_PI_F;
    }
    while (theta < 0.0f) {
        theta += TWO_PI_F;
    }
    return (int32_t)(theta * 1000.0f);
}

static int encoder_accept_sample(uint16_t raw, int16_t speed, int16_t delta)
{
    if (s_has_prev) {
        s_snapshot.raw_unwrapped += delta;
    } else {
        s_snapshot.raw_unwrapped = (int32_t)raw;
        s_has_prev = true;
    }

    s_snapshot.raw_delta = delta;
    s_snapshot.raw16 = raw;
    s_snapshot.speed_raw = speed;
    s_snapshot.mech_mdeg = encoder_mech_mdeg_from_raw(raw);
    s_snapshot.elec_mrad = encoder_elec_mrad_from_raw(raw);
    s_snapshot.valid = 1u;
    s_snapshot.fresh = 1u;
    s_snapshot.spike_rejected = 0u;
    s_snapshot.accept_count++;
    s_consec_error_count = 0u;
    return 0;
}

void encoder_service_init(void)
{
    memset((void *)&s_snapshot, 0, sizeof(s_snapshot));
    memset(s_cal_table, 0, sizeof(s_cal_table));
    s_zero_raw = 0u;
    s_cal_valid = false;
    s_has_prev = false;
    s_consec_error_count = 0u;
}

int encoder_service_update_from_isr(void)
{
    uint16_t raw = 0u;
    int16_t speed = 0;
    int16_t delta = 0;

    s_snapshot.sample_count++;
    if (motor_encoder_read_raw_frame(&raw, &speed) != 0) {
        s_snapshot.bus_error_count++;
        s_consec_error_count++;
        return -1;
    }

    if (s_has_prev) {
        delta = encoder_raw_delta(s_snapshot.raw16, raw);
        if ((delta > ENC_MAX_DELTA_PER_TICK) || (delta < -ENC_MAX_DELTA_PER_TICK)) {
            s_snapshot.spike_count++;
            s_snapshot.spike_rejected = 1u;
            s_snapshot.last_rejected_raw16 = raw;
            s_snapshot.last_rejected_delta = delta;
            s_consec_error_count++;
            if (s_consec_error_count >= ENC_CONSEC_ERROR_THRESHOLD) {
                s_has_prev = false;
                return encoder_accept_sample(raw, speed, 0);
            }
            return -2;
        }
    }

    return encoder_accept_sample(raw, speed, delta);
}

int encoder_service_poll_once_thread(void)
{
    s_has_prev = false;
    return encoder_service_update_from_isr();
}

bool encoder_service_get_snapshot(encoder_snapshot_t *out)
{
    if (out == 0) {
        return false;
    }
    *out = s_snapshot;
    if (s_snapshot.fresh == 0u) {
        s_snapshot.stale_count++;
    }
    s_snapshot.fresh = 0u;
    return s_snapshot.valid != 0u;
}

float encoder_service_get_electrical_angle_rad(void)
{
    return ((float)s_snapshot.elec_mrad) / 1000.0f;
}

uint16_t encoder_service_get_raw16(void)
{
    return s_snapshot.raw16;
}

void encoder_service_set_zero(uint16_t raw)
{
    s_zero_raw = raw;
}

uint16_t encoder_service_get_zero(void)
{
    return s_zero_raw;
}

void encoder_service_set_calibration_table(const int16_t *table, bool valid)
{
    if (table != 0) {
        memcpy(s_cal_table, table, sizeof(s_cal_table));
    } else {
        memset(s_cal_table, 0, sizeof(s_cal_table));
    }
    s_cal_valid = valid;
}

void encoder_service_reset_diagnostics(void)
{
    uint16_t raw;
    int32_t unwrapped;
    uint8_t valid;

    raw = s_snapshot.raw16;
    unwrapped = s_snapshot.raw_unwrapped;
    valid = s_snapshot.valid;
    memset((void *)&s_snapshot, 0, sizeof(s_snapshot));
    s_snapshot.raw16 = raw;
    s_snapshot.raw_unwrapped = unwrapped;
    s_snapshot.valid = valid;
    s_has_prev = false;
    s_consec_error_count = 0u;
}
