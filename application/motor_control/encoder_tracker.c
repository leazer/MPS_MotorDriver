#include "encoder_tracker.h"
#include "board_motor_pins.h"
#include "encoder_acq_timer_at32m412.h"
#include "encoder_service.h"
#include "motor_encoder_at32m412.h"
#include "motor_params.h"
#include <string.h>

#define TWO_PI_F      6.28318530718f
#define ISR_DT_S      (1.0f / (float)PWM_FREQUENCY_HZ)

static float s_theta_e;
static float s_omega_e;
static uint16_t s_last_raw16;
static uint32_t s_sample_count;
static uint32_t s_age_ticks;
static uint8_t s_valid;

static float wrap_0_2pi(float x)
{
    while (x >= TWO_PI_F) {
        x -= TWO_PI_F;
    }
    while (x < 0.0f) {
        x += TWO_PI_F;
    }
    return x;
}

void encoder_tracker_init(void)
{
    encoder_tracker_reset();
}

void encoder_tracker_reset(void)
{
    s_theta_e = 0.0f;
    s_omega_e = 0.0f;
    s_last_raw16 = 0u;
    s_sample_count = 0u;
    s_age_ticks = 0u;
    s_valid = 0u;
}

void encoder_tracker_update_sample(uint16_t raw16, uint8_t valid)
{
    float measured;

    if (valid == 0u) {
        return;
    }

    measured = encoder_service_get_electrical_angle_rad();
    s_theta_e = wrap_0_2pi(measured);
    s_omega_e = encoder_service_get_speed_electrical_rad_s();
    s_valid = 1u;

    s_last_raw16 = raw16;
    s_sample_count++;
    s_age_ticks = 0u;
}

void encoder_tracker_tick(void)
{
    if (s_valid == 0u) {
        return;
    }
    s_theta_e = wrap_0_2pi(s_theta_e + (s_omega_e * ISR_DT_S));
    if (s_age_ticks < 0xFFFFFFFFu) {
        s_age_ticks++;
    }
}

float encoder_tracker_get_electrical_angle_rad(void)
{
    return s_theta_e;
}

float encoder_tracker_get_speed_rad_s(void)
{
    return s_omega_e;
}

uint32_t encoder_tracker_get_sample_age_ticks(void)
{
    return s_age_ticks;
}

bool encoder_tracker_get_snapshot(encoder_tracker_snapshot_t *out)
{
    if (out == 0) {
        return false;
    }
    out->raw16 = s_last_raw16;
    out->elec_mrad = (int32_t)(s_theta_e * 1000.0f);
    out->speed_mrad_s = (int32_t)(s_omega_e * 1000.0f);
    out->sample_count = s_sample_count;
    out->stale_ticks = s_age_ticks;
    out->valid = s_valid;
    return s_valid != 0u;
}
