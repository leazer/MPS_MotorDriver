#include "encoder_tracker.h"
#include "board_motor_pins.h"
#include "encoder_acq_timer_at32m412.h"
#include "motor_encoder_at32m412.h"
#include "motor_params.h"
#include <string.h>

#define TWO_PI_F      6.28318530718f
#define ISR_DT_S      (1.0f / (float)PWM_FREQUENCY_HZ)
#define ACQ_DT_S      (1.0f / (float)ENCODER_ACQ_TIMER_HZ)
#define TRACKER_KP    0.20f
#define TRACKER_KI    200.0f

static float s_theta_e;
static float s_omega_e;
static uint16_t s_last_raw16;
static uint32_t s_sample_count;
static uint32_t s_age_ticks;
static uint8_t s_valid;

static float wrap_pm_pi(float x)
{
    while (x > 3.14159265359f) {
        x -= TWO_PI_F;
    }
    while (x < -3.14159265359f) {
        x += TWO_PI_F;
    }
    return x;
}

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
    float err;

    if (valid == 0u) {
        return;
    }

    measured = motor_encoder_to_electrical_angle(raw16);
    if (s_valid == 0u) {
        s_theta_e = measured;
        s_omega_e = 0.0f;
        s_valid = 1u;
    } else {
        err = wrap_pm_pi(measured - s_theta_e);
        s_theta_e += TRACKER_KP * err;
        s_omega_e += TRACKER_KI * err * ACQ_DT_S;
        s_theta_e = wrap_0_2pi(s_theta_e);
    }

    s_last_raw16 = raw16;
    s_sample_count++;
    s_age_ticks = 0u;
}

void encoder_tracker_tick(void)
{
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
