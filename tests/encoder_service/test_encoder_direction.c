#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "encoder_service.h"
#include "motor_params.h"

void encoder_tracker_update_sample(uint16_t raw16, uint8_t valid)
{
    (void)raw16;
    (void)valid;
}

void motor_control_isr_on_encoder_sample(uint16_t raw16)
{
    (void)raw16;
}

int motor_encoder_read_raw_frame(uint16_t *raw, int16_t *speed)
{
    (void)raw;
    (void)speed;
    return -1;
}

int motor_encoder_read_angle_raw(uint16_t *raw)
{
    (void)raw;
    return -1;
}

static void test_decreasing_raw_is_positive_control_direction(void)
{
    int16_t zero_table[CAL_TABLE_POINTS];
    encoder_snapshot_t snap;
    int i;

    memset(zero_table, 0, sizeof(zero_table));
    encoder_service_init();
    encoder_service_set_zero(1000u);
    encoder_service_set_calibration_table(zero_table, true);

    assert(encoder_service_update_sample(900u, 0, 1u) == 0);
    assert(encoder_service_get_snapshot(&snap));
    assert(snap.raw_elec_mrad >= 66);
    assert(snap.raw_elec_mrad <= 68);
    assert(snap.elec_mrad == snap.raw_elec_mrad);

    for (i = 1; i <= 40; ++i) {
        assert(encoder_service_update_sample((uint16_t)(900 - i * 10), 0, 1u) == 0);
    }
    assert(encoder_service_get_snapshot(&snap));
    assert(snap.speed_mech_mrad_s > 0);
    assert(snap.speed_elec_mrad_s == snap.speed_mech_mrad_s * 7);
}

static void test_direction_normalization_handles_raw_wrap(void)
{
    int16_t zero_table[CAL_TABLE_POINTS];
    encoder_snapshot_t snap;

    memset(zero_table, 0, sizeof(zero_table));
    encoder_service_init();
    encoder_service_set_zero(10u);
    encoder_service_set_calibration_table(zero_table, true);

    assert(encoder_service_update_sample(65530u, 0, 1u) == 0);
    assert(encoder_service_get_snapshot(&snap));
    assert(snap.elec_mrad >= 10);
    assert(snap.elec_mrad <= 11);
}

static void test_control_position_is_continuous_and_direction_normalized(void)
{
    int16_t zero_table[CAL_TABLE_POINTS];
    encoder_snapshot_t snap;
    int32_t previous;
    const uint16_t samples[] = {20u, 10u, 0u, 65530u, 65520u};
    size_t i;

    memset(zero_table, 0, sizeof(zero_table));
    encoder_service_init();
    encoder_service_set_zero(1000u);
    encoder_service_set_calibration_table(zero_table, true);

    assert(encoder_service_update_sample(900u, 0, 1u) == 0);
    assert(encoder_service_get_snapshot(&snap));
    assert(snap.control_position_mdeg >= 548);
    assert(snap.control_position_mdeg <= 550);
    assert(encoder_service_get_control_position_mdeg() ==
           snap.control_position_mdeg);

    encoder_service_init();
    encoder_service_set_zero(20u);
    encoder_service_set_calibration_table(zero_table, true);
    previous = -1;
    for (i = 0u; i < sizeof(samples) / sizeof(samples[0]); ++i) {
        assert(encoder_service_update_sample(samples[i], 0, 1u) == 0);
        assert(encoder_service_get_snapshot(&snap));
        assert(snap.control_position_mdeg >= previous);
        assert(snap.control_position_mdeg < 1000);
        previous = snap.control_position_mdeg;
    }
}

int main(void)
{
    test_decreasing_raw_is_positive_control_direction();
    test_direction_normalization_handles_raw_wrap();
    test_control_position_is_continuous_and_direction_normalized();
    printf("encoder direction tests passed\n");
    return 0;
}
