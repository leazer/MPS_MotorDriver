#include "current_reconstruction.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void assert_close(float actual, float expected)
{
    assert(fabsf(actual - expected) < 1.0e-6f);
}

static current_sample_plan_t plan(uint16_t a, uint16_t b, uint16_t c)
{
    current_sample_plan_t p;
    p.duty_a = a;
    p.duty_b = b;
    p.duty_c = c;
    p.sample_tick = 5264u;
    return p;
}

static void test_tracker_pairs_previous_active_cycle(void)
{
    current_sample_tracker_t t;
    current_sample_plan_t sampled;

    current_sample_tracker_init(&t, 2812u, 5264u);
    current_sample_tracker_stage_duty(&t, 3000u, 3100u, 3200u);
    current_sample_tracker_latch_update(&t);
    current_sample_tracker_get_sampled(&t, &sampled);
    assert(sampled.duty_a == 2812u);
    assert(sampled.duty_b == 2812u);
    assert(sampled.duty_c == 2812u);

    current_sample_tracker_stage_duty(&t, 3300u, 3400u, 3500u);
    current_sample_tracker_latch_update(&t);
    current_sample_tracker_get_sampled(&t, &sampled);
    assert(sampled.duty_a == 3000u);
    assert(sampled.duty_b == 3100u);
    assert(sampled.duty_c == 3200u);

    current_sample_tracker_stage_trigger(&t, 5100u);
    current_sample_tracker_latch_update(&t);
    current_sample_tracker_get_sampled(&t, &sampled);
    assert(sampled.sample_tick == 5264u);
    current_sample_tracker_latch_update(&t);
    current_sample_tracker_get_sampled(&t, &sampled);
    assert(sampled.sample_tick == 5100u);
}

static void test_rearm_uses_values_that_loaded_while_irq_was_off(void)
{
    current_sample_tracker_t t;
    current_sample_plan_t sampled;
    current_sample_tracker_init(&t, 2812u, 5264u);
    current_sample_tracker_stage_duty(&t, 2000u, 2100u, 2200u);
    current_sample_tracker_rearm_from_next(&t);
    current_sample_tracker_get_sampled(&t, &sampled);
    assert(sampled.duty_a == 2000u);
    assert(sampled.duty_b == 2100u);
    assert(sampled.duty_c == 2200u);
}

static void test_exactly_two_valid_reconstructs_each_missing_phase(void)
{
    current_reconstruction_result_t r;

    current_reconstruction_run(&(current_sample_plan_t){5200u, 3000u, 3200u, 5264u},
                               99.0f, 0.20f, -0.05f, 180u, &r);
    assert(r.frame_valid && r.valid_mask == (CURRENT_PHASE_B_MASK | CURRENT_PHASE_C_MASK));
    assert(r.reconstructed_phase == CURRENT_RECON_PHASE_A);
    assert_close(r.ia, 0.15f);

    current_reconstruction_run(&(current_sample_plan_t){3000u, 5200u, 3200u, 5264u},
                               0.20f, 99.0f, -0.05f, 180u, &r);
    assert(r.reconstructed_phase == CURRENT_RECON_PHASE_B);
    assert_close(r.ib, 0.15f);

    current_reconstruction_run(&(current_sample_plan_t){3000u, 3200u, 5200u, 5264u},
                               0.20f, -0.05f, 99.0f, 180u, &r);
    assert(r.reconstructed_phase == CURRENT_RECON_PHASE_C);
    assert_close(r.ic, 0.15f);
}

static void test_low_side_samples_are_normalized_to_foc_phase_polarity(void)
{
    current_reconstruction_result_t r;
    current_sample_plan_t p = plan(3000u, 3200u, 3400u);

    current_reconstruction_run(&p, -0.30f, 0.10f, 99.0f, 180u, &r);

    assert(r.frame_valid);
    assert(r.valid_mask == CURRENT_PHASE_ALL_MASK);
    assert(r.reconstructed_phase == CURRENT_RECON_PHASE_C);
    assert_close(r.raw_ia, -0.30f);
    assert_close(r.raw_ib, 0.10f);
    assert_close(r.raw_ic, 99.0f);
    assert_close(r.ia, 0.30f);
    assert_close(r.ib, -0.10f);
    assert_close(r.ic, -0.20f);
    assert_close(r.ia + r.ib + r.ic, 0.0f);
}

static void test_all_valid_drops_smallest_margin_for_all_six_orders(void)
{
    static const uint16_t duty[6][3] = {
        {3000u, 3200u, 3400u}, {3000u, 3400u, 3200u},
        {3200u, 3000u, 3400u}, {3400u, 3000u, 3200u},
        {3200u, 3400u, 3000u}, {3400u, 3200u, 3000u}
    };
    static const current_reconstructed_phase_t expected[6] = {
        CURRENT_RECON_PHASE_C, CURRENT_RECON_PHASE_B,
        CURRENT_RECON_PHASE_C, CURRENT_RECON_PHASE_A,
        CURRENT_RECON_PHASE_B, CURRENT_RECON_PHASE_A
    };
    current_reconstruction_result_t r;
    unsigned i;

    for (i = 0u; i < 6u; ++i) {
        current_sample_plan_t p = plan(duty[i][0], duty[i][1], duty[i][2]);
        current_reconstruction_run(&p, 0.20f, -0.05f, 17.0f, 180u, &r);
        assert(r.frame_valid);
        assert(r.valid_mask == CURRENT_PHASE_ALL_MASK);
        assert(r.reconstructed_phase == expected[i]);
        assert_close(r.ia + r.ib + r.ic, 0.0f);
    }
}

static void test_fewer_than_two_valid_is_invalid(void)
{
    current_reconstruction_result_t r;
    current_sample_plan_t p = plan(3000u, 5200u, 5200u);
    current_reconstruction_run(&p, 0.2f, 0.3f, 0.4f, 180u, &r);
    assert(!r.frame_valid);
    assert(r.reconstructed_phase == CURRENT_RECON_PHASE_NONE);
}

static void test_guard_debounce_and_reset(void)
{
    current_sample_guard_t g;
    current_sample_action_t action;
    unsigned i;

    current_sample_guard_init(&g);
    for (i = 0u; i < 7u; ++i) {
        action = current_sample_guard_step(&g, false, false, 4u, 8u);
        assert(action == CURRENT_SAMPLE_ACTION_HOLD);
    }
    action = current_sample_guard_step(&g, false, false, 4u, 8u);
    assert(action == CURRENT_SAMPLE_ACTION_TRIP_INVALID);
    assert(g.invalid_total == 8u && g.invalid_consecutive == 8u);

    current_sample_guard_reset_consecutive(&g);
    for (i = 0u; i < 3u; ++i) {
        assert(current_sample_guard_step(&g, true, true, 4u, 8u) == CURRENT_SAMPLE_ACTION_USE);
    }
    assert(current_sample_guard_step(&g, true, true, 4u, 8u) ==
           CURRENT_SAMPLE_ACTION_TRIP_OVERCURRENT);
    assert(g.overcurrent_consecutive == 4u);

    assert(current_sample_guard_step(&g, true, false, 4u, 8u) == CURRENT_SAMPLE_ACTION_USE);
    assert(g.overcurrent_consecutive == 0u && g.invalid_consecutive == 0u);
}

int main(void)
{
    test_tracker_pairs_previous_active_cycle();
    test_rearm_uses_values_that_loaded_while_irq_was_off();
    test_low_side_samples_are_normalized_to_foc_phase_polarity();
    test_exactly_two_valid_reconstructs_each_missing_phase();
    test_all_valid_drops_smallest_margin_for_all_six_orders();
    test_fewer_than_two_valid_is_invalid();
    test_guard_debounce_and_reset();
    puts("current reconstruction: all tests passed");
    return 0;
}
