#include "current_reconstruction.h"
#include <string.h>

static uint16_t margin_ticks(uint16_t sample_tick, uint16_t duty)
{
    return sample_tick > duty ? (uint16_t)(sample_tick - duty) : 0u;
}

static bool margin_valid(uint16_t margin, uint16_t blanking_ticks)
{
    return margin >= blanking_ticks;
}

void current_sample_tracker_init(current_sample_tracker_t *t,
                                 uint16_t initial_duty,
                                 uint16_t initial_sample_tick)
{
    current_sample_plan_t p;
    p.duty_a = initial_duty;
    p.duty_b = initial_duty;
    p.duty_c = initial_duty;
    p.sample_tick = initial_sample_tick;
    t->sampled = p;
    t->active = p;
    t->next = p;
}

void current_sample_tracker_stage_duty(current_sample_tracker_t *t,
                                       uint16_t a, uint16_t b, uint16_t c)
{
    t->next.duty_a = a;
    t->next.duty_b = b;
    t->next.duty_c = c;
}

void current_sample_tracker_stage_trigger(current_sample_tracker_t *t, uint16_t tick)
{
    t->next.sample_tick = tick;
}

void current_sample_tracker_latch_update(current_sample_tracker_t *t)
{
    t->sampled = t->active;
    t->active = t->next;
}

void current_sample_tracker_rearm_from_next(current_sample_tracker_t *t)
{
    t->sampled = t->next;
    t->active = t->next;
}

void current_sample_tracker_get_sampled(const current_sample_tracker_t *t,
                                        current_sample_plan_t *out)
{
    *out = t->sampled;
}

void current_reconstruction_run(const current_sample_plan_t *plan,
                                float raw_ia, float raw_ib, float raw_ic,
                                uint16_t blanking_ticks,
                                current_reconstruction_result_t *out)
{
    uint8_t valid_count;
    uint16_t smallest_margin;

    memset(out, 0, sizeof(*out));
    out->raw_ia = raw_ia;
    out->raw_ib = raw_ib;
    out->raw_ic = raw_ic;
    out->margin_a = margin_ticks(plan->sample_tick, plan->duty_a);
    out->margin_b = margin_ticks(plan->sample_tick, plan->duty_b);
    out->margin_c = margin_ticks(plan->sample_tick, plan->duty_c);

    if (margin_valid(out->margin_a, blanking_ticks)) out->valid_mask |= CURRENT_PHASE_A_MASK;
    if (margin_valid(out->margin_b, blanking_ticks)) out->valid_mask |= CURRENT_PHASE_B_MASK;
    if (margin_valid(out->margin_c, blanking_ticks)) out->valid_mask |= CURRENT_PHASE_C_MASK;

    valid_count = 0u;
    if ((out->valid_mask & CURRENT_PHASE_A_MASK) != 0u) valid_count++;
    if ((out->valid_mask & CURRENT_PHASE_B_MASK) != 0u) valid_count++;
    if ((out->valid_mask & CURRENT_PHASE_C_MASK) != 0u) valid_count++;
    if (valid_count < 2u) return;

    if (valid_count == 2u) {
        if ((out->valid_mask & CURRENT_PHASE_A_MASK) == 0u) {
            out->reconstructed_phase = CURRENT_RECON_PHASE_A;
        } else if ((out->valid_mask & CURRENT_PHASE_B_MASK) == 0u) {
            out->reconstructed_phase = CURRENT_RECON_PHASE_B;
        } else {
            out->reconstructed_phase = CURRENT_RECON_PHASE_C;
        }
    } else {
        out->reconstructed_phase = CURRENT_RECON_PHASE_C;
        smallest_margin = out->margin_c;
        if (out->margin_b < smallest_margin) {
            smallest_margin = out->margin_b;
            out->reconstructed_phase = CURRENT_RECON_PHASE_B;
        }
        if (out->margin_a < smallest_margin) {
            out->reconstructed_phase = CURRENT_RECON_PHASE_A;
        }
    }

    out->ia = raw_ia;
    out->ib = raw_ib;
    out->ic = raw_ic;
    switch (out->reconstructed_phase) {
    case CURRENT_RECON_PHASE_A:
        out->ia = -(out->ib + out->ic);
        break;
    case CURRENT_RECON_PHASE_B:
        out->ib = -(out->ia + out->ic);
        break;
    case CURRENT_RECON_PHASE_C:
        out->ic = -(out->ia + out->ib);
        break;
    default:
        return;
    }
    out->frame_valid = true;
}

void current_sample_guard_init(current_sample_guard_t *g)
{
    memset(g, 0, sizeof(*g));
}

void current_sample_guard_reset_consecutive(current_sample_guard_t *g)
{
    g->overcurrent_consecutive = 0u;
    g->invalid_consecutive = 0u;
}

current_sample_action_t current_sample_guard_step(current_sample_guard_t *g,
                                                  bool frame_valid,
                                                  bool overcurrent,
                                                  uint16_t oc_limit,
                                                  uint16_t invalid_limit)
{
    if (!frame_valid) {
        g->overcurrent_consecutive = 0u;
        g->invalid_total++;
        if (g->invalid_consecutive < UINT16_MAX) g->invalid_consecutive++;
        if (g->invalid_consecutive >= invalid_limit) {
            return CURRENT_SAMPLE_ACTION_TRIP_INVALID;
        }
        return CURRENT_SAMPLE_ACTION_HOLD;
    }

    g->invalid_consecutive = 0u;
    if (!overcurrent) {
        g->overcurrent_consecutive = 0u;
        return CURRENT_SAMPLE_ACTION_USE;
    }
    if (g->overcurrent_consecutive < UINT16_MAX) g->overcurrent_consecutive++;
    if (g->overcurrent_consecutive >= oc_limit) {
        return CURRENT_SAMPLE_ACTION_TRIP_OVERCURRENT;
    }
    return CURRENT_SAMPLE_ACTION_USE;
}
