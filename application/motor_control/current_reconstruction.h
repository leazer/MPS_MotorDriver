#ifndef CURRENT_RECONSTRUCTION_H
#define CURRENT_RECONSTRUCTION_H

#include <stdbool.h>
#include <stdint.h>

#define CURRENT_PHASE_A_MASK   (1u << 0)
#define CURRENT_PHASE_B_MASK   (1u << 1)
#define CURRENT_PHASE_C_MASK   (1u << 2)
#define CURRENT_PHASE_ALL_MASK (CURRENT_PHASE_A_MASK | CURRENT_PHASE_B_MASK | CURRENT_PHASE_C_MASK)

typedef enum {
    CURRENT_RECON_PHASE_NONE = 0u,
    CURRENT_RECON_PHASE_A = 1u,
    CURRENT_RECON_PHASE_B = 2u,
    CURRENT_RECON_PHASE_C = 3u
} current_reconstructed_phase_t;

typedef struct {
    uint16_t duty_a;
    uint16_t duty_b;
    uint16_t duty_c;
    uint16_t sample_tick;
} current_sample_plan_t;

typedef struct {
    current_sample_plan_t sampled;
    current_sample_plan_t active;
    current_sample_plan_t next;
} current_sample_tracker_t;

typedef struct {
    float raw_ia;  /* calibrated SOA value, low-side device-current sign */
    float raw_ib;  /* calibrated SOB value, low-side device-current sign */
    float raw_ic;  /* calibrated SOC value, low-side device-current sign */
    float ia;      /* corrected motor phase A current, FOC sign convention */
    float ib;      /* corrected motor phase B current, FOC sign convention */
    float ic;      /* corrected motor phase C current, FOC sign convention */
    uint16_t margin_a;
    uint16_t margin_b;
    uint16_t margin_c;
    uint8_t valid_mask;
    current_reconstructed_phase_t reconstructed_phase;
    bool frame_valid;
} current_reconstruction_result_t;

typedef enum {
    CURRENT_SAMPLE_ACTION_USE = 0,
    CURRENT_SAMPLE_ACTION_HOLD,
    CURRENT_SAMPLE_ACTION_TRIP_OVERCURRENT,
    CURRENT_SAMPLE_ACTION_TRIP_INVALID
} current_sample_action_t;

typedef struct {
    uint16_t overcurrent_consecutive;
    uint16_t invalid_consecutive;
    uint32_t invalid_total;
} current_sample_guard_t;

void current_sample_tracker_init(current_sample_tracker_t *tracker,
                                 uint16_t initial_duty,
                                 uint16_t initial_sample_tick);
void current_sample_tracker_stage_duty(current_sample_tracker_t *tracker,
                                       uint16_t duty_a, uint16_t duty_b, uint16_t duty_c);
void current_sample_tracker_stage_trigger(current_sample_tracker_t *tracker,
                                          uint16_t sample_tick);
void current_sample_tracker_latch_update(current_sample_tracker_t *tracker);
void current_sample_tracker_rearm_from_next(current_sample_tracker_t *tracker);
void current_sample_tracker_get_sampled(const current_sample_tracker_t *tracker,
                                        current_sample_plan_t *out);

void current_reconstruction_run(const current_sample_plan_t *plan,
                                float raw_ia, float raw_ib, float raw_ic,
                                uint16_t blanking_ticks,
                                current_reconstruction_result_t *out);

void current_sample_guard_init(current_sample_guard_t *guard);
void current_sample_guard_reset_consecutive(current_sample_guard_t *guard);
current_sample_action_t current_sample_guard_step(current_sample_guard_t *guard,
                                                  bool frame_valid,
                                                  bool overcurrent,
                                                  uint16_t overcurrent_limit,
                                                  uint16_t invalid_limit);

#endif
