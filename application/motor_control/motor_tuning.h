#ifndef MOTOR_TUNING_H
#define MOTOR_TUNING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    float id_kp;
    float id_ki;
    float iq_kp;
    float iq_ki;
    float id_integral_limit_v;
    float iq_integral_limit_v;
    float id_output_limit_v;
    float iq_output_limit_v;
} motor_current_tuning_t;

typedef struct {
    float kp;
    float kp_brake;
    float ki;
    float integral_limit_A;
    float output_limit_A;
    float friction_A;
    float ramp_rad_s2;
} motor_speed_tuning_t;

typedef struct {
    float kp;
    float speed_limit_elec_rad_s;
    int32_t max_velocity_mdeg_s;
    int32_t max_error_mdeg;
    int32_t command_limit_mdeg;
    uint16_t extrapolation_limit_ms;
    float iq_friction_A;
    float iq_friction_moving_A;
    int32_t iq_friction_error_mdeg;
} motor_position_tuning_t;

typedef struct {
    float phase_overcurrent_A;
    uint16_t overcurrent_debounce_ticks;
    float imbalance_threshold_A;
    uint16_t imbalance_debounce_ticks;
    uint16_t sample_blanking_ticks;
    uint16_t sample_invalid_limit;
} motor_protection_tuning_t;

typedef struct {
    motor_current_tuning_t current;
    motor_speed_tuning_t speed;
    motor_position_tuning_t position;
    motor_protection_tuning_t protection;
} motor_tuning_t;

typedef struct {
    float id_ref_A;
    float iq_ref_A;
    float id_measured_A;
    float iq_measured_A;
    float id_error_A;
    float iq_error_A;
    float id_integral_v;
    float iq_integral_v;
    float vd_unlimited_v;
    float vq_unlimited_v;
    float vd_output_v;
    float vq_output_v;
    uint8_t id_saturated;
    uint8_t iq_saturated;
} motor_current_loop_debug_t;

typedef struct {
    float target_rad_s;
    float command_rad_s;
    float measured_rad_s;
    float error_rad_s;
    float integral_A;
    float active_kp;
    float friction_A;
    float iq_unlimited_A;
    float iq_output_A;
    uint8_t saturated;
} motor_speed_loop_debug_t;

typedef struct {
    int32_t target_position_mdeg;
    int32_t reference_position_mdeg;
    int32_t measured_position_mdeg;
    int32_t error_mdeg;
    int32_t velocity_ff_mdeg_s;
    float proportional_velocity_mdeg_s;
    float speed_unlimited_elec_rad_s;
    float speed_output_elec_rad_s;
    float iq_feedforward_A;
} motor_position_loop_debug_t;

typedef struct {
    float max_phase_current_A;
    float imbalance_A;
    uint16_t overcurrent_consecutive;
    uint16_t invalid_consecutive;
    uint16_t imbalance_consecutive;
    uint32_t invalid_total;
    uint8_t frame_valid;
    uint8_t overcurrent_active;
} motor_protection_debug_t;

typedef struct {
    motor_current_loop_debug_t current;
    motor_speed_loop_debug_t speed;
    motor_position_loop_debug_t position;
    motor_protection_debug_t protection;
} motor_loop_debug_t;

extern volatile motor_tuning_t g_motor_tuning;
extern volatile motor_loop_debug_t g_motor_loop_debug;

void motor_tuning_init(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_TUNING_H */
