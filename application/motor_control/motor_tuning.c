#include "motor_tuning.h"

#include <string.h>

#include "motor_params.h"

volatile motor_tuning_t g_motor_tuning;
volatile motor_loop_debug_t g_motor_loop_debug;

void motor_tuning_init(void)
{
    g_motor_tuning.current.id_kp = PID_ID_KP;
    g_motor_tuning.current.id_ki = PID_ID_KI;
    g_motor_tuning.current.iq_kp = PID_IQ_KP;
    g_motor_tuning.current.iq_ki = PID_IQ_KI;
    g_motor_tuning.current.id_integral_limit_v =
        PID_CURRENT_INTEGRAL_LIMIT;
    g_motor_tuning.current.iq_integral_limit_v =
        PID_CURRENT_INTEGRAL_LIMIT;
    g_motor_tuning.current.id_output_limit_v = PID_CURRENT_OUT_LIMIT;
    g_motor_tuning.current.iq_output_limit_v = PID_CURRENT_OUT_LIMIT;

    g_motor_tuning.speed.kp = PID_SPEED_KP;
    g_motor_tuning.speed.kp_brake = PID_SPEED_KP_BRAKE;
    g_motor_tuning.speed.ki = PID_SPEED_KI;
    g_motor_tuning.speed.integral_limit_A = PID_SPEED_INTEGRAL_LIMIT;
    g_motor_tuning.speed.output_limit_A = PID_SPEED_OUT_LIMIT;
    g_motor_tuning.speed.friction_A = SPEED_IQ_FRICTION_A;
    g_motor_tuning.speed.ramp_rad_s2 = 50.0f;

    g_motor_tuning.position.kp = PID_POSITION_KP;
    g_motor_tuning.position.speed_limit_elec_rad_s =
        POSITION_SPEED_LIMIT_ELEC_RAD_S;
    g_motor_tuning.position.max_velocity_mdeg_s =
        POSITION_MAX_VELOCITY_MDEG_S;
    g_motor_tuning.position.max_error_mdeg = POSITION_MAX_ERROR_MDEG;
    g_motor_tuning.position.command_limit_mdeg =
        POSITION_COMMAND_LIMIT_MDEG;
    g_motor_tuning.position.extrapolation_limit_ms =
        POSITION_EXTRAPOLATION_LIMIT_MS;
    g_motor_tuning.position.iq_friction_A = POSITION_IQ_FRICTION_A;
    g_motor_tuning.position.iq_friction_moving_A =
        POSITION_IQ_FRICTION_MOVING_A;
    g_motor_tuning.position.iq_friction_error_mdeg =
        POSITION_IQ_FRICTION_ERROR_MDEG;

    g_motor_tuning.protection.phase_overcurrent_A = IQ_OVERCURRENT_A;
    g_motor_tuning.protection.overcurrent_debounce_ticks =
        OVERCURRENT_DEBOUNCE_TICKS;
    g_motor_tuning.protection.imbalance_threshold_A =
        IMBALANCE_THRESHOLD_A;
    g_motor_tuning.protection.imbalance_debounce_ticks =
        IMBALANCE_DEBOUNCE_TICKS;
    g_motor_tuning.protection.sample_blanking_ticks =
        CURRENT_SAMPLE_BLANKING_TICKS;
    g_motor_tuning.protection.sample_invalid_limit =
        CURRENT_SAMPLE_INVALID_LIMIT;

    memset((void *)&g_motor_loop_debug, 0, sizeof(g_motor_loop_debug));
}
