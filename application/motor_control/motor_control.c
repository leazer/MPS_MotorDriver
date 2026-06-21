#include "motor_control.h"

static void motor_control_clear_targets(motor_control_t *control)
{
    control->iq_ref_ma = 0;
    control->speed_ref_rpm = 0;
    control->position_ref_mdeg = 0;
}

void motor_control_init(motor_control_t *control)
{
    if(control == 0)
    {
        return;
    }

    control->state = MOTOR_CONTROL_STATE_DISABLED;
    control->mode = MOTOR_CONTROL_MODE_OPEN_LOOP;
    control->fault_flags = MOTOR_CONTROL_FAULT_NONE;
    motor_control_clear_targets(control);
}

motor_control_status_t motor_control_enable(motor_control_t *control)
{
    if(control == 0)
    {
        return MOTOR_CONTROL_STATUS_NULL;
    }

    if(control->fault_flags != MOTOR_CONTROL_FAULT_NONE)
    {
        control->state = MOTOR_CONTROL_STATE_FAULT;
        motor_control_clear_targets(control);
        return MOTOR_CONTROL_STATUS_FAULT;
    }

    control->state = MOTOR_CONTROL_STATE_ENABLED;
    return MOTOR_CONTROL_STATUS_OK;
}

void motor_control_disable(motor_control_t *control)
{
    if(control == 0)
    {
        return;
    }

    control->state = MOTOR_CONTROL_STATE_DISABLED;
    motor_control_clear_targets(control);
}

void motor_control_set_fault(motor_control_t *control, motor_control_fault_t fault)
{
    if(control == 0)
    {
        return;
    }

    control->fault_flags |= (uint32_t)fault;
    control->state = MOTOR_CONTROL_STATE_FAULT;
    motor_control_clear_targets(control);
}

void motor_control_clear_fault(motor_control_t *control)
{
    if(control == 0)
    {
        return;
    }

    control->fault_flags = MOTOR_CONTROL_FAULT_NONE;
    control->state = MOTOR_CONTROL_STATE_DISABLED;
    motor_control_clear_targets(control);
}

motor_control_status_t motor_control_set_mode(motor_control_t *control, motor_control_mode_t mode)
{
    if(control == 0)
    {
        return MOTOR_CONTROL_STATUS_NULL;
    }

    if(mode > MOTOR_CONTROL_MODE_POSITION)
    {
        return MOTOR_CONTROL_STATUS_RANGE;
    }

    control->mode = mode;
    return MOTOR_CONTROL_STATUS_OK;
}

motor_control_status_t motor_control_set_iq_ref_ma(motor_control_t *control, int32_t iq_ref_ma)
{
    if(control == 0)
    {
        return MOTOR_CONTROL_STATUS_NULL;
    }

    if(control->state != MOTOR_CONTROL_STATE_ENABLED)
    {
        return MOTOR_CONTROL_STATUS_FAULT;
    }

    control->iq_ref_ma = iq_ref_ma;
    return MOTOR_CONTROL_STATUS_OK;
}

motor_control_status_t motor_control_set_speed_ref_rpm(motor_control_t *control, int32_t speed_ref_rpm)
{
    if(control == 0)
    {
        return MOTOR_CONTROL_STATUS_NULL;
    }

    if(control->state != MOTOR_CONTROL_STATE_ENABLED)
    {
        return MOTOR_CONTROL_STATUS_FAULT;
    }

    control->speed_ref_rpm = speed_ref_rpm;
    return MOTOR_CONTROL_STATUS_OK;
}

motor_control_status_t motor_control_set_position_ref_mdeg(motor_control_t *control, int32_t position_ref_mdeg)
{
    if(control == 0)
    {
        return MOTOR_CONTROL_STATUS_NULL;
    }

    if(control->state != MOTOR_CONTROL_STATE_ENABLED)
    {
        return MOTOR_CONTROL_STATUS_FAULT;
    }

    control->position_ref_mdeg = position_ref_mdeg;
    return MOTOR_CONTROL_STATUS_OK;
}

motor_control_state_t motor_control_get_state(const motor_control_t *control)
{
    if(control == 0)
    {
        return MOTOR_CONTROL_STATE_FAULT;
    }

    return control->state;
}

motor_control_mode_t motor_control_get_mode(const motor_control_t *control)
{
    if(control == 0)
    {
        return MOTOR_CONTROL_MODE_OPEN_LOOP;
    }

    return control->mode;
}

uint32_t motor_control_get_fault(const motor_control_t *control)
{
    if(control == 0)
    {
        return MOTOR_CONTROL_FAULT_SENSOR;
    }

    return control->fault_flags;
}

int32_t motor_control_get_iq_ref_ma(const motor_control_t *control)
{
    if(control == 0)
    {
        return 0;
    }

    return control->iq_ref_ma;
}

int32_t motor_control_get_speed_ref_rpm(const motor_control_t *control)
{
    if(control == 0)
    {
        return 0;
    }

    return control->speed_ref_rpm;
}

int32_t motor_control_get_position_ref_mdeg(const motor_control_t *control)
{
    if(control == 0)
    {
        return 0;
    }

    return control->position_ref_mdeg;
}
