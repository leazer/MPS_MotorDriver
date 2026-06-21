#include "motor_control.h"

#include <assert.h>
#include <stdint.h>

static void test_default_state_is_safe(void)
{
    motor_control_t control;
    motor_control_init(&control);

    assert(motor_control_get_state(&control) == MOTOR_CONTROL_STATE_DISABLED);
    assert(motor_control_get_mode(&control) == MOTOR_CONTROL_MODE_OPEN_LOOP);
    assert(motor_control_get_fault(&control) == MOTOR_CONTROL_FAULT_NONE);
    assert(motor_control_get_iq_ref_ma(&control) == 0);
    assert(motor_control_get_speed_ref_rpm(&control) == 0);
    assert(motor_control_get_position_ref_mdeg(&control) == 0);
}

static void test_enable_disable_sequence(void)
{
    motor_control_t control;
    motor_control_init(&control);

    assert(motor_control_enable(&control) == MOTOR_CONTROL_STATUS_OK);
    assert(motor_control_get_state(&control) == MOTOR_CONTROL_STATE_ENABLED);

    motor_control_disable(&control);
    assert(motor_control_get_state(&control) == MOTOR_CONTROL_STATE_DISABLED);
    assert(motor_control_get_iq_ref_ma(&control) == 0);
}

static void test_fault_latches_until_clear(void)
{
    motor_control_t control;
    motor_control_init(&control);

    assert(motor_control_enable(&control) == MOTOR_CONTROL_STATUS_OK);
    motor_control_set_fault(&control, MOTOR_CONTROL_FAULT_DRIVER);

    assert(motor_control_get_state(&control) == MOTOR_CONTROL_STATE_FAULT);
    assert(motor_control_get_fault(&control) == MOTOR_CONTROL_FAULT_DRIVER);
    assert(motor_control_enable(&control) == MOTOR_CONTROL_STATUS_FAULT);

    motor_control_clear_fault(&control);
    assert(motor_control_get_state(&control) == MOTOR_CONTROL_STATE_DISABLED);
    assert(motor_control_get_fault(&control) == MOTOR_CONTROL_FAULT_NONE);
    assert(motor_control_enable(&control) == MOTOR_CONTROL_STATUS_OK);
}

int main(void)
{
    test_default_state_is_safe();
    test_enable_disable_sequence();
    test_fault_latches_until_clear();
    return 0;
}
