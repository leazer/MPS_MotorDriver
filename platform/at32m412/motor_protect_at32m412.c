#include "motor_protect_at32m412.h"
#include "fault_manager.h"
#include "motor_params.h"

void motor_protect_at32m412_init(void) { /* stub, Plan 5 */ }

float motor_protect_read_vbus_v(void)
{
    /* stub: 返回安全中值, Plan 2 接真实 ADC */
    return 12.0f;
}

void motor_protect_check_vbus(void)
{
    float vbus = motor_protect_read_vbus_v();
    if (vbus < VBUS_UNDERVOLTAGE_THRESHOLD_V) {
        fault_manager_set(FAULT_UNDERVOLTAGE);
    } else {
        fault_manager_clear(FAULT_UNDERVOLTAGE);
    }
    if (vbus > VBUS_OVERVOLTAGE_THRESHOLD_V) {
        fault_manager_set(FAULT_OVERVOLTAGE);
    } else {
        fault_manager_clear(FAULT_OVERVOLTAGE);
    }
}
