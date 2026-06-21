#include "motor_calibration.h"
#include "fault_manager.h"

static motor_calibration_t s_cal;
static bool s_cal_valid = false;
static cal_state_t s_cal_state = CAL_STATE_IDLE;

void motor_calibration_load(void)
{
    s_cal_valid = false;  /* stub, Plan 3 (Stage 4b) 实现 FLASH 读取 + CRC */
    fault_manager_set(FAULT_CAL_INVALID);
}

bool motor_calibration_is_valid(void) { return s_cal_valid; }
const motor_calibration_t *motor_calibration_get(void) { return &s_cal; }
void motor_calibration_start(void) { s_cal_state = CAL_STATE_ZERO_ALIGN; }
cal_state_t motor_calibration_get_state(void) { return s_cal_state; }
uint8_t motor_calibration_get_progress(void) { return 0u; }
