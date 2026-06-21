#include "flash_calibration_at32m412.h"
#include "motor_params.h"
#include <string.h>

bool flash_calibration_read(motor_calibration_t *cal)
{
    /* stub: 直接返回失败, Plan 3 实现 */
    (void)cal;
    return false;
}

bool flash_calibration_write(const motor_calibration_t *cal)
{
    (void)cal;
    return false;
}

bool flash_calibration_erase(void)
{
    return false;
}
