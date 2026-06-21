#include "motor_encoder_at32m412.h"
#include "ma600a.h"
#include "ma600a_at32_spi2.h"
#include "motor_calibration.h"
#include "motor_params.h"
#include <math.h>

static ma600a_t s_ma600a;

void motor_encoder_at32m412_init(void)
{
    ma600a_init(&s_ma600a, ma600a_at32_spi2_bus_get());
}

int motor_encoder_read_angle_speed(uint16_t *raw_angle_16, int16_t *raw_speed)
{
    uint16_t raw_12 = 0u;
    int16_t spd = 0;
    if (ma600a_read_angle_and_speed_raw(&s_ma600a, &raw_12, &spd) != 0) {
        return -1;
    }
    *raw_angle_16 = (uint16_t)(raw_12 << 4);  /* 12-bit -> 16-bit 扩展 */
    *raw_speed = spd;
    return 0;
}

float motor_encoder_to_electrical_angle(uint16_t raw_angle_16)
{
    /* stub: 暂不做旁轴标定查表与零点修正, Plan 3 填充 */
    (void)raw_angle_16;
    return 0.0f;
}
