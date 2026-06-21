#include "current_sense_at32m412.h"
#include "motor_params.h"

void current_sense_at32m412_init(void) { /* stub, Plan 2 */ }
void current_sense_at32m412_read_raw(uint16_t *ia, uint16_t *ib, uint16_t *ic)
{
    *ia = CURRENT_ZERO_OFFSET_LSB;
    *ib = CURRENT_ZERO_OFFSET_LSB;
    *ic = CURRENT_ZERO_OFFSET_LSB;
}
void current_sense_at32m412_calibrate_offset(void) { /* stub */ }

float current_sense_calc(uint16_t raw, float offset_lsb, float gain_a_per_lsb)
{
    return ((float)raw - offset_lsb) * gain_a_per_lsb;
}
