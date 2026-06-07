#ifndef MA600A_DEBUG_H
#define MA600A_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern volatile uint16_t g_ma600a_raw_angle;
extern volatile float g_ma600a_angle_deg;
extern volatile int32_t g_ma600a_status;
extern volatile uint32_t g_ma600a_sample_count;
extern volatile uint32_t g_ma600a_error_count;
extern volatile uint8_t g_ma600a_bct;
extern volatile uint8_t g_ma600a_axis;
extern volatile int16_t g_ma600a_speed_raw;
extern volatile float g_ma600a_speed_rpm;

void ma600a_debug_init(void);
void ma600a_debug_poll(void);

#ifdef __cplusplus
}
#endif

#endif
