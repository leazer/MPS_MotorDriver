#ifndef SPEED_LOOP_H
#define SPEED_LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "current_loop.h"  /* pid_f32_t */

void speed_loop_init(void);
void speed_loop_reset(void);
void speed_loop_set_target_rad_s(float target_rad_s);
float speed_loop_run(float measured_rad_s);
float speed_loop_get_target_rad_s(void);
float speed_loop_get_command_rad_s(void);
float speed_loop_get_measured_rad_s(void);
float speed_loop_get_iq_ref_A(void);

#ifdef __cplusplus
}
#endif

#endif /* SPEED_LOOP_H */
