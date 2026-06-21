#ifndef SPEED_LOOP_H
#define SPEED_LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "current_loop.h"  /* pid_f32_t */

void speed_loop_init(void);
void speed_loop_run(int16_t raw_speed);

#ifdef __cplusplus
}
#endif

#endif /* SPEED_LOOP_H */
