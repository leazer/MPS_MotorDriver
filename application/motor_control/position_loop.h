#ifndef POSITION_LOOP_H
#define POSITION_LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void position_loop_init(void);
void position_loop_run(uint16_t raw_angle);

#ifdef __cplusplus
}
#endif

#endif /* POSITION_LOOP_H */
