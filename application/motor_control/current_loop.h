#ifndef CURRENT_LOOP_H
#define CURRENT_LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "motor_params.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float integral_limit;
    float out_limit;
    float last_error;
} pid_f32_t;

float pid_f32_exec(pid_f32_t *pid, float error);
void current_loop_init(void);
void current_loop_run(float id, float iq, float *vd_ref, float *vq_ref);

#ifdef __cplusplus
}
#endif

#endif /* CURRENT_LOOP_H */
