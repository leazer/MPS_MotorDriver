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

/* Stage 5 新增 */
/* 设置电流环目标 (A). id_ref 通常 0, iq_ref 由 shell/CAN 设置. */
void current_loop_set_targets(float id_ref_A, float iq_ref_A);
/* 清积分 (模式切换/stop 时调用, 避免残留冲击). */
void current_loop_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* CURRENT_LOOP_H */
