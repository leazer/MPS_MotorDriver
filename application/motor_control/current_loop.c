#include "current_loop.h"

float pid_f32_exec(pid_f32_t *pid, float error)
{
    (void)pid; (void)error;
    return 0.0f; /* stub, Plan 4 实现 */
}

void current_loop_init(void)
{
}

void current_loop_run(float id, float iq, float *vd_ref, float *vq_ref)
{
    (void)id; (void)iq;
    *vd_ref = 0.0f;
    *vq_ref = 0.0f;
}
