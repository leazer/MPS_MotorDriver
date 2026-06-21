#ifndef MOTOR_CONTROL_ISR_H
#define MOTOR_CONTROL_ISR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* FOC ISR 主体, 由 TMR1_OVF 中断调用 (spec §3.1).
 * Stage 2+ 实现真正逻辑, Plan 1 仅空实现.
 */
void motor_control_isr_tick(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_ISR_H */
