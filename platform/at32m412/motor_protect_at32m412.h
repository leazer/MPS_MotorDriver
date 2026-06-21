#ifndef MOTOR_PROTECT_AT32M412_H
#define MOTOR_PROTECT_AT32M412_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* nFAULT EXINT 初始化 (PB2, spec §1.2) */
void motor_protect_at32m412_init(void);

/* 读 VBUS ADC (普通转换), 返回电压 V (spec §4.3.6) */
float motor_protect_read_vbus_v(void);

/* 1 kHz 调用: 检查 VBUS 过/欠压, 触发 fault_manager */
void motor_protect_check_vbus(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PROTECT_AT32M412_H */
