#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    MOTOR_CONTROL_STATUS_OK = 0,
    MOTOR_CONTROL_STATUS_NULL = -1,
    MOTOR_CONTROL_STATUS_FAULT = -2,
    MOTOR_CONTROL_STATUS_RANGE = -3
} motor_control_status_t;

typedef enum
{
    MOTOR_CONTROL_STATE_DISABLED = 0,
    MOTOR_CONTROL_STATE_ENABLED,
    MOTOR_CONTROL_STATE_FAULT
} motor_control_state_t;

typedef enum
{
    MOTOR_CONTROL_MODE_OPEN_LOOP = 0,
    MOTOR_CONTROL_MODE_CURRENT,
    MOTOR_CONTROL_MODE_SPEED,
    MOTOR_CONTROL_MODE_POSITION,
    MOTOR_CONTROL_MODE_ALIGN      /* spec §4.5.3: 零点对齐, 固定 Vd 锁定转子到 d 轴 0° */
} motor_control_mode_t;

typedef enum
{
    MOTOR_CONTROL_FAULT_NONE = 0,
    MOTOR_CONTROL_FAULT_DRIVER = 1u << 0,
    MOTOR_CONTROL_FAULT_OVERCURRENT = 1u << 1,
    MOTOR_CONTROL_FAULT_SENSOR = 1u << 2,
    MOTOR_CONTROL_FAULT_UNDERVOLTAGE = 1u << 3,
    MOTOR_CONTROL_FAULT_OVERVOLTAGE = 1u << 4,
    MOTOR_CONTROL_FAULT_OVERTEMPERATURE = 1u << 5
} motor_control_fault_t;

typedef struct
{
    motor_control_state_t state;
    motor_control_mode_t mode;
    uint32_t fault_flags;
    int32_t iq_ref_ma;
    int32_t speed_ref_rpm;
    int32_t position_ref_mdeg;
} motor_control_t;

void motor_control_init(motor_control_t *control);
motor_control_status_t motor_control_enable(motor_control_t *control);
void motor_control_disable(motor_control_t *control);
void motor_control_set_fault(motor_control_t *control, motor_control_fault_t fault);
void motor_control_clear_fault(motor_control_t *control);

motor_control_status_t motor_control_set_mode(motor_control_t *control, motor_control_mode_t mode);
motor_control_status_t motor_control_set_iq_ref_ma(motor_control_t *control, int32_t iq_ref_ma);
motor_control_status_t motor_control_set_speed_ref_rpm(motor_control_t *control, int32_t speed_ref_rpm);
motor_control_status_t motor_control_set_position_ref_mdeg(motor_control_t *control, int32_t position_ref_mdeg);

motor_control_state_t motor_control_get_state(const motor_control_t *control);
motor_control_mode_t motor_control_get_mode(const motor_control_t *control);
uint32_t motor_control_get_fault(const motor_control_t *control);
int32_t motor_control_get_iq_ref_ma(const motor_control_t *control);
int32_t motor_control_get_speed_ref_rpm(const motor_control_t *control);
int32_t motor_control_get_position_ref_mdeg(const motor_control_t *control);

#ifdef __cplusplus
}
#endif

#endif
