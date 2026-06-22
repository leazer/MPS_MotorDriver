#ifndef FAULT_MANAGER_H
#define FAULT_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 故障位图 (spec §5.5) */
typedef enum {
    FAULT_NONE             = 0u,
    FAULT_DRIVER           = 1u << 0,  /* MP6540H nFAULT */
    FAULT_OVERCURRENT      = 1u << 1,
    FAULT_SENSOR           = 1u << 2,
    FAULT_UNDERVOLTAGE     = 1u << 3,
    FAULT_OVERVOLTAGE      = 1u << 4,
    FAULT_CAN_TIMEOUT      = 1u << 5,
    FAULT_CAL_INVALID      = 1u << 6,  /* 告警级: 不阻止使能 (spec §5.5/§4.7.3) */
} motor_fault_t;

/* 致命故障掩码: 这些位置位时 ISR 强制关 PWM.
 * FAULT_CAL_INVALID 是告警, 不在此掩码内 (spec §4.7.3: 不阻止电机使能). */
#define FAULT_FATAL_MASK  (FAULT_DRIVER | FAULT_OVERCURRENT | FAULT_SENSOR | \
                           FAULT_UNDERVOLTAGE | FAULT_OVERVOLTAGE)

void fault_manager_init(void);
void fault_manager_set(uint32_t fault);
void fault_manager_clear(uint32_t fault);
void fault_manager_clear_all(void);
uint32_t fault_manager_get(void);
bool fault_manager_any(void);
bool fault_manager_any_fatal(void);  /* 仅 FAULT_FATAL_MASK 内的位 */

#ifdef __cplusplus
}
#endif

#endif /* FAULT_MANAGER_H */
