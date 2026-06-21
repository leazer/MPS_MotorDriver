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
    FAULT_CAL_INVALID      = 1u << 6,
} motor_fault_t;

void fault_manager_init(void);
void fault_manager_set(uint32_t fault);
void fault_manager_clear(uint32_t fault);
void fault_manager_clear_all(void);
uint32_t fault_manager_get(void);
bool fault_manager_any(void);

#ifdef __cplusplus
}
#endif

#endif /* FAULT_MANAGER_H */
